#!/usr/bin/env python3
"""The instrument wrist in the controller, on mock hardware (decision 17, 2026-11-03).

    ARTS_ARM=psm_left ARTS_WRIST=471006_large_needle_driver.yaml \\
        run_servo_smoke_test.sh wrist_mock_test.py

The mock arm runs with wrist_instrument (9 joints: the UR's six plus roll, pitch, yaw) and the
fake instrument stands in for disc_drive (333 Hz, the needle driver's limits). Checks:

  1. Nine joints commanded (joint_command), and set_tool_tip_offset refused: with the wrist
     the tip is the instrument chain's.
  2. Full orientation, RCM off: the tip turned 20 deg about its x, 20 deg about its y, and
     rolled 30 deg about the shaft, position held. The tip reaches each orientation within
     1 deg and stays within 1 mm; the roll is the INSTRUMENT roll joint's. (A's roll row
     asks for exactly that, but on mock the posture term also pulls the UR back, so this
     check does not isolate the roll row; the real arm's transient will.)
  3. Full orientation under the RCM (frame E, 100 mm inserted): a 20 deg wrist turn plus a 5 mm
     lateral tip move. Orientation within 1 deg, position within 1 mm, and the kinematic RCM
     deviation stays small (mock: the controller's own consistency, not a paper number).
  4. Under the RCM (the shaft pinned, so only the wrist can turn the tip off the shaft), a
     target 110 deg about the tip's x drives the pitch INTO its limit (80 deg needle driver, 70 forceps family) and stops there;
     the RCM still holds.
  5. The drive stalls (fake instrument paused): the controller holds and zeroes the wrist
     command; resumed, it follows again.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_io import ARM, JOINT_COMMAND, TARGET_TOPIC, TOOL_TIP_STATE, target
from arts_interfaces.msg import RcmState, ToolTipState
from arts_interfaces.srv import SetRcm, SetToolTipOffset

DEPTH = 0.10
# The wrist_pitch limit of the instrument the arm runs with (ARTS_WRIST, as the wrapper passes it),
# read from its file: the needle driver's is 80 deg, the forceps family's 70.
_WRIST_FILE = os.environ.get('ARTS_WRIST', '471006_large_needle_driver.yaml')
with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config', 'instruments',
                       _WRIST_FILE)) as _f:
    PITCH_MAX = float(yaml.safe_load(_f)['limits']['wrist_pitch']['max'])


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def rot(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def angle_between(r1, r2):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r1.T @ r2) - 1) / 2))))


class Rig(Node):

    def __init__(self):
        super().__init__('wrist_mock_test')
        self.pub = self.create_publisher(
            type(target(None, (0, 0, 0), (0, 0, 0, 1))), TARGET_TOPIC, 10)
        self.tip = None
        self.rcm = None
        self.cmd = None
        self.wrist = None
        self.wrist_cmd = None
        self.dev_log = None
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        self.create_subscription(JointState, JOINT_COMMAND, lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(JointState, f'/{ARM}/tool_joint_states',
                                 lambda m: setattr(self, 'wrist', m), 10)
        self.create_subscription(JointState, f'/{ARM}/tool_joint_target',
                                 lambda m: setattr(self, 'wrist_cmd', m), 10)
        self.set_rcm = self.create_client(SetRcm, f'/{ARM}/set_rcm')
        self.set_offset = self.create_client(SetToolTipOffset, f'/{ARM}/set_tool_tip_offset')
        self.set_param = self.create_client(SetParameters, f'/{ARM}/fake_instrument/set_parameters')

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), quat_to_matrix((o.x, o.y, o.z, o.w)))

    def _on_rcm(self, m):
        self.rcm = m
        if self.dev_log is not None:
            self.dev_log.append(m.deviation)

    def call(self, client, req, timeout=3.0):
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f'{client.srv_name}: no response')
        return fut.result()

    def hold_target(self, p, r, seconds):
        q = matrix_to_quat(r)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(target(self.get_clock().now().to_msg(), p, q))
            time.sleep(0.004)

    def wrist_joint(self, short):
        for n, q in zip(self.wrist.name, self.wrist.position):
            if n.endswith(short):
                return q
        return float('nan')

    def pause(self, on):
        v = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on)
        self.call(self.set_param, SetParameters.Request(parameters=[Parameter(name='paused', value=v)]))


def main():
    rclpy.init()
    n = Rig()
    ex = SingleThreadedExecutor()
    ex.add_node(n)

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()
    failures = []

    def check(ok, what):
        print(('PASS ' if ok else 'FAIL ') + what)
        if not ok:
            failures.append(what)

    for c in (n.set_rcm, n.set_offset, n.set_param):
        if not c.wait_for_service(timeout_sec=10.0):
            sys.exit(f'{c.srv_name} not available')
    time.sleep(1.5)
    if n.tip is None or n.wrist is None or n.cmd is None:
        sys.exit('no tool_tip_state / tool_joint_states / joint_command')

    # 1. nine joints, offset refused
    check(len(n.cmd.name) == 9 and n.cmd.name[-1].endswith('tool_wrist_yaw'),
          f'joint_command carries 9 joints ({n.cmd.name[-3:]})')
    req = SetToolTipOffset.Request()
    req.offset = Pose()
    req.offset.orientation.w = 1.0
    r = n.call(n.set_offset, req)
    check(not r.success, f'set_tool_tip_offset refused with the wrist ({r.message})')

    # 2. full orientation, RCM off
    p0, r0 = n.tip
    for label, axis, ang in (('x', (1, 0, 0), 20), ('y', (0, 1, 0), 20), ('z (roll)', (0, 0, 1), 30)):
        r_t = r0 @ rot(axis, math.radians(ang))
        n.hold_target(p0, r_t, 3.0)
        e_ang = angle_between(n.tip[1], r_t)
        e_pos = 1e3 * np.linalg.norm(n.tip[0] - p0)
        check(e_ang < 1.0 and e_pos < 1.0,
              f'RCM off, {ang} deg about tip {label}: orientation error {e_ang:.3f} deg, position '
              f'{e_pos:.3f} mm')
        if label.startswith('z'):
            roll = math.degrees(n.wrist_joint('tool_roll'))
            check(abs(roll - 30.0) < 2.0,
                  f'the roll is the instrument roll joint\'s: {roll:.2f} deg of 30 (A\'s roll row)')
        n.hold_target(p0, r0, 3.0)  # back

    # 3. under the RCM
    p0, r0 = n.tip
    rq = SetRcm.Request(enable=True, from_current_shaft=True, insertion_depth=DEPTH)
    r = n.call(n.set_rcm, rq)
    check(r.success, f'RCM on at frame E, {DEPTH * 1e3:.0f} mm ({r.message})')
    time.sleep(0.3)
    lateral = r0 @ np.array([1.0, 0, 0]) * 0.005
    r_t = r0 @ rot((0, 1, 0), math.radians(20))
    n.dev_log = []
    n.hold_target(p0 + lateral, r_t, 4.0)
    devs, n.dev_log = np.array(n.dev_log) * 1e3, None
    e_ang = angle_between(n.tip[1], r_t)
    e_pos = 1e3 * np.linalg.norm(n.tip[0] - (p0 + lateral))
    check(e_ang < 1.0 and e_pos < 1.0,
          f'RCM on, 20 deg wrist turn + 5 mm lateral: orientation {e_ang:.3f} deg, position '
          f'{e_pos:.3f} mm')
    check(len(devs) > 0 and devs.max() < 0.05,
          f'RCM held: deviation median {np.median(devs):.4f} mm, max {devs.max():.4f} mm '
          f'(n={len(devs)})')
    n.hold_target(p0, r0, 3.0)
    n.call(n.set_rcm, SetRcm.Request(enable=False))
    time.sleep(0.3)

    # 4. pitch into its limit, under the RCM
    p0, r0 = n.tip
    r = n.call(n.set_rcm, SetRcm.Request(enable=True, from_current_shaft=True,
                                         insertion_depth=DEPTH))
    time.sleep(0.3)
    n.dev_log = []
    n.hold_target(p0, r0 @ rot((1, 0, 0), math.radians(110)), 5.0)
    devs, n.dev_log = np.array(n.dev_log) * 1e3, None
    pitch = abs(n.wrist_joint('tool_wrist_pitch'))
    check(PITCH_MAX - math.radians(5) < pitch <= PITCH_MAX + 0.01,
          f'RCM on, 110 deg about tip x: pitch driven into its limit, {math.degrees(pitch):.2f} '
          f'deg (limit {math.degrees(PITCH_MAX):.1f}); RCM max deviation {devs.max():.4f} mm')
    n.hold_target(p0, r0, 5.0)
    n.call(n.set_rcm, SetRcm.Request(enable=False))
    time.sleep(0.3)

    # 5. stalled drive
    p0, r0 = n.tip
    n.pause(True)
    time.sleep(0.3)
    n.hold_target(p0, r0 @ rot((0, 1, 0), math.radians(20)), 1.5)
    moved = angle_between(n.tip[1], r0)
    v = max(abs(x) for x in n.wrist_cmd.velocity) if n.wrist_cmd else float('nan')
    check(moved < 0.5 and v == 0.0,
          f'drive stalled: arm held ({moved:.3f} deg), wrist command zero ({v})')
    n.pause(False)
    time.sleep(0.3)
    r_t = r0 @ rot((0, 1, 0), math.radians(20))
    n.hold_target(p0, r_t, 3.0)
    e_ang = angle_between(n.tip[1], r_t)
    check(e_ang < 1.0, f'drive resumed: follows again ({e_ang:.3f} deg)')

    rclpy.shutdown()
    print('\nPASS' if not failures else f'\n{len(failures)} FAILED: {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
