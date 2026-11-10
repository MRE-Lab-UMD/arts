#!/usr/bin/env python3
"""set_rcm with the instrument wrist BENT, on mock hardware (the 2026-11-07 fix).

    ARTS_ARM=psm_left ARTS_WRIST=470093_prograsp_forceps.yaml \\
        run_servo_smoke_test.sh $PWD/rcm_bent_wrist_mock_test.py

Before the fix, set_rcm placed the point `depth` back along the TIP's z (the jaw axis), while
every control cycle measures it against the SHAFT (frame E); with the wrist bent by theta the
two differ by depth * sin(theta), and the controller latched an RCM fault in the same cycle
(real arm: 12.45 mm at ~8 deg). Checks:
  1. The tip is turned 20 deg about its x with the position held: the wrist pitch bends by
     more than 5 deg.
  2. set_rcm on (100 mm) succeeds, and the reply point lies on the SHAFT line (< 0.1 mm), and
     NOT on the jaw-axis line (> 5 mm: the failure path the old placement would take).
  3. For the next 2 s every rcm_state says constraint_ok, deviation < 0.05 mm.
The shaft frame is the tip pose taken back through the measured wrist angles with the
controller's wrist geometry (tip = shaft * Rx(pitch) * T(0, 0, pitch_to_yaw) * Ry(-yaw)).
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from arts_interfaces.msg import RcmState, ToolTipState
from arts_interfaces.srv import SetRcm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_io import ARM, TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget, target  # noqa: E402

PITCH_TO_YAW = 0.009
FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


def line_dist(p, origin, axis):
    v = p - origin
    return float(np.linalg.norm(v - (v @ axis) * axis))


class Rig(Node):

    def __init__(self):
        super().__init__('rcm_bent_wrist_mock_test')
        self.tip = None
        self.wrist = None
        self.rcm = []
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(JointState, f'/{ARM}/tool_joint_states', self._on_j, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', lambda m: self.rcm.append(m), 50)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.set_rcm = self.create_client(SetRcm, f'/{ARM}/set_rcm')

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))

    def _on_j(self, m):
        j = dict(zip(m.name, m.position))
        p, y = j.get(f'{ARM}_tool_wrist_pitch'), j.get(f'{ARM}_tool_wrist_yaw')
        if p is not None and y is not None:
            self.wrist = (p, y)

    def call(self, req):
        fut = self.set_rcm.call_async(req)
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result()


def main():
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    rig.set_rcm.wait_for_service(timeout_sec=10.0)
    time.sleep(2.0)
    try:
        p0, r0 = rig.tip
        pitch0 = rig.wrist[0]
        print('[1] tip turned 20 deg about its x, position held')
        goal = r0 * Rotation.from_euler('x', math.radians(20.0))
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            rig.pub.publish(target(rig.get_clock().now().to_msg(), p0, goal.as_quat()))
            time.sleep(0.008)
        time.sleep(0.5)
        pitch, yaw = rig.wrist
        check(abs(math.degrees(pitch - pitch0)) > 5.0,
              f'wrist bent: pitch {math.degrees(pitch):+.1f} deg, yaw {math.degrees(yaw):+.1f} deg')

        print('[2] set_rcm on with the wrist bent')
        pt, rt = rig.tip
        r_s = rt * (Rotation.from_euler('x', pitch) * Rotation.from_euler('y', -yaw)).inv()
        o_s = pt - (r_s * Rotation.from_euler('x', pitch)).apply([0.0, 0.0, PITCH_TO_YAW])
        shaft_axis = r_s.apply([0.0, 0.0, 1.0])
        jaw_axis = rt.apply([0.0, 0.0, 1.0])
        rig.rcm.clear()
        r = rig.call(SetRcm.Request(enable=True, from_current_shaft=True, insertion_depth=0.10))
        check(r is not None and r.success, f'RCM on ({r.message if r else "no response"})')
        c = np.array([r.applied_point.x, r.applied_point.y, r.applied_point.z])
        d_shaft = 1e3 * line_dist(c, o_s, shaft_axis)
        d_jaw = 1e3 * line_dist(c, pt, jaw_axis)
        check(d_shaft < 0.1, f'point on the shaft line: {d_shaft:.3f} mm')
        check(d_jaw > 5.0, f'point off the jaw-axis line (the old placement): {d_jaw:.2f} mm')

        print('[3] 2 s of rcm_state')
        time.sleep(2.0)
        msgs = [m for m in rig.rcm if m.enabled]
        ok = sum(m.constraint_ok for m in msgs)
        dev = max((m.deviation for m in msgs), default=float('nan'))
        check(msgs and ok == len(msgs), f'constraint_ok {ok}/{len(msgs)}')
        check(dev < 5e-5, f'deviation max {1e3 * dev:.4f} mm')
    finally:
        rig.call(SetRcm.Request(enable=False))
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    return 1 if FAILS else 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)
