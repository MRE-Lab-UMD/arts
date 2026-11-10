#!/usr/bin/env python3
"""Teleop follower on a mock UR5e: scripted handle + pedal, measured arm motion.

Run through the mock-arm wrapper, which brings up the arm and tears it down:

    arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/teleop_follower_mock_test.py

Starts teleop_follower itself with deadman mode, scale 0.25, a 50 mm box, haply_to_operator
identity and operator_to_base a 90 deg rotation about z (handle +x -> arm +y), so a wrong axis
mapping cannot pass. It plays
the MTM bridge (/mtm_left/cursor + cursor_velocity at 500 Hz) and the footswitch
(/console/footswitch_teleop), and measures the arm from tf psm_left_base_link -> psm_left_tool0. Checks, in order:
  1. pedal up: handle moves 20 mm, arm does not move
  2. pedal down: handle +40 mm along x -> arm +10 mm along +y, nothing along x or z
  3. pedal up: handle moves on, arm stays
  4. pedal down again: no jump at re-engagement; handle -40 mm -> arm back 10 mm
  5. handle +400 mm: arm stops at the 50 mm box edge
  6. handle velocity 1.0 m/s: drop guard disengages; arm then ignores the handle until the
     pedal is pressed again
  7. cursor stream stops with the pedal down: arm holds
  8. UR safeguard stop (ARM HOLD) with the pedal down: the handle moving during it and after
     it clears does not move the arm; a fresh pedal press re-engages without a jump
  9. RCM switched on with the pedal down (set_rcm, 100 mm behind the tip): the follower
     disengages; after a fresh press it re-engages without a jump and teleoperates UNDER the
     RCM (deviation from /psm_left/rcm_state stays < 0.1 mm); switching it off disengages
     again
"""
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, RcmState
from arts_interfaces.srv import SetRcm

R_TEST = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # handle +x -> arm +y
SCALE = 0.25
BOX = 0.05


class Rig(Node):

    def __init__(self):
        super().__init__('teleop_follower_mock_test')
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cur = self.create_publisher(PointStamped, '/mtm_left/cursor', 10)
        self.vel = self.create_publisher(Vector3Stamped, '/mtm_left/cursor_velocity', 10)
        self.joy = self.create_publisher(Joy, '/console/footswitch_teleop', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status = self.create_publisher(ArmStatus, '/psm_left/status', latched)
        self.set_rcm = self.create_client(SetRcm, '/psm_left/set_rcm')
        self.max_dev = 0.0
        self.create_subscription(RcmState, '/psm_left/rcm_state', self._on_rcm, 10)
        self.h = np.array([0.0, -0.05, 0.10])
        self.v = 0.0
        self.streaming = True
        self.create_timer(0.002, self._tick)

    def _tick(self):
        if not self.streaming:
            return
        now = self.get_clock().now().to_msg()
        m = PointStamped()
        m.header.stamp, m.header.frame_id = now, 'mtm_left_base'
        m.point.x, m.point.y, m.point.z = map(float, self.h)
        self.cur.publish(m)
        v = Vector3Stamped()
        v.header.stamp = now
        v.vector.x = float(self.v)
        self.vel.publish(v)

    def _on_rcm(self, m):
        if m.enabled:
            self.max_dev = max(self.max_dev, m.deviation)

    def rcm(self, enable):
        req = SetRcm.Request(enable=enable, from_current_shaft=True, insertion_depth=0.10)
        fut = self.set_rcm.call_async(req)
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result()

    def pedal(self, down):
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.buttons = [int(down), 0, 0]
        self.joy.publish(j)

    def arm(self):
        t = self.tf.lookup_transform('psm_left_base_link', 'psm_left_tool0', Time())
        p = t.transform.translation
        return np.array([p.x, p.y, p.z])


def spin_for(node, s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.001)


def move(node, d, seconds):
    """Move the handle by d [m] in a straight line over `seconds`."""
    start, n = node.h.copy(), max(int(seconds / 0.002), 1)
    for k in range(1, n + 1):
        node.h = start + np.asarray(d) * k / n
        spin_for(node, 0.002)


FAILS = []


def check(name, ok, detail):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        FAILS.append(name)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mapping = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    mapping.write('footswitch_teleop:\n  device: none\n  pedals: {}\n'
                  '  actions: {left: clutch, middle: camera, right: record_toggle}\n')
    mapping.close()
    follower = subprocess.Popen(
        [sys.executable, os.path.join(here, '..', 'scripts', 'teleop_follower.py'),
         '--ros-args', '-p', 'arm:=psm_left', '-p', 'mtm:=mtm_left', '-p', 'mode:=deadman',
         '-p', f'scale:={SCALE}', '-p', f'max_offset:={BOX}',
         '-p', f'operator_to_base:={R_TEST}', '-p', f'mapping_file:={mapping.name}'])
    rclpy.init()
    rig = Rig()
    try:
        spin_for(rig, 4.0)  # follower up, tf filled
        a0 = rig.arm()
        print(f'arm start {np.round(a0 * 1e3, 1)} mm')

        print('[1] pedal up, handle +20 mm x')
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a0) * 1e3
        check('no motion without the pedal', d < 0.1, f'{d:.3f} mm')

        print('[2] pedal down, handle +40 mm x over 1 s')
        rig.pedal(True)
        spin_for(rig, 0.3)
        a1 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a1) * 1e3
        check('arm +10 mm along +y', abs(d[1] - 10.0) < 0.5, f'dy {d[1]:.2f} mm')
        check('nothing along x, z', abs(d[0]) < 0.5 and abs(d[2]) < 0.5,
              f'dx {d[0]:.2f}, dz {d[2]:.2f} mm')

        print('[3] pedal up, handle +40 mm x')
        rig.pedal(False)
        spin_for(rig, 0.2)
        a2 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a2) * 1e3
        check('arm stays after release', d < 0.2, f'{d:.3f} mm')

        print('[4] pedal down again, then handle -40 mm x')
        rig.pedal(True)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a2) * 1e3
        check('no jump at re-engagement', d < 0.2, f'{d:.3f} mm')
        move(rig, [-0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a2) * 1e3
        check('arm back 10 mm along -y', abs(d[1] + 10.0) < 0.5, f'dy {d[1]:.2f} mm')

        print('[5] handle +400 mm x: box edge')
        move(rig, [0.40, 0, 0], 2.0)
        spin_for(rig, 1.5)
        off = (rig.arm() - a1) * 1e3  # box is centred on the FIRST engagement pose
        check('stops at the 50 mm box edge', abs(off[1] - BOX * 1e3) < 0.5,
              f'offset y {off[1]:.2f} mm')
        move(rig, [-0.40, 0, 0], 1.0)  # back inside the box
        spin_for(rig, 2.0)

        print('[6] drop guard')
        rig.v = 1.0
        spin_for(rig, 0.1)
        rig.v = 0.0
        a3 = rig.arm()
        move(rig, [-0.04, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a3) * 1e3
        check('disengaged after a 1.0 m/s handle', d < 0.2, f'{d:.3f} mm')
        rig.pedal(False)
        spin_for(rig, 0.1)
        rig.pedal(True)
        spin_for(rig, 0.3)
        a4 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a4) * 1e3
        check('pedal press clears it', abs(d[1] - 10.0) < 0.5, f'dy {d[1]:.2f} mm')

        print('[7] cursor stream stops, pedal down')
        rig.streaming = False
        spin_for(rig, 0.3)
        a5 = rig.arm()
        spin_for(rig, 1.0)
        d = np.linalg.norm(rig.arm() - a5) * 1e3
        check('arm holds without a stream', d < 0.1, f'{d:.3f} mm')

        print('[8] safeguard stop with the pedal down')
        rig.streaming = True
        rig.pedal(False)
        spin_for(rig, 0.2)
        rig.pedal(True)
        spin_for(rig, 0.3)
        rig.status.publish(ArmStatus(safety_mode=SafetyMode.SAFEGUARD_STOP, program_running=True))
        spin_for(rig, 0.3)
        a6 = rig.arm()
        move(rig, [-0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a6) * 1e3
        check('no motion during the stop', d < 0.2, f'{d:.3f} mm')
        rig.status.publish(ArmStatus(safety_mode=SafetyMode.NORMAL, program_running=True))
        spin_for(rig, 0.3)
        a7 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a7) * 1e3
        check('still disengaged after the stop clears', d < 0.2, f'{d:.3f} mm')
        a6 = rig.arm()
        rig.pedal(False)
        spin_for(rig, 0.1)
        rig.pedal(True)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a6) * 1e3
        check('no jump at re-engagement after the stop', d < 0.2, f'{d:.3f} mm')
        move(rig, [-0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a6) * 1e3
        check('follows again after a fresh press', abs(d[1] + 10.0) < 0.5, f'dy {d[1]:.2f} mm')

        print('[9] RCM switched on with the pedal down')
        r = rig.rcm(True)
        check('RCM on', r is not None and r.success, r.message if r else 'no response')
        spin_for(rig, 0.3)
        a8 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a8) * 1e3
        check('disengaged by the switch', d < 0.2, f'{d:.3f} mm')
        rig.pedal(False)
        spin_for(rig, 0.1)
        rig.pedal(True)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a8) * 1e3
        check('no jump at re-engagement under the RCM', d < 0.2, f'{d:.3f} mm')
        rig.max_dev = 0.0
        move(rig, [-0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a8) * 1e3
        check('teleop under the RCM follows', abs(d[1] + 10.0) < 0.5, f'dy {d[1]:.2f} mm')
        check('RCM held during teleop', rig.max_dev < 1e-4,
              f'max deviation {rig.max_dev * 1e3:.4f} mm')
        r = rig.rcm(False)
        spin_for(rig, 0.3)
        a9 = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = np.linalg.norm(rig.arm() - a9) * 1e3
        check('switching off disengages again', r.success and d < 0.2, f'{d:.3f} mm')
    finally:
        follower.terminate()
        follower.wait(5)
        os.unlink(mapping.name)
        rig.destroy_node()
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\nFAIL: {FAILS}')
    sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
    main()
