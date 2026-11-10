#!/usr/bin/env python3
"""Teleop follower with orientation (collaborator A's design) against the 9-joint mock arm.

    ARTS_ARM=psm_left ARTS_WRIST=471006_large_needle_driver.yaml \\
        arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/follower_orientation_mock_test.py

The follower runs in clutch mode (A's: pedal UP = active), orientation on, no camera frame
(the screen maps to the base through operator_to_base = identity, haply_to_operator =
identity), left hand. A scripted handle and VerseGrip; the stylus orientation is sent as the
raw value that maps to the wanted screen orientation (A's fixed Rz(-90) for the left pen).
A's sequence:

  [1] enabled, clutch pedal up, stylus 30 deg off the instrument: NOT engaged (the gate),
      the arm holds, teleop_alignment reports ~30 deg
  [2] clutch pedal down + VerseGrip button b held ("bring to stylus"): the arm turns the
      instrument to the stylus (alignment -> ~0), position held
  [3] b released, clutch pedal up: engages through the gate
  [4] stylus turned 20 deg: the tip turns with it (absolute, 1:1)
  [5] handle +40 mm: the tip moves +10 mm (scale 0.25), orientation kept
  [5b] stylus turned 70 deg further: the jaw axis stops at the 60 deg bend limit from the shaft
       (tool0 +y; RCM off), the arm does not chase the rest
  [6] clutch pedal down, stylus turned again: nothing moves
  [7] still disengaged, VerseGrip button a held ("straighten"): the jaw axis returns onto the
      shaft, position held
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from arts_interfaces.msg import StylusState, TeleopState, ToolTipState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

FAILS = []
S_LEFT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # Rz(-90)


def quat_to_matrix(x, y, z, w):
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


def rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = math.radians(deg)
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def ang(r1, r2):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r1.T @ r2) - 1) / 2))))


class Rig(Node):

    def __init__(self):
        super().__init__('follower_orientation_mock_test')
        self.cur = self.create_publisher(PointStamped, '/mtm_left/cursor', 10)
        self.vel = self.create_publisher(Vector3Stamped, '/mtm_left/cursor_velocity', 10)
        self.sty = self.create_publisher(StylusState, '/mtm_left/stylus_state', 10)
        self.joy = self.create_publisher(Joy, '/console/footswitch_teleop', 10)
        self.enable = self.create_client(SetBool, '/teleop_follower/enable')
        self.tip = None
        self.align = None
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.create_subscription(ToolTipState, '/psm_left/tool_tip_state', self._on_tip, 10)
        self.create_subscription(Float64, '/psm_left/teleop_alignment',
                                 lambda m: setattr(self, 'align', m.data), 10)
        self.engaged = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TeleopState, '/psm_left/teleop_state',
                                 lambda m: setattr(self, 'engaged', m.clutch_engaged), latched)
        self.h = np.array([0.0, 0.0, 0.10])
        self.r_screen = np.eye(3)   # the stylus orientation we want in the screen frame
        self.buttons = [0, 0, 0]
        self.b = False              # VerseGrip button b
        self.a = False              # VerseGrip button a
        self.create_timer(0.002, self._tick)
        self.create_timer(0.02, self._tick_slow)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), quat_to_matrix(o.x, o.y, o.z, o.w))

    def _tick(self):
        now = self.get_clock().now().to_msg()
        m = PointStamped()
        m.header.stamp, m.header.frame_id = now, 'mtm_left_base'
        m.point.x, m.point.y, m.point.z = map(float, self.h)
        self.cur.publish(m)
        v = Vector3Stamped()
        v.header.stamp = now
        self.vel.publish(v)

    def _tick_slow(self):
        s = StylusState()
        s.header.stamp = self.get_clock().now().to_msg()
        raw = S_LEFT.T @ self.r_screen  # M = I, D = I, C = I
        (s.orientation.x, s.orientation.y, s.orientation.z, s.orientation.w) = matrix_to_quat(raw)
        s.buttons = [self.a, self.b, False]
        self.sty.publish(s)
        j = Joy()
        j.header.stamp = s.header.stamp
        j.buttons = list(self.buttons)
        self.joy.publish(j)


def spin_for(node, s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.001)


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
         '--ros-args', '-p', 'arm:=psm_left', '-p', 'mtm:=mtm_left', '-p', 'mode:=clutch',
         '-p', 'orientation:=true', '-p', 'scale:=0.25', '-p', 'max_offset:=0.05',
         '-p', f'mapping_file:={mapping.name}'])
    rclpy.init()
    rig = Rig()
    try:
        spin_for(rig, 4.0)
        p0, r0 = rig.tip
        rig.r_screen = r0 @ rot((1, 0, 0), 30)  # 30 deg off the instrument
        fut = rig.enable.call_async(SetBool.Request(data=True))
        while not fut.done():
            rclpy.spin_once(rig, timeout_sec=0.01)

        print('[1] enabled, pedal up, stylus 30 deg off')
        spin_for(rig, 1.5)
        moved = np.linalg.norm(rig.tip[0] - p0) * 1e3
        turned = ang(rig.tip[1], r0)
        check('not engaged, the arm holds', moved < 0.1 and turned < 0.1,
              f'{moved:.3f} mm, {turned:.3f} deg')
        check('alignment reported', rig.align is not None and abs(rig.align - 30) < 1,
              f'{rig.align}')
        check('teleop_state NOT engaged while the gate is closed', rig.engaged is False,
              f'clutch_engaged {rig.engaged}')

        print('[2] clutch down + button b: the arm turns the instrument to the stylus')
        rig.buttons = [1, 0, 0]
        rig.b = True
        spin_for(rig, 3.0)
        check('aligned', rig.align is not None and rig.align < 1.0, f'{rig.align:.3f} deg')
        moved = np.linalg.norm(rig.tip[0] - p0) * 1e3
        check('position held while aligning', moved < 1.0, f'{moved:.3f} mm')

        print('[3] b released, clutch up: engages')
        rig.b = False
        rig.buttons = [0, 0, 0]
        spin_for(rig, 1.0)
        p1, r1 = rig.tip
        check('teleop_state engaged once aligned', rig.engaged is True,
              f'clutch_engaged {rig.engaged}')

        print('[4] stylus turned 20 deg about the screen y')
        want = rot((0, 1, 0), 20) @ rig.r_screen
        for k in range(1, 51):  # over 1 s
            rig.r_screen = rot((0, 1, 0), 20 * k / 50) @ (r0 @ rot((1, 0, 0), 30))
            spin_for(rig, 0.02)
        spin_for(rig, 1.5)
        check('tip follows the stylus (absolute, 1:1)', ang(rig.tip[1], want) < 1.0,
              f'{ang(rig.tip[1], want):.3f} deg from the stylus; turned {ang(rig.tip[1], r1):.2f} '
              'deg')
        check('position held', np.linalg.norm(rig.tip[0] - p1) * 1e3 < 1.0,
              f'{np.linalg.norm(rig.tip[0] - p1) * 1e3:.3f} mm')

        print('[5] handle +40 mm x')
        p2, r2 = rig.tip
        start = rig.h.copy()
        for k in range(1, 251):
            rig.h = start + np.array([0.04 * k / 250, 0, 0])
            spin_for(rig, 0.004)
        spin_for(rig, 1.0)
        d = (rig.tip[0] - p2) * 1e3
        check('tip +10 mm x (scale 0.25)', abs(d[0] - 10.0) < 1.0 and np.linalg.norm(d[1:]) < 1.0,
              f'{np.round(d, 2)} mm')
        check('orientation kept', ang(rig.tip[1], r2) < 1.0, f'{ang(rig.tip[1], r2):.3f} deg')

        print('[5b] stylus bent 70 deg further away from the shaft: held at the bend limit')
        def shaft_and_bend():
            q = rig.tf.lookup_transform('psm_left_base_link', 'psm_left_tool0',
                                        rclpy.time.Time()).transform.rotation
            sh = quat_to_matrix(q.x, q.y, q.z, q.w)[:, 1]
            return sh, math.degrees(math.acos(max(-1.0, min(1.0, float(rig.tip[1][:, 2] @ sh)))))
        shaft, bend0 = shaft_and_bend()
        away = np.cross(shaft, rig.tip[1][:, 2])  # turning about it bends the jaw axis away
        r_start = rig.r_screen.copy()
        for k in range(1, 101):  # over 2 s
            rig.r_screen = rot(away, 70 * k / 100) @ r_start
            spin_for(rig, 0.02)
        spin_for(rig, 1.5)
        _, bend = shaft_and_bend()
        check('jaw axis held at the 60 deg bend limit', 58.0 < bend < 60.5,
              f'{bend:.2f} deg from the shaft (was {bend0:.1f}, stylus asks ~{bend0 + 70:.0f})')

        print('[6] clutch down, stylus turned again')
        rig.buttons = [1, 0, 0]
        spin_for(rig, 0.5)
        p3, r3 = rig.tip
        rig.r_screen = rot((0, 0, 1), 20) @ rig.r_screen
        spin_for(rig, 1.5)
        check('nothing moves', ang(rig.tip[1], r3) < 0.2 and
              np.linalg.norm(rig.tip[0] - p3) * 1e3 < 0.2,
              f'{ang(rig.tip[1], r3):.3f} deg, {np.linalg.norm(rig.tip[0] - p3) * 1e3:.3f} mm')

        print('[7] disengaged, button a held: straighten')
        p4 = rig.tip[0].copy()
        _, bend4 = shaft_and_bend()
        rig.a = True
        spin_for(rig, 3.0)
        rig.a = False
        spin_for(rig, 0.5)
        _, bend5 = shaft_and_bend()
        check('straightened: jaw axis on the shaft', bend5 < 1.0,
              f'{bend4:.1f} -> {bend5:.2f} deg from the shaft')
        check('position held while straightening', np.linalg.norm(rig.tip[0] - p4) * 1e3 < 1.0,
              f'{np.linalg.norm(rig.tip[0] - p4) * 1e3:.3f} mm')
    finally:
        follower.send_signal(2)
        try:
            follower.wait(timeout=5)
        except subprocess.TimeoutExpired:
            follower.kill()
        os.unlink(mapping.name)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
