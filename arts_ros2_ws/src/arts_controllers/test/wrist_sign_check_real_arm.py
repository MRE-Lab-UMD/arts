#!/usr/bin/env python3
"""E9 sign check on a REAL arm with the instrument wrist in the controller (wrist.enabled).

One small rotation of the tool tip about ONE axis of the tip frame, position fixed, then back:
the operator watches the tip on screen and names the direction; the script prints how far the
tip turned (from <arm>/tool_tip_state), how much of it the instrument joints took (from
<arm>/tool_joint_states) and how far the UR joints moved. Like first_motion_real_arm.py:

  1. holds HOLD s with no command; the tip must not move more than 0.5 mm;
  2. rotates the target ANGLE about tip x, y or z (z = the roll about the shaft at zero
     wrist angles) at RATE, holds 3 s, rotates back, holds 3 s;
  3. aborts -- stops publishing, so the controller holds after its 0.1 s reference timeout --
     when the tip is more than ABORT_RADIUS from its start or a UR joint more than ABORT_JOINT
     from its start.

    python3 wrist_sign_check_real_arm.py --axis x [--angle 5] [--rate 5] [--hold 5]

Run it with the arm under External Control, disc_drive running with the instrument zeroed, the
arm launched with wrist_instrument:=<file>, and a hand on the e-stop.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from arm_io import ARM, BASE_FRAME, JOINT_STATES, TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget
from arts_interfaces.msg import ToolTipState
from rclpy.node import Node
from sensor_msgs.msg import JointState

ABORT_RADIUS = 0.015         # m
ABORT_JOINT = math.radians(10.0)
PUBLISH_PERIOD = 0.02        # s


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def axis_angle(axis, angle):
    s = math.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)])


def angle_between(a, b):
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))


class SignCheck(Node):
    def __init__(self):
        super().__init__('wrist_sign_check_real_arm')
        self.tip = None
        self.tip_time = 0.0
        self.q = None
        self.wrist = {}
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(JointState, JOINT_STATES, self._on_js, 10)
        self.create_subscription(JointState, f'/{ARM}/tool_joint_states', self._on_wrist, 10)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)

    def _on_tip(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))
        self.tip_time = time.time()

    def _on_js(self, msg):
        self.q = np.array(msg.position[:6])

    def _on_wrist(self, msg):
        self.wrist = {n.replace(f'{ARM}_tool_', ''): q for n, q in zip(msg.name, msg.position)}

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def tool(self):
        if self.tip is None or time.time() - self.tip_time > 0.2:
            raise RuntimeError(f'no fresh {TOOL_TIP_STATE}')
        return self.tip

    def send(self, p, quat):
        msg = ToolTipTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, quat)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--axis', required=True, choices=['x', 'y', 'z'], help='tip-frame axis')
    ap.add_argument('--angle', type=float, default=5.0, help='deg, at most 15')
    ap.add_argument('--rate', type=float, default=5.0, help='deg/s')
    ap.add_argument('--hold', type=float, default=5.0, help='s of no command first')
    args = ap.parse_args()
    if not 0 < args.angle <= 15.0:
        sys.exit('--angle must be in (0, 15] deg')

    rclpy.init()
    node = SignCheck()
    node.spin_for(3.0)
    if node.q is None or node.tip is None or not node.wrist:
        sys.exit(f'no {JOINT_STATES}, {TOOL_TIP_STATE} or /{ARM}/tool_joint_states')
    p0, q_tip0 = node.tool()
    q0 = node.q.copy()
    w0 = dict(node.wrist)

    def guard():
        p, _ = node.tool()
        if np.linalg.norm(p - p0) > ABORT_RADIUS:
            return f'tip {1000 * np.linalg.norm(p - p0):.1f} mm from start'
        if np.abs(node.q - q0).max() > ABORT_JOINT:
            return f'a UR joint moved {math.degrees(np.abs(node.q - q0).max()):.1f} deg'
        return None

    node.spin_for(args.hold)
    drift = np.linalg.norm(node.tool()[0] - p0)
    print(f'[hold] {args.hold:.0f} s, no command: tip moved {1000 * drift:.3f} mm')
    if drift > 5e-4:
        print('ABORT: the arm moved without a command')
        return 1

    axis = np.eye(3)['xyz'.index(args.axis)]
    total = math.radians(args.angle)
    duration = args.angle / args.rate
    for label, a_from, a_to in (('+', 0.0, total), ('back', total, 0.0)):
        began = time.time()
        while time.time() - began < duration + 3.0:
            s = min(1.0, (time.time() - began) / duration)
            a = a_from + s * (a_to - a_from)
            node.send(p0, qmul(q_tip0, axis_angle(axis, a)))  # about the TIP's own axis
            node.spin_for(PUBLISH_PERIOD)
            why = guard()
            if why:
                print(f'ABORT: {why}; stopped publishing, the controller holds')
                return 1
        p, q_tip = node.tool()
        turned = math.degrees(angle_between(q_tip, q_tip0))
        dw = {k: math.degrees(node.wrist.get(k, 0.0) - w0.get(k, 0.0))
              for k in ('roll', 'wrist_pitch', 'wrist_yaw')}
        print(f'[{label:>4}] tip {args.axis} {args.angle if label == "+" else 0.0:.0f} deg: tip turned '
              f'{turned:.2f} deg from start, moved {1000 * np.linalg.norm(p - p0):.2f} mm; '
              f'instrument roll {dw["roll"]:+.2f}, pitch {dw["wrist_pitch"]:+.2f}, '
              f'yaw {dw["wrist_yaw"]:+.2f} deg; UR joints up to '
              f'{math.degrees(np.abs(node.q - q0).max()):.2f} deg')
    print('DONE')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
