#!/usr/bin/env python3
"""First motion of a REAL arm under the Cartesian servo controller.

Not servo_smoke_test.py: that one drives toward a target 1.5 m away to probe the workspace
boundary, which is fine on mock hardware and wrong on a real arm. This script only:

  1. holds: 10 s with no command, the tool must not move more than 0.5 mm;
  2. moves the tool STEP along +x, back, +y, back, +z, back (base_link axes), orientation
     fixed, the target interpolated at SPEED so the controller tracks a slow path;
  3. aborts -- stops publishing, so the controller holds after its 0.1 s reference timeout --
     the moment the tool is more than ABORT_RADIUS from its start or any joint more than
     ABORT_JOINT from its start.

The tool pose is the controller's own tip (<arm>/tool_tip_state), the frame its targets are
in: with a tool-tip offset (the da Vinci X preset puts the tip 0.6 m from the flange), a
target built from tf tool0 would be read as a tip target 0.6 m away. The first version did
exactly that; caught before it ran with the preset (2026-11-01).
Run it with the arm under External Control and the controller active, a hand on the e-stop.
Prerequisite: branch_singularity_test.py passing on mock (decision 17).

    python3 first_motion_real_arm.py [--step 0.03] [--speed 0.02] [--axes xyz] [--hold 10]

--axes x runs one axis only, e.g. so an operator can name the direction each base axis
moves in the room (the tool-arm bases are mounted at an angle).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from arm_io import BASE_FRAME, JOINT_STATES, TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget
from arts_interfaces.msg import ToolTipState
from rclpy.node import Node
from sensor_msgs.msg import JointState

ABORT_RADIUS = 0.04          # m
ABORT_JOINT = math.radians(15.0)
PUBLISH_PERIOD = 0.02        # s


class FirstMotion(Node):
    def __init__(self):
        super().__init__("first_motion_real_arm")
        self.tip = None
        self.tip_time = 0.0
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.pub = self.create_publisher(
            ToolTipTarget, TARGET_TOPIC, 10)
        self.q = None
        self.create_subscription(JointState, JOINT_STATES, self._on_js, 10)

    def _on_tip(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w))
        self.tip_time = time.time()

    def _on_js(self, msg):
        self.q = np.array(msg.position)

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
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--step", type=float, default=0.03, help="m, at most 0.035")
    ap.add_argument("--speed", type=float, default=0.02, help="m/s")
    ap.add_argument("--axes", default="xyz", help="base_link axes to move along, in order")
    ap.add_argument("--hold", type=float, default=10.0, help="s of no command first")
    args = ap.parse_args()
    if not args.axes or set(args.axes) - set("xyz"):
        sys.exit("--axes takes letters from xyz")
    if args.step > ABORT_RADIUS - 0.005:
        sys.exit(f"--step must stay {1000 * 0.005:.0f} mm inside the {1000 * ABORT_RADIUS:.0f} mm abort radius")

    rclpy.init()
    node = FirstMotion()
    node.spin_for(3.0)
    if node.q is None or node.tip is None:
        sys.exit(f"no /joint_states or {TOOL_TIP_STATE}")
    p0, quat = node.tool()
    q0 = node.q.copy()
    print(f"start tool tip ({p0[0]:+.4f}, {p0[1]:+.4f}, {p0[2]:+.4f}) m; joints (deg, /joint_states order)",
          np.round(np.degrees(q0), 1))

    def guard():
        p, _ = node.tool()
        if np.linalg.norm(p - p0) > ABORT_RADIUS:
            return f"tool {1000 * np.linalg.norm(p - p0):.1f} mm from start"
        if np.abs(node.q - q0).max() > ABORT_JOINT:
            return f"a joint moved {math.degrees(np.abs(node.q - q0).max()):.1f} deg"
        return None

    node.spin_for(args.hold)
    drift = np.linalg.norm(node.tool()[0] - p0)
    print(f"[hold] {args.hold:.0f} s, no command: tool moved {1000 * drift:.3f} mm")
    if drift > 5e-4:
        print("ABORT: the arm moved without a command")
        return 1

    results = []
    for name in args.axes:
        axis = "xyz".index(name)
        for direction in (+1, -1):
            start = p0 if direction > 0 else p0 + args.step * np.eye(3)[axis]
            goal = p0 + args.step * np.eye(3)[axis] if direction > 0 else p0
            duration = args.step / args.speed
            began = time.time()
            while time.time() - began < duration + 2.0:
                s = min(1.0, (time.time() - began) / duration)
                node.send(start + s * (goal - start), quat)
                node.spin_for(PUBLISH_PERIOD)
                why = guard()
                if why:
                    print(f"ABORT: {why}; stopped publishing, the controller holds")
                    return 1
            err = np.linalg.norm(node.tool()[0] - goal)
            label = f"{'+' if direction > 0 else 'back'}{name if direction > 0 else ''}"
            print(f"[{label:>4}] {1000 * args.step:.0f} mm at {1000 * args.speed:.0f} mm/s: "
                  f"residual {1000 * err:.2f} mm")
            results.append(err)

    node.spin_for(1.0)
    p, _ = node.tool()
    print(f"back at start: {1000 * np.linalg.norm(p - p0):.2f} mm; "
          f"joints differ by up to {math.degrees(np.abs(node.q - q0).max()):.3f} deg")
    print("DONE" if max(results) < 1e-3 else "DONE, but a residual exceeded 1 mm")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
