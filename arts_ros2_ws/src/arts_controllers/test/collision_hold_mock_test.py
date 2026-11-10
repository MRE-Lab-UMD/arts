#!/usr/bin/env python3
"""The servo controller obeys /arts/collision_hold (arts_safety collision_guard), on mock.

    ARTS_ARM=psm_left run_servo_smoke_test.sh $PWD/collision_hold_mock_test.py

  1. hold = true (latched), a target 20 mm away streamed for 2 s: the tip does not move
     (< 0.2 mm).
  2. hold = false, the same target: the tip arrives (< 0.5 mm).
  3. hold = true 0.3 s into a move to a target 100 mm away: the tip stops clearly short (> 10 mm left).
"""
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from arts_interfaces.msg import ToolTipState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_io import TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget, target  # noqa: E402

FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


class Rig(Node):

    def __init__(self):
        super().__init__('collision_hold_mock_test')
        self.tip = None
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.hold = self.create_publisher(Bool, '/arts/collision_hold', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w))

    def stream(self, pos, quat, seconds, at=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if at is not None and time.monotonic() >= at[0]:
                at[1]()
                at = None
            self.pub.publish(target(self.get_clock().now().to_msg(), pos, quat))
            time.sleep(0.005)


def main():
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    time.sleep(3.0)
    try:
        p0, q0 = rig.tip
        goal = p0 + np.array([0.0, 0.0, 0.02])
        print('[1] hold on, target 20 mm away')
        rig.hold.publish(Bool(data=True))
        time.sleep(0.3)
        rig.stream(goal, q0, 2.0)
        d = np.linalg.norm(rig.tip[0] - p0) * 1e3
        check(d < 0.2, f'no motion while held: {d:.3f} mm')
        print('[2] hold off, same target')
        rig.hold.publish(Bool(data=False))
        time.sleep(0.3)
        rig.stream(goal, q0, 3.0)
        d = np.linalg.norm(rig.tip[0] - goal) * 1e3
        check(d < 0.5, f'arrives once released: {d:.3f} mm from the target')
        print('[3] hold on 0.3 s into a 100 mm move')
        p1 = rig.tip[0]
        goal2 = p1 + np.array([0.0, 0.10, 0.0])
        rig.stream(goal2, q0, 3.0, at=(time.monotonic() + 0.3,
                                         lambda: rig.hold.publish(Bool(data=True))))
        left = np.linalg.norm(rig.tip[0] - goal2) * 1e3
        moved = np.linalg.norm(rig.tip[0] - p1) * 1e3
        check(left > 10.0, f'stopped short of the target: moved {moved:.1f} of 100 mm (the mock arm runs up to 250 mm/s)')
    finally:
        rig.hold.publish(Bool(data=False))
        time.sleep(0.3)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    return 1 if FAILS else 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)
