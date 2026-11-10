#!/usr/bin/env python3
"""The camera arm (ECM) under an RCM, on mock hardware: the scope-tip preset and the roll-only
orientation task it carries (tool_tips.yaml stereo_laparoscope).

    ARTS_ARM=ecm ARTS_TIP=stereo_laparoscope run_servo_smoke_test.sh $PWD/ecm_rcm_mock_test.py

Commands are tool_tip_target messages of the kind camera_teleop.py sends: a tip position plus
the orientation LATCHED at the start (turned only by the steering roll), so under the RCM the
scope direction must come from the constraint, not from that orientation. Checks:
  1. set_rcm on at 100 mm: accepted, insertion 100 mm.
  2. Tip 10 mm sideways (normal to the scope), the start orientation: the tip arrives
     (< 0.5 mm), the scope PIVOTS by atan(10/100) = 5.71 deg (+-0.3), insertion = the goal's
     distance from the point, sqrt(100^2 + 10^2) = 100.50 mm (+-0.1), the RCM deviation stays
     < 0.05 mm throughout.
  3. Tip 10 mm further in along the (pivoted) scope: insertion 110.50 mm (+-0.1), direction
     unchanged (< 0.3 deg).
  4. Steering roll 20 deg about the scope: the scope turns 20 deg (+-1) about its own axis,
     the tip stays (< 0.5 mm), direction unchanged (< 0.3 deg).
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

from arts_interfaces.msg import RcmState, ToolTipState
from arts_interfaces.srv import SetRcm

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from arm_io import ARM, TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget, target  # noqa: E402

FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


class Rig(Node):

    def __init__(self):
        super().__init__('ecm_rcm_mock_test')
        self.tip = None
        self.rcm = None
        self.max_dev = 0.0
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.set_rcm = self.create_client(SetRcm, f'/{ARM}/set_rcm')

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))

    def _on_rcm(self, m):
        self.rcm = m
        if m.enabled and np.isfinite(m.deviation):
            self.max_dev = max(self.max_dev, m.deviation)

    def hold(self, pos, rot, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(target(self.get_clock().now().to_msg(), pos, rot.as_quat()))
            time.sleep(0.008)

    def axis(self):
        return self.tip[1].apply([0.0, 0.0, 1.0])


def angle(a, b):
    return math.degrees(math.acos(np.clip(np.dot(a, b), -1.0, 1.0)))


def main():
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    rig.set_rcm.wait_for_service(timeout_sec=10.0)
    time.sleep(2.0)
    try:
        print('[1] RCM on, 100 mm')
        fut = rig.set_rcm.call_async(SetRcm.Request(enable=True, from_current_shaft=True,
                                                    insertion_depth=0.10))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        r = fut.result()
        check(r is not None and r.success, f'RCM on ({r.message if r else "no response"})')
        time.sleep(0.5)
        p0, r0 = rig.tip
        a0 = rig.axis()
        ins0 = rig.rcm.insertion_depth
        check(abs(ins0 - 0.10) < 5e-4, f'insertion {1e3 * ins0:.1f} mm')
        rig.max_dev = 0.0

        print('[2] tip 10 mm sideways, start orientation held in the command')
        side = np.cross(a0, [0.0, 0.0, 1.0])
        if np.linalg.norm(side) < 0.3:
            side = np.cross(a0, [1.0, 0.0, 0.0])
        side /= np.linalg.norm(side)
        goal = p0 + 0.010 * side
        rig.hold(goal, r0, 4.0)
        p1, _ = rig.tip
        a1 = rig.axis()
        piv = angle(a0, a1)
        check(np.linalg.norm(p1 - goal) < 5e-4, f'tip at the goal: {1e3 * np.linalg.norm(p1 - goal):.3f} mm')
        check(abs(piv - math.degrees(math.atan(0.1))) < 0.3,
              f'scope pivoted {piv:.2f} deg (expected {math.degrees(math.atan(0.1)):.2f})')
        # The goal is off the sphere: |goal - c| = sqrt(100^2 + 10^2) = 100.50 mm.
        ins1 = math.hypot(ins0, 0.010)
        check(abs(rig.rcm.insertion_depth - ins1) < 1e-4,
              f'insertion {1e3 * rig.rcm.insertion_depth:.2f} mm (expected {1e3 * ins1:.2f})')

        print('[3] 10 mm further in along the scope')
        goal = p1 + 0.010 * a1
        rig.hold(goal, r0, 4.0)
        p2, _ = rig.tip
        check(abs(rig.rcm.insertion_depth - (ins1 + 0.010)) < 1e-4,
              f'insertion {1e3 * rig.rcm.insertion_depth:.2f} mm (expected {1e3 * (ins1 + 0.01):.2f})')
        check(angle(a1, rig.axis()) < 0.3, f'direction kept: {angle(a1, rig.axis()):.3f} deg')

        print('[4] steering roll 20 deg about the scope')
        a2 = rig.axis()
        _, r2 = rig.tip
        roll_goal = Rotation.from_rotvec(math.radians(20.0) * a2) * r0
        rig.hold(goal, roll_goal, 5.0)
        p3, r3 = rig.tip
        # The roll actually turned: the twist of r3 r2^-1 about the scope axis.
        q = (r3 * r2.inv()).as_quat()
        q = -q if q[3] < 0 else q
        twist = math.degrees(2.0 * math.atan2(np.dot(q[:3], a2), q[3]))
        check(abs(twist - 20.0) < 1.0, f'scope rolled {twist:.2f} deg about its axis (20)')
        check(np.linalg.norm(p3 - goal) < 5e-4, f'tip stayed: {1e3 * np.linalg.norm(p3 - goal):.3f} mm')
        check(angle(a2, rig.axis()) < 0.3, f'direction kept: {angle(a2, rig.axis()):.3f} deg')
        check(rig.max_dev < 5e-5, f'RCM deviation max {1e3 * rig.max_dev:.4f} mm over [2]-[4]')
    finally:
        fut = rig.set_rcm.call_async(SetRcm.Request(enable=False))
        time.sleep(1.0)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    return 1 if FAILS else 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)    # the daemon spin thread can abort the interpreter's teardown
