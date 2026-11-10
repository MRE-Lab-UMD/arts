#!/usr/bin/env python3
"""Shake response of a REAL arm: same scripted motion for a servoj / speedj A/B.

Run with the arm under External Control, the controller active in rate mode, a hand on the
e-stop. Launch the arm once with command_interface:=position (servoj) and once with
command_interface:=velocity (speedj), and run this script unchanged against each:

    python3 shake_ab_real_arm.py [--pp 0.010] [--freqs 1,2,4] [--axis x] [--out FILE.json]

  1. holds 5 s with no command; the tool must not move more than 0.5 mm;
  2. shakes the tool along one base axis, PP peak to peak, 4 s at each frequency, with the
     analytic velocity as feedforward on ~/target_velocity, 1.5 s at rest between;
  3. reports, per frequency over the last 2 s: the arm's peak to peak (the controller's tip,
     <arm>/tool_tip_state; the arm is ARTS_ARM, default psm_left) as a fraction of the command, and the lag from the command's send time to the
     MEASURED joints: the <arm>/joint_states stamp at which the most-moving joint best correlates
     with the command;
  4. aborts -- stops publishing, the controller then holds -- the moment the tool is more
     than ABORT_RADIUS from its start or any joint more than ABORT_JOINT from its start.
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from arm_io import JOINT_STATES, TARGET_TOPIC, TOOL_TIP_STATE, ToolTipTarget, target
from arts_interfaces.msg import ToolTipState
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

ABORT_RADIUS = 0.030  # m
ABORT_JOINT = math.radians(15.0)


class Shaker(Node):

    def __init__(self):
        super().__init__('shake_ab_real_arm')
        self.tip = None
        self.tip_time = 0.0
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.q = None
        self.joints = []  # (stamp, positions) of every /joint_states, for the lag
        self.create_subscription(JointState, JOINT_STATES, self._on_js, 10)

    def _on_js(self, m):
        self.q = np.array(m.position)
        self.joints.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, *m.position))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tip(self, m):
        self.tip = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                    m.pose.orientation)
        self.tip_time = time.monotonic()

    def tool(self):
        # The controller's own tip, the frame its targets are in. tf tool0 would be wrong by
        # the whole tool-tip offset (0.6 m with the da Vinci X preset).
        if self.tip is None or time.monotonic() - self.tip_time > 0.2:
            raise RuntimeError(f'no fresh {TOOL_TIP_STATE}')
        return self.tip

    def send(self, p, q, v):
        self.pub.publish(target(self.get_clock().now().to_msg(), p, q, v))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--pp', type=float, default=0.010, help='m peak to peak, at most 0.02')
    ap.add_argument('--freqs', default='1,2,4', help='Hz, comma separated, at most 5')
    ap.add_argument('--axis', default='x', choices=['x', 'y', 'z'])
    ap.add_argument('--out', help='write the results as JSON')
    args = ap.parse_args()
    freqs = [float(f) for f in args.freqs.split(',')]
    if args.pp > 0.02 or max(freqs) > 5:
        sys.exit('--pp at most 0.02 m and frequencies at most 5 Hz on a real arm')
    e = np.eye(3)['xyz'.index(args.axis)]

    rclpy.init()
    n = Shaker()
    # Callbacks run in their own thread. Spinning once per 2 ms send cycle instead let /tf
    # and /joint_states queue up ~270 ms behind on the real arm (2026-10-30), which made the
    # lag meaningless and the abort guard late.
    ex = SingleThreadedExecutor()
    ex.add_node(n)

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass
    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    time.sleep(3.0)
    if n.q is None or n.tip is None:
        sys.exit(f'no {JOINT_STATES} or {TOOL_TIP_STATE}')
    p0, q0 = n.tool()
    j0 = n.q.copy()

    def guard():
        p, _ = n.tool()
        if np.linalg.norm(p - p0) > ABORT_RADIUS:
            return f'tool {1e3 * np.linalg.norm(p - p0):.1f} mm from start'
        if np.abs(n.q - j0).max() > ABORT_JOINT:
            return f'a joint moved {math.degrees(np.abs(n.q - j0).max()):.1f} deg'
        return None

    time.sleep(5.0)
    drift = np.linalg.norm(n.tool()[0] - p0)
    print(f'[hold] 5 s, no command: tool moved {1e3 * drift:.3f} mm')
    if drift > 5e-4:
        print('ABORT: the arm moved without a command')
        return 1

    results = []
    for f in freqs:
        w = 2 * np.pi * f
        log, began, nxt = [], time.monotonic(), time.monotonic()
        while time.monotonic() - began < 4.0:
            t = time.monotonic() - began
            x, v = 0.5 * args.pp * np.sin(w * t), 0.5 * args.pp * w * np.cos(w * t)
            n.send(p0 + x * e, q0, v * e)
            log.append((t, x, float((n.tool()[0] - p0) @ e), n.now()))
            why = guard()
            if why:
                print(f'ABORT: {why}; stopped publishing, the controller holds')
                return 1
            nxt += 0.002
            time.sleep(max(0.0, nxt - time.monotonic()))
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            n.send(p0, q0, np.zeros(3))
            time.sleep(0.002)
        a = np.array(log)
        a = a[a[:, 0] > 2.0]
        ratio = float(np.ptp(a[:, 2]) / np.ptp(a[:, 1]))
        js = np.array(n.joints)
        g = np.arange(a[0, 3], a[-1, 3], 0.001)
        c = np.interp(g, a[:, 3], a[:, 1])
        win = js[(js[:, 0] > g[0]) & (js[:, 0] < g[-1] + 0.3)]
        k = 1 + int(np.argmax(np.ptp(win[:, 1:], axis=0)))
        lag = int(np.argmax([abs(np.corrcoef(c, np.interp(g + L * 1e-3, js[:, 0], js[:, k]))[0, 1])
                             for L in range(300)]))
        results.append(dict(freq_hz=f, pp_m=args.pp, ratio=ratio, lag_ms=lag))
        print(f'[{f:3.1f} Hz] {1e3 * args.pp:.0f} mm p-p: arm {ratio:.2f} of the command, lag {lag} ms')

    time.sleep(1.0)
    print(f'back at start: {1e3 * np.linalg.norm(n.tool()[0] - p0):.2f} mm')
    if args.out:
        with open(args.out, 'w') as fh:
            json.dump(results, fh, indent=1)
    print('DONE')
    rclpy.shutdown()
    spinner.join()
    n.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())
