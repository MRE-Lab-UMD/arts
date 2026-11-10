#!/usr/bin/env python3
"""Shake sweep for rate-mode tracking: the arm may LAG a commanded shake, never AMPLIFY it.

Run through the mock-arm wrapper (the default config is rate mode):

    arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh rate_sweep_test.py

Commands a sinusoid along base x, with its analytic velocity in tool_tip_target as the
feedforward, at 2 to 10 Hz and 15 / 30 mm peak to peak, 4 s each, and measures the tool's
peak-to-peak along x from tf over the last 2 s and the lag that best aligns it with the
command. Fails if any arm amplitude exceeds the command by more than 5 % (without the
feedforward fade, a 10 Hz shake swung the mock arm to 1.7x, 2026-10-30), or if the slowest
shake (2 Hz, 15 mm) is not followed to within 5 %.
"""
import sys
import time

import numpy as np
import rclpy
from arm_io import BASE_FRAME, TARGET_TOPIC, TOOL_FRAME, ToolTipTarget, target
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

CASES = [(f, pp) for pp in (0.015, 0.030) for f in (2, 4, 6, 8, 10)]
MAX_RATIO = 1.05
SLOW_CASE, MIN_SLOW_RATIO = (2, 0.015), 0.95


class Sweep(Node):

    def __init__(self):
        super().__init__('rate_sweep_test')
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.002)

    def tool(self):
        t = self.tf.lookup_transform(BASE_FRAME, TOOL_FRAME, Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z]), t.rotation

    def send(self, p, q, vx):
        self.pub.publish(target(self.get_clock().now().to_msg(), p, q, (vx, 0.0, 0.0)))

    def shake(self, f, pp, p0, q0, seconds=4.0):
        log, began, nxt = [], time.monotonic(), time.monotonic()
        w = 2 * np.pi * f
        while time.monotonic() - began < seconds:
            t = time.monotonic() - began
            x, vx = 0.5 * pp * np.sin(w * t), 0.5 * pp * w * np.cos(w * t)
            self.send(p0 + [x, 0, 0], q0, vx)
            rclpy.spin_once(self, timeout_sec=0.0)
            log.append((t, x, self.tool()[0][0] - p0[0]))
            nxt += 0.002
            time.sleep(max(0.0, nxt - time.monotonic()))
        # return to rest at the start pose
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            self.send(p0, q0, 0.0)
            self.spin_for(0.002)
        a = np.array(log)
        a = a[a[:, 0] > seconds - 2.0]
        g = np.arange(a[0, 0], a[-1, 0], 0.001)
        c, r = np.interp(g, a[:, 0], a[:, 1]), np.interp(g, a[:, 0], a[:, 2])
        lag = int(np.argmax([np.dot(c[:len(c) - k], r[k:]) for k in range(250)]))
        return np.ptp(r) / np.ptp(c), lag


def main():
    rclpy.init()
    n = Sweep()
    n.spin_for(3.0)
    p0, q0 = n.tool()
    fails = []
    print(' freq  p-p     ratio  lag')
    for f, pp in CASES:
        ratio, lag = n.shake(f, pp, p0, q0)
        flag = ''
        if ratio > MAX_RATIO:
            fails.append(f'{f} Hz {pp * 1e3:.0f} mm: arm {ratio:.2f}x the command (overshoot)')
            flag = '  <- OVERSHOOT'
        if (f, pp) == SLOW_CASE and ratio < MIN_SLOW_RATIO:
            fails.append(f'{f} Hz {pp * 1e3:.0f} mm: followed only {ratio:.2f}')
            flag = '  <- NOT FOLLOWED'
        print(f' {f:3d}Hz {pp * 1e3:3.0f}mm  {ratio:5.2f}  {lag:3d} ms{flag}')
    for m in fails:
        print('FAIL:', m)
    print('PASS' if not fails else f'{len(fails)} failure(s)')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
