#!/usr/bin/env python3
"""E8 motion: move a REAL arm back and forth at a set speed until the operator presses ARM HOLD.

Run under run_e8.sh (which records the bag), the arm under External Control, the controller
active, a hand on a pendant e-stop and the other on the ARM HOLD box:

    python3 e8_hold_real_arm.py [--speeds 0.10,0.25] [--trials 5] [--stroke 0.12] [--axis x]
                                [--hold 20]

Per trial: the tool moves back and forth along one base axis over STROKE at constant SPEED
(reversals at 2 m/s^2), with the analytic velocity as feedforward. The operator presses the
blue ARM HOLD whenever they like. The moment the UR reports SAFEGUARD_STOP
(<arm>/status, safety mode 5) the script STOPS PUBLISHING, so nothing is waiting to
be followed on reset, and asks for blue out + green after HOLD s. Once the arm is back in
NORMAL with the program running it waits 3 s (the resume window the analyzer inspects), then
returns the tool to the start at 20 mm/s for the next trial. Aborts (stops publishing) the
moment the tool is more than STROKE/2 + 30 mm from the start or a joint more than 30 deg.

The numbers come from the bag, not from this script (e8_analyze.py).
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, ToolTipState

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'test'))
from arm_io import (JOINT_STATES, STATUS, TARGET_TOPIC, TOOL_TIP_STATE,  # noqa: E402
                    ToolTipTarget, target)

REVERSAL_ACC = 2.0     # m/s^2
RETURN_SPEED = 0.02    # m/s
ABORT_JOINT = math.radians(30.0)
PRESS_TIMEOUT = 60.0   # s of motion without a press before the trial is skipped


class Mover(Node):

    def __init__(self):
        super().__init__('e8_hold_real_arm')
        self.tip = None
        self.tip_time = 0.0
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.q = None
        self.safety = None
        self.running = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, JOINT_STATES,
                                 lambda m: setattr(self, 'q', np.array(m.position)), 10)
        self.create_subscription(ArmStatus, STATUS, self._on_status, latched)

    def _on_status(self, m):
        self.safety, self.running = m.safety_mode, m.program_running

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

    def ready(self):
        return self.safety == SafetyMode.NORMAL and self.running


def profile(t, stroke, speed):
    """Back and forth over [0, stroke] at constant speed, reversals at REVERSAL_ACC: (x, v)."""
    ta = speed / REVERSAL_ACC
    da = 0.5 * REVERSAL_ACC * ta ** 2
    tc = (stroke - 2 * da) / speed
    leg = 2 * ta + tc
    k, u = divmod(t, leg)
    if u < ta:
        x, v = 0.5 * REVERSAL_ACC * u ** 2, REVERSAL_ACC * u
    elif u < ta + tc:
        x, v = da + speed * (u - ta), speed
    else:
        w = leg - u
        x, v = stroke - 0.5 * REVERSAL_ACC * w ** 2, REVERSAL_ACC * w
    return (x, v) if int(k) % 2 == 0 else (stroke - x, -v)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--speeds', default='0.10,0.25', help='m/s, at most 0.25')
    ap.add_argument('--trials', type=int, default=5, help='presses per speed')
    ap.add_argument('--stroke', type=float, default=0.12, help='m, at most 0.15')
    ap.add_argument('--axis', default='x', choices=['x', 'y', 'z'])
    ap.add_argument('--hold', type=float, default=20.0, help='s to stay in HOLD')
    args = ap.parse_args()
    speeds = [float(s) for s in args.speeds.split(',')]
    if max(speeds) > 0.25 or args.stroke > 0.15:
        sys.exit('speeds at most 0.25 m/s (the controller cap) and stroke at most 0.15 m')
    for v in speeds:
        if args.stroke < v * v / REVERSAL_ACC + 0.02:
            sys.exit(f'stroke too short for {v} m/s')
    e = np.eye(3)['xyz'.index(args.axis)]
    abort_radius = args.stroke / 2 + 0.03

    rclpy.init()
    n = Mover()
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
    if n.q is None or n.safety is None or n.tip is None:
        sys.exit(f'no {JOINT_STATES}, {STATUS} or {TOOL_TIP_STATE}')
    if not n.ready():
        sys.exit(f'arm not ready: safety mode {n.safety}, program running {n.running}')
    home, quat = n.tool()
    j0 = n.q.copy()
    # The stroke is centred on the start pose.
    start = home - 0.5 * args.stroke * e

    def far():
        p, _ = n.tool()
        if np.linalg.norm(p - home) > abort_radius:
            return f'tool {1e3 * np.linalg.norm(p - home):.1f} mm from home'
        if np.abs(n.q - j0).max() > ABORT_JOINT:
            return f'a joint moved {math.degrees(np.abs(n.q - j0).max()):.1f} deg'
        return None

    def line(a, b, speed):
        """Straight move a -> b at speed; False if HOLD or an abort interrupts it."""
        d = np.linalg.norm(b - a)
        dur, t0 = max(d / speed, 0.5), time.monotonic()
        u = (b - a) / max(d, 1e-9)
        while (t := time.monotonic() - t0) < dur + 0.5:
            s = min(t / dur, 1.0)
            n.send(a + s * (b - a), quat, u * speed if s < 1.0 else np.zeros(3))
            if not n.ready() or far():
                return False
            time.sleep(0.002)
        return True

    print(f'home {np.round(home * 1e3, 1)} mm; stroke {args.stroke * 1e3:.0f} mm along {args.axis}')
    if not line(home, start, RETURN_SPEED):
        sys.exit(f'ABORT on the way to the stroke start: {far() or "not ready"}')
    for v in speeds:
        for k in range(args.trials):
            print(f'\n[{v * 1e3:.0f} mm/s, press {k + 1}/{args.trials}] moving -- press ARM HOLD '
                  'whenever you like')
            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                x, vx = profile(t, args.stroke, v)
                n.send(start + x * e, quat, vx * e)
                if n.safety == SafetyMode.SAFEGUARD_STOP:
                    print(f'  SAFEGUARD_STOP after {t:.1f} s: stopped publishing')
                    break
                why = far()
                if why or not n.ready():
                    print(f'ABORT: {why or f"safety mode {n.safety}, program {n.running}"}; '
                          'stopped publishing')
                    return 1
                if t > PRESS_TIMEOUT:
                    print('  no press; trial skipped')
                    break
                time.sleep(0.002)
            if n.safety == SafetyMode.SAFEGUARD_STOP:
                time.sleep(args.hold)
                print(f'  held {args.hold:.0f} s -- now: blue OUT, then tap GREEN')
                while not n.ready():
                    time.sleep(0.01)
                print('  resumed; 3 s resume window')
                time.sleep(3.0)
            here, _ = n.tool()
            if not line(here, start, RETURN_SPEED):
                print(f'ABORT on the way back: {far() or "not ready"}')
                return 1
    line(n.tool()[0], home, RETURN_SPEED)
    print('\nDONE')
    rclpy.shutdown()
    spinner.join()
    n.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())
