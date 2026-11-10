#!/usr/bin/env python3
"""Pivot the instrument about its RCM, through the same input teleop uses (RCM-drift benchmark).

The arm under External Control, instrument engaged and zeroed, the RCM ON (set_rcm.py on; no
cannula: the point is in free air), a hand on the pendant e-stop:

    python3 rcm_motion_real_arm.py wobble [--amplitude 15] [--period 6] [--seconds S]
    python3 rcm_motion_real_arm.py drift  [--minutes 5] [--seed 1]

Every target is the tip pose at the start RIGIDLY ROTATED about the RCM point c by R(t):
p(t) = c + R(t) (p0 - c), R_tip(t) = R(t) R0. The whole instrument turns about c, so the shaft
keeps passing through c whatever the wrist angle, and the wrist stays fixed relative to the
shaft. No insertion change. R(t) = pivot (about the two axes normal to the start shaft) composed
with roll (about the start shaft). Streamed at 500 Hz on <arm>/tool_tip_target with linear and
angular feedforward (numerical derivative of the same path).

  wobble   the marker-alignment motion: the shaft circles a cone of half-angle `amplitude` at
           one revolution per `period` s, until Ctrl+C or `seconds`; slide the marker along the
           shaft meanwhile and watch pivot_monitor.py in the tracker's shell.
  drift    the benchmark trial: a smooth random pivot (two axes, up to max_pivot_deg) plus roll
           (up to max_roll_deg), each axis a sum of 3 sinusoids with seeded random periods
           (7-23 s) and phases, scaled so no axis exceeds its bound and the angular rate stays
           under max_rate_deg_s; `minutes` long. The same seed gives the same motion.

Both ramp in over 3 s and back to the start over 3 s (also on Ctrl+C). Events on
/<arm>/bench_event (std_msgs/String, JSON): start (mode, c, p0, q0, parameters), done | abort.
Stops publishing (the controller then holds) and exits on: the RCM switched off or its QP
failing, the arm leaving NORMAL / the program stopping, the measured tip more than 10 mm from
the commanded one, or a UR joint more than 45 deg from its start.
"""
import argparse
import json
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
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, RcmState, ToolTipState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'test'))
from arm_io import (ARM, JOINT_STATES, STATUS, TARGET_TOPIC, TOOL_TIP_STATE,  # noqa: E402
                    ToolTipTarget, target)

RATE_HZ = 500.0
RAMP_S = 3.0
ABORT_TRACKING = 0.010
ABORT_JOINT = math.radians(45.0)
EXECUTOR = []


class Mover(Node):

    def __init__(self):
        super().__init__('rcm_motion_real_arm')
        self.tip = None
        self.tip_time = 0.0
        self.q = None
        self.safety = None
        self.running = None
        self.rcm = None
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(JointState, JOINT_STATES, self._on_joints, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ArmStatus, STATUS, self._on_status, latched)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.event_pub = self.create_publisher(String, f'/{ARM}/bench_event', 10)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))
        self.tip_time = time.monotonic()

    def _on_joints(self, m):
        self.q = np.array([p for n, p in zip(m.name, m.position) if '_tool_' not in n])

    def _on_rcm(self, m):
        self.rcm = m

    def _on_status(self, m):
        self.safety, self.running = m.safety_mode, m.program_running

    def tool(self):
        if self.tip is None or time.monotonic() - self.tip_time > 0.2:
            raise RuntimeError(f'no fresh {TOOL_TIP_STATE}')
        return self.tip

    def send(self, pos, quat, v, w):
        m = target(self.get_clock().now().to_msg(), pos, quat, v)
        m.velocity.angular.x, m.velocity.angular.y, m.velocity.angular.z = map(float, w)
        self.pub.publish(m)

    def event(self, **kw):
        self.event_pub.publish(String(data=json.dumps(kw)))

    def ready(self):
        return self.safety == SafetyMode.NORMAL and self.running


def ramp(t, total):
    """Smooth 0 -> 1 over RAMP_S, 1 -> 0 over the last RAMP_S of `total` (inf: no ramp-out)."""
    def s(x):
        x = min(max(x / RAMP_S, 0.0), 1.0)
        return x * x * (3 - 2 * x)
    return s(t) * s(total - t) if math.isfinite(total) else s(t)


def wobble_angles(amplitude_deg, period_s):
    a = math.radians(amplitude_deg)

    def f(t):
        ph = 2 * math.pi * t / period_s
        return np.array([a * math.sin(ph), a * math.cos(ph), 0.0])
    return f


def drift_angles(seed, max_pivot_deg, max_roll_deg, max_rate_deg_s):
    rng = np.random.default_rng(seed)
    per = rng.uniform(7.0, 23.0, (3, 3))
    pha = rng.uniform(0, 2 * math.pi, (3, 3))
    amp = rng.uniform(0.5, 1.0, (3, 3))
    bound = np.radians([max_pivot_deg, max_pivot_deg, max_roll_deg])
    # Scale each axis to its bound over a long sample, then all of them to the rate limit.
    ts = np.arange(0, 600, 0.02)
    raw = np.array([sum(amp[i, k] * np.sin(2 * math.pi * ts / per[i, k] + pha[i, k])
                        for k in range(3)) for i in range(3)])
    scale = bound / np.abs(raw).max(axis=1)
    rate = np.abs(np.diff(raw * scale[:, None], axis=1)).max() / 0.02
    if rate > math.radians(max_rate_deg_s):
        scale *= math.radians(max_rate_deg_s) / rate

    def f(t):
        return scale * np.array([sum(amp[i, k] * math.sin(2 * math.pi * t / per[i, k] + pha[i, k])
                                     for k in range(3)) for i in range(3)])
    return f, {'periods_s': per.tolist(), 'phases': pha.tolist(), 'amplitudes': amp.tolist(),
               'scale_rad': scale.tolist()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['wobble', 'drift'])
    ap.add_argument('--amplitude', type=float, default=15.0, help='wobble: cone half-angle, deg')
    ap.add_argument('--period', type=float, default=6.0, help='wobble: s per revolution')
    ap.add_argument('--seconds', type=float, default=math.inf, help='wobble: stop after S s')
    ap.add_argument('--minutes', type=float, default=5.0, help='drift: trial length')
    ap.add_argument('--seed', type=int, default=1, help='drift: the motion')
    ap.add_argument('--max-pivot', type=float, default=20.0, help='drift: deg about each axis')
    ap.add_argument('--max-roll', type=float, default=30.0, help='drift: deg about the shaft')
    ap.add_argument('--max-rate', type=float, default=15.0, help='drift: deg/s per axis')
    args = ap.parse_args()

    rclpy.init()
    n = Mover()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    EXECUTOR.append((ex, n))

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()
    time.sleep(2.0)
    if n.q is None or n.safety is None or n.tip is None or n.rcm is None:
        sys.exit(f'no {JOINT_STATES}, {STATUS}, {TOOL_TIP_STATE} or /{ARM}/rcm_state')
    if not n.ready():
        sys.exit(f'arm not ready: safety mode {n.safety}, program running {n.running}')
    if not (n.rcm.enabled and n.rcm.constraint_ok):
        sys.exit('the RCM is off: switch it on first (set_rcm.py on)')

    p0, q0 = n.tool()
    r0 = Rotation.from_quat(q0)
    c = np.array([n.rcm.point.x, n.rcm.point.y, n.rcm.point.z])
    axis = (p0 - c) / np.linalg.norm(p0 - c)               # start shaft, RCM -> tip
    e1 = np.cross(axis, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 0.3:
        e1 = np.cross(axis, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    j0 = n.q.copy()

    if args.mode == 'wobble':
        angles, total, extra = wobble_angles(args.amplitude, args.period), args.seconds, {}
    else:
        angles, extra = drift_angles(args.seed, args.max_pivot, args.max_roll, args.max_rate)
        total = 60.0 * args.minutes

    def rot(t, stop_at):
        """R(t) about c, with the ramp; stop_at = the time the ramp-out ends."""
        a = angles(t) * ramp(t, stop_at)
        return (Rotation.from_rotvec(a[0] * e1 + a[1] * e2) * Rotation.from_rotvec(a[2] * axis))

    def pose(t, stop_at):
        r = rot(t, stop_at)
        return c + r.apply(p0 - c), r * r0

    def fault(cmd):
        here, _ = n.tool()
        if np.linalg.norm(here - cmd) > ABORT_TRACKING:
            return f'tip {1e3 * np.linalg.norm(here - cmd):.1f} mm from the command'
        if np.abs(n.q - j0).max() > ABORT_JOINT:
            return f'a UR joint moved {math.degrees(np.abs(n.q - j0).max()):.1f} deg'
        if not (n.rcm.enabled and n.rcm.constraint_ok):
            return 'RCM switched off or its QP failed'
        if not n.ready():
            return f'safety mode {n.safety}, program running {n.running}'
        return None

    print(f'{ARM}: {args.mode}, RCM {np.round(1e3 * c, 1)} mm, insertion '
          f'{1e3 * np.linalg.norm(p0 - c):.1f} mm (tip), '
          + (f'cone {args.amplitude:.0f} deg, {args.period:.0f} s/rev, Ctrl+C to stop'
             if args.mode == 'wobble' else
             f'{args.minutes:.1f} min, seed {args.seed}, pivot <= {args.max_pivot:.0f} deg, '
             f'roll <= {args.max_roll:.0f} deg, <= {args.max_rate:.0f} deg/s'))
    n.event(event='start', mode=args.mode, arm=ARM, c=c.tolist(), p0=p0.tolist(), q0=q0.tolist(),
            args={k: (v if math.isfinite(v) else None) if isinstance(v, float) else v
                  for k, v in vars(args).items()}, **extra)
    dt = 1.0 / RATE_HZ
    h = 1e-3
    stop_at = total
    t0 = time.monotonic()
    result, last_print = 'done', 0.0
    try:
        while True:
            t = time.monotonic() - t0
            if t >= stop_at:
                break
            pa, ra = pose(t, stop_at)
            pb, rb = pose(t + h, stop_at)
            v = (pb - pa) / h
            w = (rb * ra.inv()).as_rotvec() / h
            n.send(pa, ra.as_quat(), v, w)
            if (why := fault(pa)):
                print(f'\nABORT: {why}; stopped publishing')
                n.event(event='abort', why=why)
                return 1
            if t - last_print > 10.0:
                print(f'  {t:6.0f} s  RCM deviation {1e3 * n.rcm.deviation:.3f} mm', flush=True)
                last_print = t
            time.sleep(dt)
    except KeyboardInterrupt:
        # Ramp out from here: re-time the path so the angles fade to zero over RAMP_S.
        t_int = time.monotonic() - t0
        a_int = angles(t_int) * ramp(t_int, stop_at)
        print('\n  stopping: back to the start over 3 s')
        t1 = time.monotonic()
        while (s := time.monotonic() - t1) < RAMP_S:
            k = 1.0 - ramp(s, math.inf)
            r = (Rotation.from_rotvec(k * (a_int[0] * e1 + a_int[1] * e2))
                 * Rotation.from_rotvec(k * a_int[2] * axis))
            n.send(c + r.apply(p0 - c), (r * r0).as_quat(), np.zeros(3), np.zeros(3))
            time.sleep(dt)
        result = 'stopped'
    n.send(p0, q0, np.zeros(3), np.zeros(3))
    n.event(event='done', result=result)
    print(f'\nDONE ({result})')
    return 0


if __name__ == '__main__':
    try:
        code = main()
    except SystemExit as e:
        code = e.code
    finally:
        for ex, node in EXECUTOR:
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if isinstance(code, str):
        print(code, file=sys.stderr)
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code or 0)
