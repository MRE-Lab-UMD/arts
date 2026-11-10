#!/usr/bin/env python3
"""RCM switch and constraint test on mock hardware (decision 17 task layer).

    ARTS_ARM=psm_left run_servo_smoke_test.sh rcm_mock_test.py [--hz 0.5] [--amp 0.010]

A 300 mm instrument (tool-tip offset along the flange z) is servoed with the remote centre of
motion 100 mm behind the tip. Checks, each with its failure path where one exists:

  1. set_rcm refuses a point 10 mm off the shaft, and a tip not inserted far enough.
  2. set_rcm(from_current_shaft) engages with ~0 deviation, and the arm stays put while no
     new reference has arrived.
  3. set_tool_tip_offset is refused while the RCM is on.
  4. Tip motion with velocity feedforward, a Lissajous in the plane normal to the shaft plus
     insertion: the tip follows, the shaft pivots by atan(lateral / depth), and the kinematic
     RCM deviation (rcm_state, the open laparoscopic platform's definition) stays small;
     then the same at twice the rate and 1.5x the size, beyond the joint limits, where the
     tip lags but the RCM must hold.
  5. A target that would pull the tip back through the cannula stops at the minimum depth.
  6. With the RCM switched off, the same lateral motion moves the shaft off the point: the
     check in 4 can fail, so its passing means something.
  7. set_rcm with no depth (`set_rcm.py on`) after that motion puts the point on the arm's
     NEW shaft at the last engaged depth.

Prints the deviation statistics (median, p95, max) for each motion.
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from arm_io import ARM, BASE_FRAME, TARGET_TOPIC, TOOL_TIP_STATE, target
from arts_interfaces.msg import RcmState, ToolTipState
from arts_interfaces.srv import SetRcm, SetToolTipOffset

INSTRUMENT = 0.30   # m, tool-tip offset along the flange z
DEPTH = 0.10        # m, RCM point behind the tip
MIN_DEPTH = 0.005    # m, the controller's default rcm.insertion_limits[0]


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Rig(Node):

    def __init__(self):
        super().__init__('rcm_mock_test')
        self.pub = self.create_publisher(type(target(None, (0, 0, 0), (0, 0, 0, 1))), TARGET_TOPIC, 10)
        self.tip = None
        self.rcm = None
        self.dev_log = None
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        self.set_rcm = self.create_client(SetRcm, f'/{ARM}/set_rcm')
        self.set_offset = self.create_client(SetToolTipOffset, f'/{ARM}/set_tool_tip_offset')

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w))

    def _on_rcm(self, m):
        self.rcm = m
        if self.dev_log is not None:
            self.dev_log.append((m.deviation, m.insertion_depth, m.constraint_ok))

    def call(self, client, req, timeout=3.0):
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f'{client.srv_name}: no response')
        return fut.result()

    def rcm_on(self, from_current=True, depth=DEPTH, point=None):
        r = SetRcm.Request()
        r.enable = True
        r.from_current_shaft = from_current
        r.insertion_depth = depth
        if point is not None:
            r.point.header.frame_id = BASE_FRAME
            r.point.point.x, r.point.point.y, r.point.point.z = map(float, point)
        return self.call(self.set_rcm, r)

    def rcm_off(self):
        return self.call(self.set_rcm, SetRcm.Request(enable=False))

    def send(self, p, q, v=None):
        self.pub.publish(target(self.get_clock().now().to_msg(), p, q, v))


def stats(name, log):
    d = np.array([x[0] for x in log]) * 1e3
    d = d[np.isfinite(d)]
    print(f'  {name}: RCM deviation median {np.median(d):.4f} mm, p95 '
          f'{np.percentile(d, 95):.4f} mm, max {d.max():.4f} mm (n={len(d)})')
    return np.median(d), d.max()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hz', type=float, default=0.5, help='Lissajous base frequency')
    ap.add_argument('--amp', type=float, default=0.010, help='m, lateral amplitude')
    ap.add_argument('--seconds', type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    n = Rig()
    ex = SingleThreadedExecutor()
    ex.add_node(n)

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()
    failures = []

    def check(ok, what):
        print(('PASS ' if ok else 'FAIL ') + what)
        if not ok:
            failures.append(what)

    for c in (n.set_rcm, n.set_offset):
        if not c.wait_for_service(timeout_sec=10.0):
            sys.exit(f'{c.srv_name} not available')
    req = SetToolTipOffset.Request()
    req.offset = Pose()
    req.offset.position.z = INSTRUMENT
    req.offset.orientation.w = 1.0
    check(n.call(n.set_offset, req).success, 'tool-tip offset set to a 300 mm instrument')
    time.sleep(1.0)
    p0, q0 = n.tip
    axis0 = quat_to_matrix(q0) @ np.array([0, 0, 1])
    rcm_point = p0 - DEPTH * axis0

    # 1. refusals
    r = n.rcm_on(from_current=False, point=rcm_point + 0.010 * np.cross(axis0, [1, 0, 0]) /
                 np.linalg.norm(np.cross(axis0, [1, 0, 0])))
    check(not r.success and 'refused' in r.message, f'point 10 mm off the shaft refused ({r.message})')
    r = n.rcm_on(depth=0.001)
    check(not r.success and 'insertion' in r.message, f'1 mm insertion refused ({r.message})')

    # 2. engage + hold
    r = n.rcm_on()
    check(r.success and r.deviation < 1e-6, f'engaged from the current shaft ({r.message})')
    time.sleep(0.5)
    p_after, _ = n.tip
    check(np.linalg.norm(p_after - p0) < 1e-4,
          f'held after the switch: tip moved {1e3 * np.linalg.norm(p_after - p0):.3f} mm')
    p0, q0 = n.tip
    check(n.rcm.enabled and n.rcm.constraint_ok, 'rcm_state reports enabled and ok')

    # 3. offset frozen
    check(not n.call(n.set_offset, req).success, 'tool-tip offset refused while the RCM is on')

    # Lateral directions normal to the start shaft.
    e1 = np.cross(axis0, [0, 0, 1.0])
    if np.linalg.norm(e1) < 0.1:
        e1 = np.cross(axis0, [1.0, 0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis0, e1)

    def run_motion(seconds, lateral_amp, insert_amp, hz):
        """Lissajous 1:2 laterally plus insertion at 0.5x, with analytic feedforward."""
        w = 2 * math.pi * hz
        n.dev_log = []
        errs = []
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < seconds:
            ramp = min(t / 1.0, 1.0)  # 1 s fade-in so the start is not a step
            a, b, c = lateral_amp * ramp, lateral_amp * ramp, insert_amp * ramp
            p = p0 + a * math.sin(w * t) * e1 + b * math.sin(2 * w * t) * e2 + \
                c * math.sin(0.5 * w * t) * axis0
            v = a * w * math.cos(w * t) * e1 + 2 * b * w * math.cos(2 * w * t) * e2 + \
                0.5 * c * w * math.cos(0.5 * w * t) * axis0
            n.send(p, q0, v)
            if n.tip is not None and t > 1.5:
                errs.append(np.linalg.norm(n.tip[0] - p))
            time.sleep(0.002)
        log, n.dev_log = n.dev_log, None
        return log, np.array(errs)

    # 4. pivoting under the RCM: a realistic motion that must track, then a stress motion
    #    beyond the joint limits (pivot rate = lateral tip speed / depth: 2 Hz x 15 mm at
    #    100 mm depth needs ~1.9 rad/s against the 1 rad/s joint limit), where the tip may
    #    lag but the RCM must still hold.
    axis_start = quat_to_matrix(n.tip[1]) @ np.array([0, 0, 1])
    log, errs = run_motion(args.seconds, args.amp, 0.010, args.hz)
    med, mx = stats(f'RCM on, {args.hz:g} Hz x {1e3 * args.amp:.0f} mm + 10 mm insertion', log)
    rms = 1e3 * np.sqrt(np.mean(errs ** 2))
    print(f'  tip tracking error: RMS {rms:.2f} mm, max {1e3 * errs.max():.2f} mm')
    check(all(ok for _, _, ok in log), 'constraint_ok throughout')
    check(med < 0.1, f'median kinematic RCM deviation {med:.4f} mm < 0.1 mm')
    check(mx < 0.5, f'max kinematic RCM deviation {mx:.4f} mm < 0.5 mm')
    check(rms < 3.0, f'tip follows: tracking RMS {rms:.2f} mm < 3 mm')

    tilt_pos = p0 + args.amp * e1
    for _ in range(600):
        n.send(tilt_pos, q0)
        time.sleep(0.002)
    tilt = math.degrees(math.acos(np.clip(
        np.dot(quat_to_matrix(n.tip[1]) @ np.array([0, 0, 1]), axis_start), -1, 1)))
    expected = math.degrees(math.atan(args.amp / DEPTH))
    print(f'  shaft tilt at {1e3 * args.amp:.0f} mm lateral: {tilt:.2f} deg (atan: {expected:.2f})')
    check(abs(tilt - expected) < 0.5, 'the shaft pivoted about the point')

    log, errs = run_motion(args.seconds, 1.5 * args.amp, 0.010, 2 * args.hz)
    med, mx = stats(f'stress, {2 * args.hz:g} Hz x {1.5e3 * args.amp:.0f} mm (beyond joint limits)', log)
    print(f'  tip tracking error: RMS {1e3 * np.sqrt(np.mean(errs ** 2)):.2f} mm (lag expected)')
    check(mx < 0.5, f'stress: max kinematic RCM deviation {mx:.4f} mm < 0.5 mm')

    # 5. insertion floor: ask for the tip 20 mm BEHIND the RCM point
    n.dev_log = []
    for _ in range(1500):
        n.send(rcm_point - 0.020 * axis0, q0)
        time.sleep(0.002)
    depth_min = min(x[1] for x in n.dev_log)
    n.dev_log = None
    check(depth_min > MIN_DEPTH - 0.0005,
          f'insertion floor held: minimum depth {1e3 * depth_min:.2f} mm (limit {1e3 * MIN_DEPTH:.1f} mm)')
    for _ in range(1500):
        n.send(p0, q0)
        time.sleep(0.002)

    # 6. switch off; the same lateral motion now leaves the point
    r = n.rcm_off()
    check(r.success, f'switched off ({r.message})')
    p0, q0 = n.tip
    log, _ = run_motion(4.0, args.amp, 0.0, args.hz)
    _, mx_off = stats('RCM off, same lateral motion', log)
    check(mx_off > 5.0, f'without the RCM the shaft leaves the point (max {mx_off:.1f} mm > 5 mm): '
          'the metric can fail')

    # 7. plain "on" after moving with the RCM off: the point lands on the NEW shaft at the
    #    last engaged depth (set_rcm.py on), not back where it was.
    time.sleep(0.5)
    r = n.rcm_on(depth=0.0)
    p_new, q_new = n.tip
    axis_new = quat_to_matrix(q_new) @ np.array([0, 0, 1])
    expected_point = p_new - DEPTH * axis_new
    applied = np.array([r.applied_point.x, r.applied_point.y, r.applied_point.z])
    moved = np.linalg.norm(applied - rcm_point)
    check(r.success and abs(r.insertion_depth - DEPTH) < 1e-4
          and np.linalg.norm(applied - expected_point) < 1e-3 and moved > 1e-3,
          f'plain on relocates: depth {1e3 * r.insertion_depth:.1f} mm (last {1e3 * DEPTH:.0f}), '
          f'point moved {1e3 * moved:.1f} mm with the arm ({r.message})')
    n.rcm_off()

    print('\nALL PASS' if not failures else f'\n{len(failures)} FAILED: {failures}')
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
