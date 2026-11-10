#!/usr/bin/env python3
"""Live pivot monitor for aligning a shaft-mounted marker with the RCM (RCM-drift benchmark).

Platform-neutral: reads only the tracker topic (geometry_msgs/PoseStamped), so run it in the
tracker's shell (source ~/atracsys_ws/install/setup.bash):

    python3 pivot_monitor.py --topic /atracsys/<tool>/measured_cp [--window 6] [--threshold-mm 0.5]

While the arm pivots the instrument about its RCM (rcm_motion_real_arm.py wobble), every second
it fits the pivot over the last `window` s (pivot_fit.fit) and prints:
  |d|     how far the marker frame's ORIGIN is from the point the motion pivots about, in mm,
          and d itself in the marker frame
  along / across   d split along the shaft and across it. The shaft direction in the marker
          frame comes from the data: in the wobble (no roll) every rotation from the wobble's
          centre is about an axis normal to the shaft, so the shaft is the direction normal to
          all the body-frame rotation vectors (smallest singular vector). `along` (signed; its sign is only
          the marker frame's) is what sliding changes: slide until it is ~0. `across` is the
          origin's distance from the shaft axis, which sliding cannot change (with the
          pivot-calibrated geometry it is ~0 only if that point lies on this shaft's axis)
  origin  how much the origin moves (RMS about its mean), the fit-free version of the same
  fit     the pivot error RMS of the fit and the rotation range it saw (needs >= 10 deg)
ALIGNED when |along| <= threshold with enough rotation (the slide is done). --duration S and --json OUT (the last fit)
are for tests and logs.
"""
import argparse
import json
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pivot_fit  # noqa: E402


class Monitor(Node):

    def __init__(self, args):
        super().__init__('pivot_monitor')
        self.args = args
        self.buf = deque()
        self.last_keep = 0.0
        self.create_subscription(PoseStamped, args.topic, self._on_pose, 100)

    def _on_pose(self, m):
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return    # the tracker node keeps publishing the LAST pose with a zero stamp while the marker is not seen (2026-11-07)
        now = time.monotonic()
        if now - self.last_keep < 1.0 / self.args.rate:
            return
        self.last_keep = now
        p, o = m.pose.position, m.pose.orientation
        self.buf.append((now, np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])))
        while self.buf and now - self.buf[0][0] > self.args.window:
            self.buf.popleft()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--topic', required=True)
    ap.add_argument('--window', type=float, default=6.0, help='s of samples per fit')
    ap.add_argument('--rate', type=float, default=50.0, help='Hz of samples kept')
    ap.add_argument('--threshold-mm', type=float, default=0.5)
    ap.add_argument('--min-range-deg', type=float, default=10.0)
    ap.add_argument('--duration', type=float, default=float('inf'))
    ap.add_argument('--json')
    args = ap.parse_args()
    rclpy.init()
    n = Monitor(args)
    t_start = time.monotonic()
    last, out = 0.0, None
    try:
        while time.monotonic() - t_start < args.duration:
            rclpy.spin_once(n, timeout_sec=0.02)
            if time.monotonic() - last < 1.0:
                continue
            last = time.monotonic()
            if not n.buf or last - n.buf[-1][0] > 0.2:
                print('  marker not visible', flush=True)
                continue
            if len(n.buf) < 10:
                continue
            pos = np.array([b[1] for b in n.buf])
            rot = Rotation.from_quat(np.array([b[2] for b in n.buf])).as_matrix()
            f = pivot_fit.fit(pos, rot)
            d_mm = 1e3 * f['d']
            # Shaft direction in the marker frame: normal to the body-frame rotation vectors,
            # taken relative to the window's MEAN orientation (the wobble's centre). Relative to
            # the first sample instead, the cone's twist tilts the estimate (7.5 deg at 15 deg
            # amplitude, checked offline). Sign fixed by the largest component, so it does not
            # flip between windows.
            rs = Rotation.from_matrix(rot)
            rv = (rs.mean().inv() * rs).as_rotvec()
            u = np.linalg.svd(rv, full_matrices=False)[2][-1]
            u = -u if u[np.argmax(np.abs(u))] < 0 else u
            along = float(d_mm @ u)
            across = float(np.linalg.norm(d_mm - along * u))
            origin_mm = 1e3 * np.sqrt(np.mean(np.sum((pos - pos.mean(axis=0)) ** 2, axis=1)))
            ok = abs(along) <= args.threshold_mm and f['range_deg'] >= args.min_range_deg
            flag = 'ALIGNED' if ok else ('   more rotation' if f['range_deg'] < args.min_range_deg
                                         else '        ')
            print(f'{flag:16s} along {along:+8.2f} mm  across {across:6.2f} mm  |d| '
                  f'{np.linalg.norm(d_mm):7.2f} mm  origin {origin_mm:6.2f} mm RMS  fit '
                  f'{1e3 * f["rmse"]:.3f} mm over {f["range_deg"]:.0f} deg', flush=True)
            out = {'d_mm': d_mm.tolist(), 'd_norm_mm': float(np.linalg.norm(d_mm)),
                   'along_mm': along, 'across_mm': across, 'shaft_in_marker': u.tolist(),
                   'origin_rms_mm': float(origin_mm), 'fit_rmse_mm': 1e3 * f['rmse'],
                   'range_deg': f['range_deg'], 'aligned': bool(ok), 'samples': len(pos)}
    except KeyboardInterrupt:
        pass
    finally:
        if args.json and out:
            with open(args.json, 'w') as fh:
                json.dump(out, fh, indent=1)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
