#!/usr/bin/env python3
"""Marker geometry from a static recording of its raw fiducials (optical tracker, any platform).

Run in the tracker's shell with the marker STILL and fully visible:

    python3 tracker_geometry_maker.py --topic /atracsys/measured_cp_array --count 4 \
        --seconds 10 --id 1001 --out geometry-prograsp-20261107.json [--compare geometry-prograsp.json]

Keeps the frames that show exactly `count` fiducials, matches them to the first frame by
nearest neighbour, averages, and writes the tracker's JSON geometry format (mm): origin at the
fiducials' centroid, axes along their principal directions (largest spread = x, right-handed),
pivot zero. Reports per-fiducial scatter, pairwise distances and the per-frame fiducial
registration error (FRE). With --compare, the pairwise distances of an existing geometry beside
the new ones, and the rigid-fit residual of each fiducial (which one moved).
"""
import argparse
import itertools
import json
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from scipy.spatial.distance import cdist


def rigid_fit(a, b):
    """R, t minimizing |R a + t - b| (rows are points)."""
    a0, b0 = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - a0).T @ (b - b0))
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    r = vt.T @ d @ u.T
    return r, b0 - r @ a0


def pair_dists(p):
    return np.array([np.linalg.norm(p[i] - p[j]) for i, j in itertools.combinations(range(len(p)), 2)])


class Recorder(Node):

    def __init__(self, topic, count):
        super().__init__('tracker_geometry_maker')
        self.count = count
        self.ref = None
        self.frames = []
        self.skipped = 0
        self.create_subscription(PoseArray, topic, self._on, 50)

    def _on(self, m):
        p = np.array([[q.position.x, q.position.y, q.position.z] for q in m.poses])
        if len(p) > 1:
            # Drop isolated reflections: a marker fiducial has a neighbour within 0.15 m.
            d = cdist(p, p) + np.eye(len(p)) * 1e9
            p = p[d.min(axis=1) < 0.15]
        if len(p) != self.count:
            self.skipped += 1
            return
        if self.ref is None:
            self.ref = p
        idx = cdist(self.ref, p).argmin(axis=1)
        if len(set(idx)) != self.count:
            self.skipped += 1
            return
        self.frames.append(p[idx])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--topic', default='/atracsys/measured_cp_array')
    ap.add_argument('--count', type=int, default=4)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--id', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--compare')
    args = ap.parse_args()
    if os.path.exists(args.out):
        sys.exit(f'{args.out} exists; refusing to overwrite')
    rclpy.init()
    n = Recorder(args.topic, args.count)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    data = 1e3 * np.array(n.frames)            # (N, M, 3) mm
    n.destroy_node()
    rclpy.shutdown()
    if len(data) < 50:
        sys.exit(f'only {len(data)} frames with {args.count} fiducials ({n.skipped} skipped): '
                 'is the marker fully visible and nothing else reflective in view?')
    avg = data.mean(axis=0)
    c = avg - avg.mean(axis=0)
    _, sv, vt = np.linalg.svd(c, full_matrices=False)
    if np.linalg.det(vt) < 0:
        vt[2] = -vt[2]
    pts = c @ vt.T
    geom = {'count': args.count,
            'fiducials': [{'x': float(x), 'y': float(y), 'z': float(z)} for x, y, z in pts],
            'id': args.id, 'pivot': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    with open(args.out, 'w') as fh:
        json.dump(geom, fh, indent=4)
        fh.write('\n')

    scatter = np.linalg.norm(data - avg, axis=2)          # (N, M)
    fre = [np.sqrt(np.mean(np.sum(((rigid_fit(f, avg)[0] @ f.T).T + rigid_fit(f, avg)[1] - avg) ** 2,
                                  axis=1))) for f in data]
    print(f'{len(data)} frames ({n.skipped} skipped) over {args.seconds:.0f} s -> {args.out}')
    print('fiducials in the marker frame (mm) and their scatter (RMS about the mean):')
    for i, p in enumerate(pts):
        print(f'  #{i + 1}  {p[0]:9.3f} {p[1]:9.3f} {p[2]:9.3f}   scatter '
              f'{np.sqrt(np.mean(scatter[:, i] ** 2)):.4f}')
    print(f'principal spreads (mm): {np.round(sv, 2)} (the last = out-of-plane)')
    print(f'FRE per frame: {np.mean(fre):.4f} +- {np.std(fre):.4f} mm')
    d_new = pair_dists(pts)
    print('pairwise distances (mm):', np.round(np.sort(d_new), 2))
    if args.compare:
        old = np.array([[f['x'], f['y'], f['z']] for f in json.load(open(args.compare))['fiducials']])
        print(f'{args.compare} pairwise:', np.round(np.sort(pair_dists(old)), 2))
        best = min(itertools.permutations(range(len(old))),
                   key=lambda pm: np.abs(np.sort(pair_dists(old)) - np.sort(d_new)).sum() +
                   np.linalg.norm((rigid_fit(old, pts[list(pm)])[0] @ old.T).T +
                                  rigid_fit(old, pts[list(pm)])[1] - pts[list(pm)]))
        r, t = rigid_fit(old, pts[list(best)])
        res = np.linalg.norm((r @ old.T).T + t - pts[list(best)], axis=1)
        print('rigid fit old -> new, residual per old fiducial (mm):', np.round(res, 2),
              f'RMS {np.sqrt(np.mean(res ** 2)):.2f}')


if __name__ == '__main__':
    main()
