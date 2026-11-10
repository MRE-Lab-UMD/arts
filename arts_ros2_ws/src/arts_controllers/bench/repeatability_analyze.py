#!/usr/bin/env python3
"""ISO 9283 pose repeatability from a repeatability bag (repeatability_real_arm.py).

    python3 repeatability_analyze.py BAG [--arm psm_left] [--tracker-topic T ...] [--json OUT]

The stops are read from /<arm>/bench_event; each stop's attained pose is the MEAN of a source's
samples inside its measurement window (window_start..window_end, by bag receive time, which is
one clock on one machine). Sources: the kinematic tip (/<arm>/tool_tip_state, forward kinematics
of the measured joints) and every tracker topic given or found (geometry_msgs/PoseStamped whose
name starts with /atracsys/, e.g. the marker's measured_cp).

Per pose j over n cycles (ISO 9283 7.2.2): barycentre x_bar of the attained positions,
l_i = |x_i - x_bar|, RP_j = mean(l) + 3 S_l (S_l with n - 1). Orientation: the attained
rotations relative to their mean rotation, as rotation-vector components a, b, c about the
source frame's axes, RP_a,b,c = +-3 S. Beside ISO, collaborator A's statistics of the same l
(mean, std with n - 1, RMS, max), per pose and pooled over all poses, so the dVRK Classic
numbers A sends compare one to one. The table reports every pose and the worst pose; a source
missing in a window drops that stop and says so. Misses (no static tracked window) and
discarded windows are counted from the events; the settling time (arrival to the measured
window) is summarized.
"""
import argparse
import json
import sys
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


def tracked(msg):
    """A tracker PoseStamped that is a measurement (header stamp set), not a repeat."""
    return msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = {t for t in types if t in topics or (t.startswith('/atracsys/') and
                                               types[t] == 'geometry_msgs/msg/PoseStamped')}
    r.set_filter(rosbag2_py.StorageFilter(topics=sorted(want)))
    out = defaultdict(list)
    while r.has_next():
        name, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[name]))
        # Not tracked: the tracker node keeps publishing the LAST pose with a zero stamp while the marker is not seen (2026-11-07).
        if types[name] == 'geometry_msgs/msg/PoseStamped' and not tracked(m):
            continue
        out[name].append((t * 1e-9, m))
    return out, types


def pose_of(msg):
    p = msg.pose.position
    q = msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def iso_rp(positions, quats):
    x = np.asarray(positions)
    l = np.linalg.norm(x - x.mean(axis=0), axis=1)
    rots = Rotation.from_quat(np.asarray(quats))
    rel = (rots.mean().inv() * rots).as_rotvec()
    return {'n': len(x), 'rp_mm': 1e3 * (l.mean() + 3 * l.std(ddof=1)),
            'l_mean_mm': 1e3 * l.mean(), 'l_std_mm': 1e3 * l.std(ddof=1),
            'l_rms_mm': 1e3 * np.sqrt(np.mean(l ** 2)), 'l_max_mm': 1e3 * l.max(),
            'l_mm': (1e3 * l).tolist(),
            'rp_abc_deg': (3 * np.degrees(rel.std(axis=0, ddof=1))).tolist()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--arm', default='psm_left')
    ap.add_argument('--tracker-topic', action='append', default=[])
    ap.add_argument('--json', help='write the results here')
    args = ap.parse_args()
    ev_topic, tip_topic = f'/{args.arm}/bench_event', f'/{args.arm}/tool_tip_state'
    data, types = read(args.bag, {ev_topic, tip_topic, *args.tracker_topic})
    if ev_topic not in data:
        sys.exit(f'no {ev_topic} in {args.bag}')

    windows, misses, discards, settle = {}, [], 0, []
    for t, m in data[ev_topic]:
        e = json.loads(m.data)
        key = (e.get('cycle'), e.get('pose'))
        if e['event'] == 'window_start':
            windows[key] = [t, None]      # a later start (after a discard) replaces it
        elif e['event'] == 'window_end' and key in windows:
            windows[key][1] = t
            settle.append(e.get('settle_s', np.nan))
        elif e['event'] == 'miss':
            misses.append(key)
        elif e['event'] == 'window_discard':
            discards += 1
    windows = {k: v for k, v in windows.items() if v[1] is not None}
    sources = [tip_topic] + sorted(s for s in data if s not in (ev_topic, tip_topic))
    print(f'{args.bag}: {len(windows)} complete stops, '
          f'{len({c for c, _ in windows})} cycles; sources {sources}')
    print(f'misses (no static tracked window): {len(misses)} {misses}; discarded windows: '
          f'{discards}')
    if settle:
        print(f'settling time (arrival to measured window): median {np.median(settle):.2f} s, '
              f'max {np.max(settle):.2f} s')

    results = {}
    for src in sources:
        stamps = np.array([t for t, _ in data[src]])
        poses = [pose_of(m) for _, m in data[src]]
        per_pose = defaultdict(lambda: ([], []))
        dropped = 0
        for (cycle, k), (t0, t1) in sorted(windows.items()):
            idx = np.nonzero((stamps >= t0) & (stamps <= t1))[0]
            if len(idx) == 0:
                dropped += 1
                continue
            pos = np.mean([poses[i][0] for i in idx], axis=0)
            quat = Rotation.from_quat([poses[i][1] for i in idx]).mean().as_quat()
            per_pose[k][0].append(pos)
            per_pose[k][1].append(quat)
        res = {f'P{k}': iso_rp(*per_pose[k]) for k in sorted(per_pose) if len(per_pose[k][0]) >= 2}
        pooled = np.concatenate([r['l_mm'] for r in res.values()]) if res else np.array([])
        overall = ({'n': len(pooled), 'l_mean_mm': pooled.mean(), 'l_std_mm': pooled.std(ddof=1),
                    'l_rms_mm': np.sqrt(np.mean(pooled ** 2)), 'l_max_mm': pooled.max()}
                   if len(pooled) > 1 else {})
        results[src] = {'poses': res, 'overall': overall, 'dropped_stops': dropped,
                        'misses': [list(m) for m in misses], 'discarded_windows': discards,
                        'settle_s': settle}
        print(f'\n{src} (stops without a sample: {dropped})')
        print('  pose   n   RP mm | mean l   std l   RMS l   max l (mm) | RPa/RPb/RPc deg')
        for name, r in res.items():
            a, b, c = r['rp_abc_deg']
            print(f'  {name:4s} {r["n"]:3d} {r["rp_mm"]:7.3f} | {r["l_mean_mm"]:6.3f}  '
                  f'{r["l_std_mm"]:6.3f}  {r["l_rms_mm"]:6.3f}  {r["l_max_mm"]:6.3f}     | '
                  f'{a:.3f}/{b:.3f}/{c:.3f}')
        if res:
            worst = max(res, key=lambda k: res[k]['rp_mm'])
            print(f'  worst pose {worst}: RP {res[worst]["rp_mm"]:.3f} mm')
            print(f'  all poses pooled (A\'s overall): n {overall["n"]}, mean {overall["l_mean_mm"]:.4f}, '
                  f'std {overall["l_std_mm"]:.4f}, RMS {overall["l_rms_mm"]:.4f}, '
                  f'max {overall["l_max_mm"]:.4f} mm')
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results, f, indent=1)


if __name__ == '__main__':
    main()
