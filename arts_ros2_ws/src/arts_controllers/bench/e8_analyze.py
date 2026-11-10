#!/usr/bin/env python3
"""E8 analysis: ARM HOLD stopping time, stopping distance, hold drift and resume jump.

    python3 e8_analyze.py BAG_DIR [--out RESULT.json]

From a run_e8.sh bag. The tool is the UR's own measured TCP pose
([/<arm>]/tcp_pose_broadcaster/pose, stamped in the control loop); the stop is the UR's report
(the driver's safety_mode topic in bags before decision 18, the safety_mode field of
/<arm>/status after; published on change, timed by bag receive time).
Per SAFEGUARD_STOP onset t_s, with the held pose = mean tool position 1-2 s after t_s and
positions averaged over 10 ms (the UR's pose is steady to ~10 um while frozen; speeds
differentiated from it are not, because its stamps jitter):
  speed       tool speed over the 20 ms before t_s;
  t_1mm       last time within 2 s that the tool is more than 1 mm from the held pose: the
              end of the deceleration;
  t_0.1mm     the same for 0.1 mm: the end of the settle that follows (synthetic bag,
              5 m/s^2 from 250 mm/s: 40 / 54 ms for a true 60 ms stop,
              as the deceleration predicts: 1 mm and 0.1 mm short of rest);
  distance    held pose minus the tool at t_s;
  drift       mean pose over the last 1 s before the next NORMAL minus the held pose, and
              the largest 10 ms-average deviation from the held pose in between;
  resume      largest 10 ms-average deviation, over the 1.5 s after NORMAL, from the mean
              pose of the last 1 s before it (e8_hold_real_arm.py commands nothing until
              3 s after resume).
The delay from the button to the UR's report (relay plus safety input) is not in the bag.
RUN rule: source arts_ros2_ws/install only (rosbag2_py).
"""
import argparse
import json

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SAFEGUARD_STOP, NORMAL = 5, 1


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    tcp, safety = [], []
    while r.has_next():
        topic, raw, t_recv = r.read_next()
        # Both namings: the 2026-11-01 bag (/tcp_pose_broadcaster/pose, the UR driver's
        # safety_mode topic) and decision 18 (/<arm>/tcp_pose_broadcaster/pose, /<arm>/status).
        if topic.endswith('tcp_pose_broadcaster/pose'):
            m = deserialize_message(raw, types[topic])
            p = m.pose.position
            tcp.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
        elif topic.endswith('io_and_status_controller/safety_mode') or topic.endswith('/status'):
            mode = deserialize_message(raw, types[topic])
            mode = mode.mode if hasattr(mode, 'mode') else mode.safety_mode
            if not safety or safety[-1][1] != mode:  # ArmStatus repeats on other changes
                safety.append((t_recv * 1e-9, mode))
    tcp = np.array(tcp)
    _, k = np.unique(tcp[:, 0], return_index=True)
    return tcp[k], safety


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out')
    args = ap.parse_args()
    tcp, safety = read(args.bag)
    t, p = tcp[:, 0], tcp[:, 1:]
    dt = np.median(np.diff(t))
    v = np.gradient(p, t, axis=0)
    k = max(int(round(0.010 / dt)), 1)
    pavg = np.stack([np.convolve(np.pad(p[:, j], (k // 2, k - 1 - k // 2), mode='edge'),
                                 np.ones(k) / k, 'valid') for j in range(3)], 1)
    print(f'tcp pose {len(t)} samples, {1 / dt:.0f} Hz; safety mode changes: '
          + ', '.join(f'{m}' for _, m in safety))
    res = []
    for i, (ts, mode) in enumerate(safety):
        if mode != SAFEGUARD_STOP:
            continue
        tn = next((tt for tt, mm in safety[i + 1:] if mm == NORMAL), None)
        pre = (t >= ts - 0.020) & (t < ts)
        v0 = float(np.linalg.norm(v[pre].mean(axis=0))) if pre.any() else float('nan')
        held = pavg[(t >= ts + 1.0) & (t < ts + 2.0)].mean(axis=0)
        win = (t >= ts) & (t < ts + 2.0)
        d = np.linalg.norm(pavg[win] - held, axis=1)
        tw = t[win] - ts
        ps = np.array([np.interp(ts, t, p[:, j]) for j in range(3)])
        r = dict(t_stop=ts, speed_mm_s=1e3 * v0,
                 t_1mm_ms=1e3 * float(tw[d > 1e-3][-1]) if (d > 1e-3).any() else 0.0,
                 t_01mm_ms=1e3 * float(tw[d > 1e-4][-1]) if (d > 1e-4).any() else 0.0,
                 stop_distance_mm=1e3 * float(np.linalg.norm(held - ps)))
        if tn is not None and tn > ts + 3.0:
            last = pavg[(t >= tn - 1.0) & (t < tn)].mean(axis=0)
            during = (t >= ts + 1.0) & (t < tn)
            after = (t >= tn) & (t < tn + 1.5)
            r['hold_s'] = float(tn - ts)
            r['drift_mm'] = 1e3 * float(np.linalg.norm(last - held))
            r['drift_max_mm'] = 1e3 * float(np.linalg.norm(pavg[during] - held, axis=1).max())
            r['resume_jump_mm'] = 1e3 * float(np.linalg.norm(pavg[after] - last, axis=1).max())
        res.append(r)
        print(f"stop {len(res)}: {r['speed_mm_s']:6.1f} mm/s -> within 1 mm at {r['t_1mm_ms']:5.1f} ms, "
              f"0.1 mm at {r['t_01mm_ms']:5.1f} ms, distance {r['stop_distance_mm']:5.2f} mm; "
              + (f"held {r['hold_s']:.1f} s, drift {r['drift_mm']:.3f} mm (max "
                 f"{r['drift_max_mm']:.3f}), resume {r['resume_jump_mm']:.3f} mm"
                 if 'hold_s' in r else 'no reset'))
    for lo, hi in ((0.05, 0.15), (0.15, 0.30)):
        g = [r for r in res if lo <= r['speed_mm_s'] * 1e-3 < hi]
        if g:
            a, b, sd = (np.array([r[k] for r in g]) for k in ('t_1mm_ms', 't_01mm_ms', 'stop_distance_mm'))
            print(f'{lo * 1e3:.0f}-{hi * 1e3:.0f} mm/s: n={len(g)}, within 1 mm {a.mean():.1f} +- {a.std():.1f} ms '
                  f'(max {a.max():.1f}), within 0.1 mm {b.mean():.1f} +- {b.std():.1f} ms (max {b.max():.1f}), '
                  f'distance {sd.mean():.2f} +- {sd.std():.2f} mm (max {sd.max():.2f})')
    h = [r for r in res if 'hold_s' in r]
    if h:
        print(f"all holds (n={len(h)}): drift max {max(r['drift_mm'] for r in h):.3f} mm, largest "
              f"deviation {max(r['drift_max_mm'] for r in h):.3f} mm; resume jump max "
              f"{max(r['resume_jump_mm'] for r in h):.3f} mm")
    if args.out:
        with open(args.out, 'w') as fh:
            json.dump(res, fh, indent=1)


if __name__ == '__main__':
    main()
