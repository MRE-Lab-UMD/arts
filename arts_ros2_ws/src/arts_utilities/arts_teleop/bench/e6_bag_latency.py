#!/usr/bin/env python3
"""E6 alongside: the software-path share of the teleop latency, from the session bag.

    python3 e6_bag_latency.py BAG_DIR [--side left]

Same lag estimator as e6_latency.py (arm velocity at t + L fitted as a linear map of handle
velocity at t), on bag stamps instead of video frames: the handle is /haply/<side>/cursor,
stamped by haply_bridge on receipt of each service frame; the arm's velocity is the joint velocity from /joint_states (the UR's measured joint positions,
stamped on receipt by the driver), mapped from the handle's Cartesian velocity by the same
fitted linear map -- a local linearisation, valid for the few-cm teleop workspace. (Composing
base_link -> tool0 from /tf instead quantised the lag to 51 / 102 ms, 2026-10-30: each /tf
message carries only some of the chain.)
The difference from the video lag is what lies outside both stamps: the Haply's own sensing
and transport to the service, and the UR's joint-position report path.

RUN rule: source arts_ros2_ws/install only (rosbag2_py).
"""
import argparse
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import e6_latency as E  # noqa: E402


def read(bag, side):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    handle, joints = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        # Both namings: /haply/<side>/cursor + /joint_states (bags before decision 18, e.g. E6
        # 2026-10-30) and /mtm_<side>/cursor + /<arm>/joint_states (after).
        if topic in (f'/haply/{side}/cursor', f'/mtm_{side}/cursor'):
            m = deserialize_message(raw, types[topic])
            handle.append((stamp(m), m.point.x * 1e3, m.point.y * 1e3, m.point.z * 1e3))
        elif topic.endswith('/joint_states') and not topic.startswith('/mtm'):
            m = deserialize_message(raw, types[topic])
            joints.append((stamp(m), *np.degrees(m.position)))
    return [unique_stamps(np.array(x)) for x in (handle, joints)]


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def unique_stamps(a):
    _, k = np.unique(a[:, 0], return_index=True)
    return a[k]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--side', default='left')
    args = ap.parse_args()
    h, a = read(args.bag, args.side)
    g = np.arange(max(h[0, 0], a[0, 0]) + 0.1, min(h[-1, 0], a[-1, 0]) - 0.1, E.GRID)
    vh, va = E.velocity(h[:, 0], h[:, 1:], g), E.velocity(a[:, 0], a[:, 1:], g)
    moving = np.linalg.norm(va, axis=1) > 0.5  # deg/s
    idx = np.where(moving)[0]
    L, r2, _ = E.fit_lag(vh, va, idx)
    wins = []
    w, s = int(E.WINDOW / E.GRID), int(E.STEP / E.GRID)
    for i in range(0, len(g) - w - int(E.MAX_LAG / E.GRID), s):
        sel = np.arange(i, i + w)
        sel = sel[moving[sel]]
        if len(sel) >= 0.5 * w:
            Lw, rr, _ = E.fit_lag(vh, va, sel)
            if rr > 0.8:
                wins.append(Lw)
    w = np.array(wins)
    print(f'{args.bag}: arm moving {len(idx) * E.GRID:.1f} s; lag handle stamp -> measured joints '
          f'{L} ms (R^2 {r2:.3f}); windows n={len(w)}' +
          (f', median {np.median(w):.0f}, mean {w.mean():.1f} +- {w.std():.1f}, range {w.min()}-{w.max()} ms'
           if len(w) else ''))


if __name__ == '__main__':
    main()
