#!/usr/bin/env python3
"""E5 analyzer: servo loop health of one arm's controller_manager from a recorded bag.

The joint_state_broadcaster runs inside the controller_manager loop and stamps every message
with that cycle's time, so consecutive /joint_states stamps ARE the loop's periods: one
message per cycle, 500 Hz for a UR5e manager. The stamps are written by the loop itself, so
DDS and the recorder add no jitter to them. A doubled period is EITHER a cycle the loop
skipped OR a message lost between the broadcaster and the bag, and the stamps alone cannot
tell which (E5, 2026-10-29: four gaps, none of them a loop overrun). The launch log's
"Overrun detected" warnings and the controller_manager's own periodicity statistics in
/diagnostics decide it; both are reported.

  analyze_e5.py BAG [--rate 500] [--launch-log LOG] [--json OUT]

Reported: cycle count, period mean / std / min / max / p99.9, jitter as std over period,
gap cycles (a period longer than 1.5 nominal counts floor(period/nominal + 0.5) - 1), and, from the launch log if given, the controller_manager's own overrun warnings.
"""
import argparse
import json
import re
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def stamps(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, JointState)
        out.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
    return np.array(out, dtype=np.int64)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/joint_states',
                    help='/joint_states for bags before decision 18 (2026-10-29); /<arm>/joint_states after')
    ap.add_argument('--rate', type=float, default=500.0)
    ap.add_argument('--launch-log')
    ap.add_argument('--json')
    args = ap.parse_args()

    t = stamps(args.bag, args.topic)
    if len(t) < 2:
        sys.exit('fewer than two messages on %s' % args.topic)
    if np.any(np.diff(t) <= 0):
        print('WARNING: %d non-increasing stamps (duplicates or reordering)'
              % int(np.sum(np.diff(t) <= 0)))
    period_us = np.diff(np.sort(t)) / 1e3
    nominal_us = 1e6 / args.rate
    long = period_us > 1.5 * nominal_us
    misses = int(np.sum(np.floor(period_us[long] / nominal_us + 0.5) - 1))
    res = dict(
        cycles=int(len(t)), duration_s=float((t[-1] - t[0]) / 1e9),
        rate_hz=float((len(t) - 1) / ((t[-1] - t[0]) / 1e9)),
        period_mean_us=float(period_us.mean()), period_std_us=float(period_us.std()),
        period_min_us=float(period_us.min()), period_max_us=float(period_us.max()),
        period_p999_us=float(np.percentile(period_us, 99.9)),
        jitter_pct_of_period=float(100 * period_us.std() / nominal_us),
        long_periods=int(long.sum()), gap_cycles=misses,
        max_abs_deviation_us=float(np.abs(period_us - nominal_us).max()))

    if args.launch_log:
        lines = open(args.launch_log, errors='replace').read().splitlines()
        res['overrun_warnings'] = sum('Overrun detected' in l for l in lines)
        res['fifo_ok'] = any('Successful set up FIFO RT scheduling' in l for l in lines)
        res['calibration_mismatch'] = any("calibration parameters of the connected robot don't match"
                                          in l for l in lines)

    print('cycles %d over %.1f s -> %.3f Hz (nominal %.0f)'
          % (res['cycles'], res['duration_s'], res['rate_hz'], args.rate))
    print('period %.2f +- %.2f us (min %.1f, max %.1f, p99.9 %.1f); jitter %.2f %% of period'
          % (res['period_mean_us'], res['period_std_us'], res['period_min_us'],
             res['period_max_us'], res['period_p999_us'], res['jitter_pct_of_period']))
    print('periods > 1.5x nominal: %d; gap cycles: %d (skipped cycles OR lost messages; see overruns)'
          % (res['long_periods'], res['gap_cycles']))
    if args.launch_log:
        print('launch log: %d overrun warnings, FIFO %s, calibration mismatch %s'
              % (res['overrun_warnings'], res['fifo_ok'], res['calibration_mismatch']))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
