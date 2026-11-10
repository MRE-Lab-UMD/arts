#!/usr/bin/env python3
"""RCM drift from a tracker bag (rcm_motion_real_arm.py drift trials, run_rcm_drift.sh).

    python3 rcm_drift_analyze.py BAG [--arm psm_left] [--tracker-topic T] [--json OUT]

Each trial is one start..done pair of /<arm>/bench_event with mode drift; the marker samples
inside it (by bag receive time) are the data. Physically measured, from the tracker only:
  pivot error   pivot_fit over the whole trial: the shaft-marker point d that stays most fixed
                and where it stays (t); per sample |R_i d + p_i - t|, reported as mean +- std
                (DASH's form), RMS, p95, max. This is the external RCM deviation.
  origin        the marker origin's distance from its own mean, fit-free: equal to the pivot
                error when the marker origin was aligned onto the RCM (|d| small, also reported)
  drift         the fitted point per --window-s window (60 s) (d held at the trial's fit), its displacement
                from the first window, and the slope of a line through the window positions
                (mm/min, the norm of the vector slope)
Beside it, the controller's KINEMATIC deviation (/<arm>/rcm_state, the open platform's
definition) over the same time: median, p95, max.
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pivot_fit  # noqa: E402
from repeatability_analyze import read  # noqa: E402



def trials(events):
    out, cur = [], None
    for t, m in events:
        e = json.loads(m.data)
        if e['event'] == 'start' and e.get('mode') == 'drift':
            cur = [t, None, e]
        elif e['event'] in ('done', 'abort') and cur is not None:
            cur[1] = t
            cur[2] = dict(cur[2], result=e.get('result', e['event']))
            out.append(cur)
            cur = None
    return out


def analyze(ts, pos, rot, kin_t, kin_dev, window_s=60.0):
    f = pivot_fit.fit(pos, rot)
    err = 1e3 * f['err']
    origin = 1e3 * np.linalg.norm(pos - pos.mean(axis=0), axis=1)
    rel = ts - ts[0]
    wins = []
    for a in np.arange(0.0, rel[-1] - 0.5 * window_s + 1e-9, window_s):
        sel = (rel >= a) & (rel < a + window_s)
        if sel.sum() >= 10:
            wins.append((a + 0.5 * min(window_s, rel[-1] - a), f['tips'][sel].mean(axis=0)))
    drift = {}
    if len(wins) >= 2:
        wt = np.array([w[0] for w in wins]) / 60.0
        wp = 1e3 * np.array([w[1] for w in wins])
        slope = np.array([np.polyfit(wt, wp[:, k], 1)[0] for k in range(3)])
        drift = {'windows': len(wins), 'window_s': window_s, 'displacement_mm': np.linalg.norm(wp - wp[0], axis=1).tolist(),
                 'max_displacement_mm': float(np.linalg.norm(wp - wp[0], axis=1).max()),
                 'slope_mm_per_min': float(np.linalg.norm(slope))}
    kin = 1e3 * kin_dev[np.isfinite(kin_dev)]
    return {
        'samples': len(ts), 'duration_s': float(rel[-1]),
        'rotation_range_deg': f['range_deg'], 'fit_cond': f['cond'],
        'd_mm': (1e3 * f['d']).tolist(), 'd_norm_mm': float(1e3 * np.linalg.norm(f['d'])),
        'pivot_error_mm': {'mean': err.mean(), 'std': err.std(ddof=1),
                           'rms': float(np.sqrt(np.mean(err ** 2))),
                           'p95': float(np.percentile(err, 95)), 'max': err.max()},
        'origin_mm': {'mean': origin.mean(), 'std': origin.std(ddof=1), 'max': origin.max()},
        'drift': drift,
        'kinematic_mm': ({'median': float(np.median(kin)), 'p95': float(np.percentile(kin, 95)),
                          'max': float(kin.max()), 'n': len(kin)} if len(kin) else {}),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--arm', default='psm_left')
    ap.add_argument('--tracker-topic', help='default: the one /atracsys/* PoseStamped topic')
    ap.add_argument('--window-s', type=float, default=60.0, help='drift window')
    ap.add_argument('--json')
    args = ap.parse_args()
    ev, rcm = f'/{args.arm}/bench_event', f'/{args.arm}/rcm_state'
    data, types = read(args.bag, {ev, rcm} | ({args.tracker_topic} if args.tracker_topic else set()))
    trk = args.tracker_topic or next((t for t in sorted(data) if t.startswith('/atracsys/')), None)
    if trk not in data:
        sys.exit(f'no tracker topic in {args.bag} ({sorted(data)})')
    tt = np.array([t for t, _ in data[trk]])
    pp = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in data[trk]])
    rr = Rotation.from_quat([[m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w] for _, m in data[trk]]).as_matrix()
    kt = np.array([t for t, _ in data.get(rcm, [])])
    kd = np.array([m.deviation if m.enabled else np.nan for _, m in data.get(rcm, [])])
    results = []
    for i, (a, b, info) in enumerate(trials(data.get(ev, []))):
        sel = (tt >= a) & (tt <= b)
        if sel.sum() < 100:
            print(f'trial {i + 1}: only {sel.sum()} tracker samples, skipped')
            continue
        ksel = (kt >= a) & (kt <= b) if len(kt) else np.zeros(0, bool)
        r = analyze(tt[sel], pp[sel], rr[sel], kt[ksel] if len(kt) else kt,
                    kd[ksel] if len(kd) else kd, args.window_s)
        r.update(trial=i + 1, seed=info.get('args', {}).get('seed'), result=info['result'],
                 tracker_topic=trk)
        results.append(r)
        e, o, dr, k = r['pivot_error_mm'], r['origin_mm'], r['drift'], r['kinematic_mm']
        print(f'trial {r["trial"]} (seed {r["seed"]}, {r["result"]}): {r["duration_s"]:.0f} s, '
              f'{r["samples"]} samples, rotation range {r["rotation_range_deg"]:.0f} deg')
        print(f'  pivot error (tracker): {e["mean"]:.3f} +- {e["std"]:.3f} mm (mean +- std), '
              f'RMS {e["rms"]:.3f}, p95 {e["p95"]:.3f}, max {e["max"]:.3f} mm')
        print(f'  marker origin from the pivot |d| {r["d_norm_mm"]:.2f} mm; origin motion '
              f'{o["mean"]:.3f} +- {o["std"]:.3f} mm, max {o["max"]:.3f} mm')
        if dr:
            print(f'  drift: {dr["windows"]} x {args.window_s:.0f} s windows, max displacement '
                  f'{dr["max_displacement_mm"]:.3f} mm, slope {dr["slope_mm_per_min"]:.4f} mm/min')
        if k:
            print(f'  kinematic (rcm_state): median {k["median"]:.4f}, p95 {k["p95"]:.4f}, '
                  f'max {k["max"]:.4f} mm')
    if not results:
        sys.exit(f'no drift trial (start mode drift .. done) in {ev}')
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(results, fh, indent=1, default=float)


if __name__ == '__main__':
    main()
