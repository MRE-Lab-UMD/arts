#!/usr/bin/env python3
"""E6 step 2: teleop motion-to-motion latency from the marker tracks of e6_track_markers.py.

    python3 e6_latency.py track_A.csv [track_B.csv ...] [--out RESULT.json]

Handle (marker 0) and arm adapter (marker 1) are filmed in one frame, so one clock serves
both. The two move in different directions in the image, at a different scale (teleop
scale, perspective, the arm's mounting), so no axis is matched by hand. Instead, for each
candidate lag L the arm's image velocity at t + L is fitted as a linear map (2x2, least
squares) of the handle's image velocity at t; the lag is the L with the best fit (R^2).

  main number: that lag over each whole take (1 ms grid, positions interpolated from the
               120 fps frames, 30 ms smoothing), plus the lag per 3 s window where both
               markers move, whose spread is reported;
  check:       motion onsets -- the handle leaves rest (speed above 20 mm/s after 0.5 s
               below) and the arm crosses the matched threshold (20 mm/s times the fitted
               arm/handle speed gain); the onset difference per event.

Positions are converted to mm with each marker's own side length (40 mm). Clutch-off
stretches, where the handle moves and the arm does not, carry no lag information; the
windows keep only those where the arm moves too.
"""
import argparse
import json

import numpy as np

SIDE_MM = 40.0
GRID = 0.001          # s
SMOOTH = 0.030        # s, moving average on the 1 ms grid before differencing
MAX_LAG = 0.300       # s
WINDOW, STEP = 3.0, 1.0
ONSET_MMS, REST_S = 20.0, 0.5


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    out = {}
    for m in (0, 1):
        ok = np.isfinite(d[f'u{m}'])
        s = np.nanmedian(d[f'side{m}'])
        out[m] = (d['t'][ok], np.stack([d[f'u{m}'][ok], d[f'v{m}'][ok]], 1) * SIDE_MM / s)
    return out


def velocity(t, p, g):
    x = np.stack([np.interp(g, t, p[:, k]) for k in range(p.shape[1])], 1)
    k = int(SMOOTH / GRID)
    x = np.pad(x, ((k // 2, k - 1 - k // 2), (0, 0)), mode='edge')  # zero padding fakes a step
    x = np.stack([np.convolve(x[:, j], np.ones(k) / k, 'valid') for j in range(x.shape[1])], 1)
    return np.gradient(x, GRID, axis=0)


def fit_lag(vh, va, idx):
    """Best lag (ms) and R^2 for arm velocity at idx + L as a linear map of handle velocity at idx."""
    best = (0, -np.inf, None)
    for L in range(int(MAX_LAG / GRID)):
        j = idx + L
        j = j[j < len(va)]
        X, Y = vh[j - L], va[j]
        A, *_ = np.linalg.lstsq(X, Y, rcond=None)
        r2 = 1 - np.sum((Y - X @ A) ** 2) / np.sum((Y - Y.mean(0)) ** 2)
        if r2 > best[1]:
            best = (L, r2, A)
    return best


def take(path):
    tr = load(path)
    (th, ph), (ta, pa) = tr[0], tr[1]
    g = np.arange(max(th[0], ta[0]) + 0.1, min(th[-1], ta[-1]) - 0.1, GRID)
    # do not interpolate across handle dropouts longer than 50 ms
    gap = np.interp(g, th[1:], np.diff(th)) > 0.05
    vh, va = velocity(th, ph, g), velocity(ta, pa, g)
    sh, sa = np.linalg.norm(vh, axis=1), np.linalg.norm(va, axis=1)
    moving = (sa > 5.0) & ~gap
    idx = np.where(moving)[0]
    lag, r2, A = fit_lag(vh, va, idx)
    gain = np.median(sa[idx]) / max(np.median(sh[idx]), 1e-9)

    wins = []
    n, w, s = len(g), int(WINDOW / GRID), int(STEP / GRID)
    for a in range(0, n - w - int(MAX_LAG / GRID), s):
        sel = np.arange(a, a + w)
        sel = sel[moving[sel]]
        if len(sel) < 0.5 * w:
            continue
        L, rr, _ = fit_lag(vh, va, sel)
        if rr > 0.8:
            wins.append(L)

    onsets = []
    rest = int(REST_S / GRID)
    thr_a = ONSET_MMS * gain
    for i in range(rest, n - int(MAX_LAG / GRID)):
        if sh[i] > ONSET_MMS and sh[i - 1] <= ONSET_MMS and sh[i - rest:i].max() <= ONSET_MMS \
                and sa[i - rest:i].max() <= thr_a and not gap[i - rest:i + 1].any():
            k = np.argmax(sa[i:i + int(MAX_LAG / GRID)] > thr_a)
            if sa[i + k] > thr_a:
                onsets.append(int(k))
    return dict(video=path, span_s=float(g[-1] - g[0]), moving_s=float(len(idx) * GRID),
                lag_ms=int(lag), r2=float(r2), gain=float(gain), window_lags_ms=wins,
                onset_lags_ms=onsets)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('tracks', nargs='+')
    ap.add_argument('--out')
    args = ap.parse_args()
    res = [take(p) for p in args.tracks]
    for r in res:
        w, o = np.array(r['window_lags_ms']), np.array(r['onset_lags_ms'])
        print(f"{r['video']}: arm moving {r['moving_s']:.1f} of {r['span_s']:.1f} s; "
              f"lag {r['lag_ms']} ms (R^2 {r['r2']:.3f}, gain {r['gain']:.2f}); "
              f"windows n={len(w)}" + (f" median {np.median(w):.0f}, IQR {np.percentile(w, 25):.0f}-"
                                       f"{np.percentile(w, 75):.0f}, range {w.min()}-{w.max()} ms" if len(w) else '') +
              f"; onsets n={len(o)}" + (f" median {np.median(o):.0f}, range {o.min()}-{o.max()} ms" if len(o) else ''))
    allw = np.concatenate([r['window_lags_ms'] for r in res]) if res else []
    allo = np.concatenate([r['onset_lags_ms'] for r in res]) if res else []
    if len(allw):
        print(f'ALL windows: n={len(allw)}, median {np.median(allw):.0f} ms, mean {np.mean(allw):.1f} '
              f'+- {np.std(allw):.1f} ms')
    if len(allo):
        print(f'ALL onsets:  n={len(allo)}, median {np.median(allo):.0f} ms, mean {np.mean(allo):.1f} '
              f'+- {np.std(allo):.1f} ms')
    if args.out:
        with open(args.out, 'w') as fh:
            json.dump(res, fh, indent=1)


if __name__ == '__main__':
    main()
