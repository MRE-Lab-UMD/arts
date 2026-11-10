#!/usr/bin/env python3
"""Decode an E7 latency-clock bag: glass-to-glass latency per frame.

Input: a bag of /ecm/stereo_left/image_raw (/camera_left/image_raw before decision 18) recorded with latency_clock.py and the tower view side
by side in the field (see latency_clock.py). Nothing is placed by hand:

  1. frames are reduced to 4x4-block grey; the LIVE squares are the pixels that change most
     over time (temporal std), split into 8 connected blobs;
  2. each blob is thresholded at the midpoint of its own range, and its bit index comes from
     how often it toggles (Gray bit j toggles every 2^(j+1) units), so the camera's
     orientation does not matter. The top two bits BOTH toggle every 2^(bits-1) units, so they are
     told apart by the frame stamps: the clock follows wall time and so do the stamps, so the
     live code must be floor(stamp / unit + c) for one constant c (2026-10-28: ordering by
     toggles alone swapped them);
  3. the COPY squares are too small and dim for a threshold (2026-10-28: ~12 px, partly
     vignetted). Each is found as the pixels whose time series best matches one live bit at
     a delay of 1-8 frames: first anywhere outside the live clock, then, for every bit,
     inside a box around the bits that matched clearly. The copy bit is then read from those
     pixels' own series (midpoint threshold), not from the live one;
  4. per frame, latency = ((k_live - k_copy) mod 2^bits) x unit. A frame is excluded when the
     live code is more than one unit from the stamp prediction, or the copy code steps from
     either neighbour by anything but 0-4 units (a misread bit).

A single frame is quantized to one unit; the mean is unbiased because the display and the
cameras are not synchronized (latency_clock.py). Frames where either code jumps implausibly
(a misread bit) are counted and excluded, never silently kept.

  analyze_latency_clock.py BAG [--bits 4] [--unit-ms 20] [--topic /ecm/stereo_left/image_raw] [--json OUT]
"""
import argparse
import json
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy import ndimage
from sensor_msgs.msg import Image

BLOCK = 4


def load(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    frames, stamps = [], []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        h, w = (m.height // BLOCK) * BLOCK, (m.width // BLOCK) * BLOCK
        frames.append(a[:h, :w].reshape(h // BLOCK, BLOCK, w // BLOCK, BLOCK)
                      .mean(axis=(1, 3)).astype(np.float32))
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return np.stack(frames), np.array(stamps)


def gray_decode(g):
    k = g.copy()
    shift = g >> 1
    while shift.any():
        k ^= shift
        shift >>= 1
    return k


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/ecm/stereo_left/image_raw',
                    help='/camera_left/image_raw for bags before decision 18 (E7, 2026-10-28)')
    ap.add_argument('--bits', type=int, default=4)
    ap.add_argument('--unit-ms', type=float, default=20.0)
    ap.add_argument('--min-area', type=int, default=6, help='blob area, 4x4 blocks')
    ap.add_argument('--json')
    args = ap.parse_args()
    BITS = args.bits

    frames, stamps = load(args.bag, args.topic)
    n = len(frames)
    std = frames.std(axis=0)
    # Threshold: the highest that yields exactly BITS blobs. A fixed mean + 3 sigma lost the
    # dimmest square once the squares filled a large part of the frame (2026-10-28). Eroded
    # before labelling so squares near the vignetted edge do not merge.
    for k in np.arange(4.0, 0.99, -0.25):
        thr = std.mean() + k * std.std()
        changing = std > thr
        labels, nlab = ndimage.label(ndimage.binary_erosion(changing, iterations=2))
        sizes = ndimage.sum(np.ones_like(std), labels, range(1, nlab + 1))
        if int((sizes >= args.min_area).sum()) == BITS:
            break
    blobs = []
    for i in range(1, nlab + 1):
        mask = labels == i
        area = int(mask.sum())
        if area < args.min_area:
            continue
        cy, cx = ndimage.center_of_mass(mask)
        series = frames[:, mask].mean(axis=1)
        lo, hi = np.percentile(series, [5, 95])
        bits = (series > (lo + hi) / 2).astype(np.int64)
        blobs.append(dict(cx=cx * BLOCK, cy=cy * BLOCK, area=area, contrast=float(hi - lo),
                          bits=bits, toggles=int(np.abs(np.diff(bits)).sum()), mask=mask))
    print('frames %d, changing blobs %d (threshold std %.1f)' % (n, len(blobs), thr))
    for b in sorted(blobs, key=lambda b: b['cx']):
        print('  x %6.0f y %6.0f area %4d contrast %5.1f toggles %5d'
              % (b['cx'], b['cy'], b['area'], b['contrast'], b['toggles']))

    live = sorted(blobs, key=lambda b: -b['toggles'])  # most toggles = Gray bit 0
    if len(live) != BITS:
        sys.exit('found %d changing blobs, need the %d live squares' % (len(live), BITS))

    def stamp_fit(k):
        best = None
        for c in np.arange(-10.0, 10.0, 0.05):
            kp = np.floor(stamps * 1e3 / args.unit_ms + c).astype(np.int64) % (1 << BITS)
            agree = float(np.mean(kp == k))
            if best is None or agree > best[0]:
                best = (agree, c, kp)
        return best

    def live_code(order):
        return gray_decode(sum(b['bits'] << j for j, b in enumerate(order)))

    swapped = live[:BITS - 2] + [live[BITS - 1], live[BITS - 2]]
    if stamp_fit(live_code(swapped))[0] > stamp_fit(live_code(live))[0]:
        live = swapped
    # Exclude the live squares only: at the lower thresholds the copy changes enough to be
    # in `changing` too.
    excl = ndimage.binary_dilation(np.any([b['mask'] for b in live], axis=0), iterations=8)
    H, W = std.shape
    fz = frames - frames.mean(axis=0)
    fz /= fz.std(axis=0) + 1e-6
    X = fz.reshape(n, -1)

    def search(b, allowed, top=6):
        bz = (b - b.mean()) / (b.std() + 1e-9)
        best = None
        # Lag >= 1: the display cannot show a frame before it is captured, and lag 0 only
        # finds the halo of the live squares (2026-10-28).
        for lag in range(1, 9):
            corr = (X[lag:].T @ bz[:n - lag]) / (n - lag)
            corr[~allowed.ravel()] = -1
            idx = np.argsort(corr)[-top:]
            val = float(corr[idx].mean())
            if best is None or val > best[0]:
                ys, xs = np.unravel_index(idx, (H, W))
                best = (val, lag, idx, ys.mean(), xs.mean(), ys.std() + xs.std())
        return best

    first = [search(b['bits'].astype(float), ~excl) for b in live]
    for j, r in enumerate(first):
        print('  first pass bit %d: corr %.2f lag %d at (%.0f,%.0f) spread %.1f'
              % (j, r[0], r[1], r[4] * BLOCK, r[3] * BLOCK, r[5]))
    # A bit can also match its own live square one period later (bit 1 repeats every 8 units
    # = 160 ms, about 5 frames), so the copy is located from the bits that agree, and every
    # bit is then searched only inside that box.
    clear = [r for r in first if r[0] >= 0.6 and r[5] < 6]
    if len(clear) < 2:
        sys.exit('copy not found: only %d bits matched clearly' % len(clear))
    cy, cx = np.median([r[3] for r in clear]), np.median([r[4] for r in clear])
    box = np.zeros((H, W), bool)
    box[max(0, int(cy) - 30):int(cy) + 30, max(0, int(cx) - 40):int(cx) + 40] = True
    box &= ~excl
    copy_bits = []
    for j, b in enumerate(live):
        val, lag, idx, ys, xs, spread = search(b['bits'].astype(float), box)
        series = X[:, idx].mean(axis=1)
        lo, hi = np.percentile(series, [5, 95])
        copy_bits.append((series > (lo + hi) / 2).astype(np.int64))
        print('  bit %d: live (%4.0f,%4.0f) toggles %4d | copy (%4.0f,%4.0f) corr %.2f lag %d'
              % (j, b['cx'], b['cy'], b['toggles'], xs * BLOCK, ys * BLOCK, val, lag))
    codes = {'live': live_code(live),
             'copy': gray_decode(sum(c << j for j, c in enumerate(copy_bits)))}
    live_rows = np.mean([b['cy'] for b in live])
    copy_rows = cy * BLOCK

    # Diagnostic only: the clock follows wall time, and so do the frame stamps, so the live
    # code should be floor(stamp / unit + c) for one constant c. Per-bit disagreement with
    # that prediction shows which squares are misread.
    for name, bitsets in (('live', [b['bits'] for b in live]), ('copy', copy_bits)):
        best = stamp_fit(codes[name])
        if name == 'live':
            k_pred = best[2]
        gp = best[2] ^ (best[2] >> 1)
        print('%s: %.1f %% of frames equal the stamp prediction (offset %.2f units); '
              'bit error %%: %s' % (name, 100 * best[0], best[1],
                                    ['%.1f' % (100 * np.mean(((gp >> j) & 1) != bb))
                                     for j, bb in enumerate(bitsets)]))

    # Exclusions (see the module docstring). Neighbouring copy codes differ by one camera
    # period (1.67 units) plus one unit of read ambiguity either side: 0-4 is plausible.
    m = 1 << BITS
    dev = (codes['live'] - k_pred) % m
    ok = (dev <= 1) | (dev >= m - 1)
    step = np.diff(codes['copy']) % m
    bad = step > 4
    ok[1:] &= ~bad
    ok[:-1] &= ~bad
    diff = (codes['live'] - codes['copy']) % (1 << BITS)
    lat = diff[ok] * args.unit_ms
    res = dict(frames=n, used=int(ok.sum()), excluded=int(n - ok.sum()),
               unit_ms=args.unit_ms, mean_ms=float(lat.mean()), std_ms=float(lat.std()),
               p5_ms=float(np.percentile(lat, 5)), median_ms=float(np.median(lat)),
               p95_ms=float(np.percentile(lat, 95)), min_ms=float(lat.min()),
               max_ms=float(lat.max()),
               sem_ms=float(lat.std() / np.sqrt(len(lat))),
               live_minus_copy_rows=float(live_rows - copy_rows),
               histogram={int(k): int(v) for k, v in zip(*np.unique(diff[ok], return_counts=True))})
    print('used %d of %d frames (%d excluded as misreads)' % (res['used'], n, res['excluded']))
    print('GLASS-TO-GLASS  mean %.1f ms  (sem %.1f)  std %.1f  median %.0f  p5 %.0f  p95 %.0f  '
          'min %.0f  max %.0f' % (res['mean_ms'], res['sem_ms'], res['std_ms'], res['median_ms'],
                                  res['p5_ms'], res['p95_ms'], res['min_ms'], res['max_ms']))
    print('histogram (units of %.0f ms): %s' % (args.unit_ms, res['histogram']))
    print('row offset live - copy: %.0f image rows (rolling-shutter bias)' %
          res['live_minus_copy_rows'])
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
