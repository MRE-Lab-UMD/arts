#!/usr/bin/env python3
"""Offline analyzer for an ARTS camera-sync recording (FLIR Stage 5e).

Reads a rosbag2 MCAP recording of

    /ecm/stereo_sync/pulse         arts_interfaces/CameraSyncPulse   (the ESP32 sync box; named
                                   /arts_camera_sync/pulse before decision 18, 2026-11-01)
    /<cam>/meta                    flir_camera_msgs/ImageMetaData    (one per camera)

and reports the stereo sync metric TWO INDEPENDENT WAYS, which is the whole point of the
exercise and the Stage 5e acceptance criterion:

  (a) HARDWARE — the difference of two ExposureActive capture timestamps taken on one 80 MHz
      MCPWM counter. Nothing in ROS, Spinnaker, DDS or any clock estimator can influence it.
  (b) SOFTWARE CROSS-CHECK — the per-frame difference of the two cameras' own `camera_time`
      device clocks, from the meta topics.

They measure the same physical quantity through completely disjoint paths, so agreement is
evidence and disagreement localizes the fault. Note what is and is not comparable: the two
camera clocks have unrelated epochs and drifting crystals, so their ABSOLUTE difference is
meaningless and only its SPREAD (and drift slope) can be set against the hardware figure.

Deliberately NOT read: `header.stamp`. Under the synchronized driver it is a Kalman-smoothed
estimate that is identical for every camera of one trigger by construction, so a skew computed
from it measures nothing (AGENTS.md decision 5 amendment).

Usage (with the FLIR overlay sourced, so flir_camera_msgs resolves):
  python3 analyze_sync.py /path/to/bag_dir --json out.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

NO_CAPTURE = 0xFFFFFFFF
TICK_WRAP = 1 << 32


def _mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


def _stdev(xs):
    if len(xs) < 2:
        return 0.0
    mu = _mean(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / (len(xs) - 1))


def _summary(xs, unit):
    if not xs:
        return {'n': 0, 'unit': unit}
    return {
        'n': len(xs),
        'unit': unit,
        'mean': _mean(xs),
        'std': _stdev(xs),
        'min': min(xs),
        'max': max(xs),
        'spread': max(xs) - min(xs),
    }


def _fmt(label, s):
    if s.get('n', 0) == 0:
        return '  %-28s (no samples)' % label
    return '  %-28s n=%-6d mean %11.3f   std %8.3f   min %11.3f   max %11.3f   spread %9.3f  %s' % (
        label, s['n'], s['mean'], s['std'], s['min'], s['max'], s['spread'], s['unit'])


def _unwrap(values, modulus):
    """Undo wraparound of a monotonically increasing counter.

    The MCPWM capture counter is 32-bit at 80 MHz, so it wraps every 53.7 s — shorter than any
    useful recording. Left wrapped, a single wrap turns the clock fit below into nonsense while
    every per-pulse difference stays correct, so this is easy to overlook.
    """
    out = []
    offset = 0
    prev = None
    for v in values:
        if prev is not None and v < prev - modulus // 2:
            offset += modulus
        out.append(v + offset)
        prev = v
    return out


def _linfit(xs, ys):
    """Least-squares y = a*x + b; returns (a, b, residual_std)."""
    n = len(xs)
    if n < 3:
        return float('nan'), float('nan'), float('nan')
    mx, my = _mean(xs), _mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0.0:
        return float('nan'), float('nan'), float('nan')
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    resid = [y - (a * x + b) for x, y in zip(xs, ys)]
    return a, b, _stdev(resid)


# Local detrend window for the camera_time delta, in frames (~10 s at 30 Hz). The two camera
# crystals' ratio is not constant over a long recording: in the 30 min E4 bag (2026-10-28) a
# single linear fit left per-minute means spanning 9.1 us, which dropped the per-pulse join to
# r = 0.73; detrended with this moving average it was r = 0.99, residual 0.42 us, as in the
# short runs. Pulse-to-pulse skew varies far faster than 10 s, so the window removes only the
# clock wander.
LOCAL_DETREND_FRAMES = 301


def _moving_average_detrend(ys, window):
    """ys minus its centred moving average (window shrinks at the ends)."""
    n, h = len(ys), window // 2
    pre = [0.0]
    for y in ys:
        pre.append(pre[-1] + y)
    out = []
    for i in range(n):
        lo, hi = max(0, i - h), min(n, i + h + 1)
        out.append(ys[i] - (pre[hi] - pre[lo]) / (hi - lo))
    return out


def read_bag(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    pulses = []
    meta = {}
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        type_name = types.get(topic)
        if type_name is None:
            continue
        msg = deserialize_message(data, get_message(type_name))
        if type_name.endswith('CameraSyncPulse'):
            pulses.append((recv_ns, msg))
        elif type_name.endswith('ImageMetaData'):
            meta.setdefault(topic, []).append((recv_ns, msg))
    return pulses, meta


def analyze(bag_path, cam0_meta=None):
    pulses, meta = read_bag(bag_path)
    if not pulses:
        raise SystemExit('no CameraSyncPulse messages in %s' % bag_path)

    tick_ns = pulses[0][1].tick_period_ns
    ncam = pulses[0][1].camera_count
    result = {
        'bag': str(bag_path),
        'tick_period_ns': tick_ns,
        'camera_count': int(ncam),
        'pulses': len(pulses),
    }

    # --- pulse-index continuity. A gap means a record was dropped between the capture ISR and
    # the publisher, which is the one failure that would bias a skew average without showing up
    # as a bad number anywhere else.
    indices = [m.pulse_index for _, m in pulses]
    expected = indices[-1] - indices[0] + 1
    result['pulse_index_span'] = int(expected)
    result['pulse_index_gaps'] = int(expected - len(indices))
    result['time_synced_all'] = all(m.time_synced for _, m in pulses)

    period_us = [m.trigger_period_ticks * tick_ns / 1000.0
                 for _, m in pulses if m.trigger_period_ticks != 0]
    result['trigger_period_us'] = _summary(period_us, 'us')

    # --- (a) hardware: absolute trigger->exposure per camera, exposure width, and the skew.
    delay = {i: [] for i in range(ncam)}
    width = {i: [] for i in range(ncam)}
    missing = {i: 0 for i in range(ncam)}
    skew = []
    skew_by_pulse = []  # same as skew, but one entry per pulse (None if a strobe is missing)
    for _, m in pulses:
        for i in range(ncam):
            if m.exposure_delay_ticks[i] == NO_CAPTURE:
                missing[i] += 1
            else:
                delay[i].append(m.exposure_delay_ticks[i] * tick_ns / 1000.0)
            if m.exposure_width_ticks[i] != NO_CAPTURE:
                width[i].append(m.exposure_width_ticks[i] * tick_ns / 1000.0)
        if ncam >= 2 and NO_CAPTURE not in (m.exposure_delay_ticks[0], m.exposure_delay_ticks[1]):
            # SIGNED, unlike the firmware's selftest print. The sign is the systematic term —
            # which camera leads — and averaging absolute values would destroy it while
            # inflating the mean by the random spread.
            #
            # int() IS LOAD-BEARING: the field is uint32 and deserializes as a numpy uint32, so
            # subtracting the larger from the smaller wraps to ~2^32 ticks instead of going
            # negative. That is 53.687 s, which is obvious once seen — but the mean of a mixture
            # of correct microseconds and wrapped values is merely a large wrong number.
            skew.append((int(m.exposure_delay_ticks[0]) - int(m.exposure_delay_ticks[1]))
                        * tick_ns / 1000.0)
            skew_by_pulse.append(skew[-1])
        else:
            skew_by_pulse.append(None)

    result['hardware'] = {
        'trigger_to_exposure_us': {str(i): _summary(delay[i], 'us') for i in range(ncam)},
        'exposure_width_us': {str(i): _summary(width[i], 'us') for i in range(ncam)},
        'strobes_missing': {str(i): missing[i] for i in range(ncam)},
        'skew_signed_us': _summary(skew, 'us'),
        'skew_abs_max_us': max((abs(s) for s in skew), default=float('nan')),
    }

    # --- capture-tick to ROS-time fit (decision 15, hop 3). The trigger is periodic, so this
    # recovers the ESP32-crystal-to-host-clock RATIO as well as the offset; the residual is the
    # transport jitter of the pulse messages and is the alignment uncertainty to report.
    # The x axis is the hardware tick count and the y axis is the bag's receive time, so the
    # residual includes micro-ROS serial transport and agent scheduling — it is an upper bound.
    ticks = _unwrap([m.trigger_ticks for _, m in pulses], TICK_WRAP)
    recv_s = [r * 1e-9 for r, _ in pulses]
    a, _b, resid = _linfit([t * tick_ns * 1e-9 for t in ticks], recv_s)
    result['tick_to_ros_fit'] = {
        'slope': a,
        'slope_ppm_error': (a - 1.0) * 1e6 if a == a else float('nan'),
        'residual_std_us': resid * 1e6 if resid == resid else float('nan'),
    }

    # --- (b) software cross-check from the two device clocks.
    # names[i] must be the meta topic of capture channel cam_i, or the software columns pair
    # with the wrong hardware channel. Sorted order happened to be right until 2026-10-28, when
    # the roles were corrected and cam0 (J1, 24057132) became /camera_right.
    names = sorted(meta)
    if cam0_meta:
        if cam0_meta not in meta:
            raise SystemExit('--cam0-meta %s not in bag (have %s)' % (cam0_meta, names))
        names = [cam0_meta] + [n for n in names if n != cam0_meta]
    result['meta_topics'] = {n: len(meta[n]) for n in names}

    # Frame continuity per camera, from its own clock. This is what classifies a missing
    # strobe: with no gap here the camera exposed on every pulse, so a missing capture is an
    # edge the box did not register, not a frame the camera did not take (found 2026-10-28).
    period_ns = (_mean(period_us) if period_us else 33333.0) * 1000.0
    result['camera_time_frame_gaps'] = {
        n: sum(1 for (_, a), (_, b) in zip(meta[n], meta[n][1:])
               if int(b.camera_time) - int(a.camera_time) > 1.5 * period_ns)
        for n in names}

    if len(names) >= 2:
        left, right = meta[names[0]], meta[names[1]]
        # Pair by receive time. No frame counter exists in ImageMetaData, so proximity is the
        # only join available; a pair is accepted only well inside one frame period, which at
        # 30 Hz makes a mis-pair impossible rather than merely unlikely.
        tol_ns = 0.4 * (_mean(period_us) * 1000.0 if period_us else 33333000.0)
        j = 0
        deltas, times, exp_l, exp_r = [], [], [], []
        for rl, ml in left:
            while j + 1 < len(right) and abs(right[j + 1][0] - rl) < abs(right[j][0] - rl):
                j += 1
            rr, mr = right[j]
            if abs(rr - rl) > tol_ns:
                continue
            deltas.append((int(ml.camera_time) - int(mr.camera_time)) * 1e-3)  # ns -> us
            times.append(rl * 1e-9)
            exp_l.append(float(ml.exposure_time))
            exp_r.append(float(mr.exposure_time))

        drift, _off, drift_resid = _linfit(times, deltas)
        result['software'] = {
            'pairs': len(deltas),
            'camera_time_delta_us': _summary(deltas, 'us'),
            # The two crystals differ, so the raw delta ramps. Slope is that ratio error; the
            # residual about the ramp is the number comparable to the hardware skew spread.
            'drift_us_per_s': drift,
            'detrended_residual_std_us': drift_resid,
            'local_detrended_residual_std_us': _stdev(
                _moving_average_detrend(deltas, LOCAL_DETREND_FRAMES)),
            'reported_exposure_time_us': {
                names[0]: _summary(exp_l, 'us'),
                names[1]: _summary(exp_r, 'us'),
            },
        }

        # Self-calibration of the strobe rise delay (AGENTS.md rung 5d): the camera reports its
        # true ExposureTime in chunk data, so ExposureTime - measured_width is the rise delay,
        # per camera, in band. It is common-mode and cancels out of the skew, but it is the bias
        # inside every ABSOLUTE trigger-to-exposure figure.
        rise = {}
        for idx, nm in enumerate(names[:ncam]):
            exp = exp_l if idx == 0 else exp_r
            if exp and width[idx]:
                rise[nm] = _mean(exp) - _mean(width[idx])
        result['software']['implied_strobe_rise_delay_us'] = rise

        # Per-pulse join: the detrended camera_time delta of frame k+lag against the hardware
        # skew of pulse k. Valid only when every left frame paired and no camera dropped a
        # frame, because then pulse order and frame order are both gap-free and differ by a
        # constant lag, found here by maximum correlation. This compares the two methods pulse
        # by pulse, signed, rather than only their spreads. The systematic term does not survive
        # (the camera clocks have unrelated epochs), so hardware is centred before differencing.
        if (len(deltas) == len(left)
                and not any(result['camera_time_frame_gaps'].values())):
            detr = _moving_average_detrend(deltas, LOCAL_DETREND_FRAMES)
            hw_mu = _mean(skew)
            best = None
            for lag in range(-10, 11):
                xs, ys = [], []
                for k, s in enumerate(skew_by_pulse):
                    if s is not None and 0 <= k + lag < len(detr):
                        xs.append(s - hw_mu)
                        ys.append(detr[k + lag])
                sx, sy = _stdev(xs), _stdev(ys)
                if len(xs) < 3 or sx == 0.0 or sy == 0.0:
                    continue
                r = sum(x * y for x, y in zip(xs, ys)) / ((len(xs) - 1) * sx * sy)
                if best is None or r > best[1]:
                    best = (lag, r, [y - x for x, y in zip(xs, ys)])
            if best is not None:
                result['software']['per_pulse_join'] = {
                    'lag_frames': best[0],
                    'correlation': best[1],
                    'residual_us': _summary(best[2], 'us'),
                }

    return result


def report(r):
    out = []
    out.append('ARTS camera sync analysis — %s' % r['bag'])
    out.append('  pulses %d   index span %d   gaps %d   tick %.3f ns   time_synced %s'
               % (r['pulses'], r['pulse_index_span'], r['pulse_index_gaps'],
                  r['tick_period_ns'], r['time_synced_all']))
    out.append('')
    out.append('TRIGGER')
    out.append(_fmt('period', r['trigger_period_us']))
    out.append('')
    out.append('(a) HARDWARE — one 80 MHz counter, ExposureActive capture')
    hw = r['hardware']
    for i in sorted(hw['trigger_to_exposure_us']):
        out.append(_fmt('cam%s trigger->exposure' % i, hw['trigger_to_exposure_us'][i]))
        out.append(_fmt('cam%s exposure width' % i, hw['exposure_width_us'][i]))
        out.append('  %-28s %d' % ('cam%s strobes missing' % i, hw['strobes_missing'][i]))
    out.append(_fmt('SKEW (cam0 - cam1, signed)', hw['skew_signed_us']))
    out.append('  %-28s %.3f us' % ('SKEW worst case |.|', hw['skew_abs_max_us']))
    out.append('')
    fit = r['tick_to_ros_fit']
    out.append('CAPTURE TICK -> ROS TIME FIT (decision 15 hop 3)')
    out.append('  %-28s %.3f ppm' % ('crystal ratio error', fit['slope_ppm_error']))
    out.append('  %-28s %.1f us  (includes micro-ROS transport jitter)'
               % ('residual std', fit['residual_std_us']))
    out.append('')
    out.append('(b) SOFTWARE CROSS-CHECK — camera_time device clocks')
    if 'software' not in r:
        out.append('  (needs two meta topics in the bag; found %s)' % r.get('meta_topics'))
    else:
        sw = r['software']
        out.append('  %-28s %d' % ('paired frames', sw['pairs']))
        out.append(_fmt('camera_time delta (raw)', sw['camera_time_delta_us']))
        out.append('  %-28s %.3f us/s   (independent crystals; absolute value is arbitrary)'
                   % ('delta drift', sw['drift_us_per_s']))
        out.append('  %-28s %.3f us   (linear; includes slow clock-ratio wander)'
                   % ('detrended residual std', sw['detrended_residual_std_us']))
        out.append('  %-28s %.3f us   <-- compare against the hardware skew spread'
                   % ('local detrend (%d fr) std' % LOCAL_DETREND_FRAMES,
                      sw['local_detrended_residual_std_us']))
        for nm, v in sw.get('implied_strobe_rise_delay_us', {}).items():
            out.append('  %-28s %.3f us' % ('rise delay %s' % nm, v))
        join = sw.get('per_pulse_join')
        if join is None:
            out.append('  per-pulse join               (skipped: unpaired frames or a frame gap)')
        else:
            out.append('  %-28s lag %+d frames   r = %.4f'
                       % ('per-pulse join', join['lag_frames'], join['correlation']))
            out.append(_fmt('  residual (sw - hw)', join['residual_us']))
            if join['correlation'] < 0.9:
                # Measured 2026-10-28: correct pairing r = 0.992; meta topics paired with the
                # wrong capture channels r = 0.45 at a spurious lag.
                out.append('  WARNING: join correlation below 0.9 — check --cam0-meta names the '
                           'camera on terminal J1')
    for nm, g in r.get('camera_time_frame_gaps', {}).items():
        out.append('  %-28s %d   (0 => every missing strobe was a lost edge, not a lost frame)'
                   % ('frame gaps %s' % nm, g))
    return '\n'.join(out)


def main():
    p = argparse.ArgumentParser(description='ARTS camera sync bag analyzer')
    p.add_argument('bag', type=Path)
    p.add_argument('--json', type=Path, help='also write the raw numbers here')
    p.add_argument('--cam0-meta', help='meta topic of the camera on sync-box channel cam0 '
                                       '(terminal J1); default: alphabetically first')
    args = p.parse_args()

    r = analyze(args.bag, args.cam0_meta)
    print(report(r))
    if args.json:
        args.json.write_text(json.dumps(r, indent=2) + '\n')
        print('\nwrote %s' % args.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
