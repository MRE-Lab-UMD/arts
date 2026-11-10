#!/usr/bin/env python3
"""Peg transfer: check one trial's recordings and report the operator's behaviour.

    python3 peg_transfer_analyze.py TRIAL_DIR [--no-video]
    python3 peg_transfer_analyze.py --summary SESSION_DIR      # one table over every trial

TRIAL_DIR is one trial as docs/modules/bench_session_20261109.md lays it out:
  teleop/bag     state bag (run_teleop_real.sh): pedals, both tool arms, the ECM, RCM, marks
  images/        image bag (cameras.launch.py image_bag:=, paused until the record pedal)
  annotation.yaml  the supervisor's counts, checked against the review video (template
                   written on the first run)

The trial is the episode between the record pedal's START and END marks
(/console/episode_mark). Without marks, the span of the recorded images is used, and the
report says so.

Written to TRIAL_DIR/analysis/: report.md, result.json, start.png / end.png (left camera at
START and at END, the view the display shows), contact.png (one frame per 15 s) and
review.mp4 (left camera, 768x512, 30 fps, time since START burned in) for counting the
drops and handoffs. Nothing here is scored: the report lists what the operator did.

Data checks (any failure is printed as FAIL and set in result.json):
  - both image streams have equal frame counts, no gap over 1.5 frame periods (camera_time)
  - the sync box's pulses (/ecm/stereo_sync/pulse, in the state bag) are present and
    their pulse_index never jumps or restarts. A box whose micro-ROS agent is not running
    reboots every ~12 s and stops triggering for ~175 ms each time (firmware main.c; seen in
    E9's images, 2026-11-05): start the cameras with ecm_stereo_sync:=true.
  - every arm's /joint_states and the RCM state have no gap over 20 ms inside the episode
  - the RCM was enabled on both tool arms for the whole episode
Behaviour, inside the episode:
  - time START -> END
  - per tool arm: tip path length (tool_tip_state, low-passed at 5 Hz first: unfiltered
    500 Hz samples add joint-encoder noise to the length), time engaged, engagements (each
    clutch press that moved the arm), RCM deviation median / p95 / max (the controller's
    kinematic definition)
  - camera: camera-control engagements and their total time
  - collision guard holds
RUN rule: source arts_ros2_ws/install only (rosbag2_py, arts_interfaces).
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt

ARMS = ('psm_left', 'psm_right')
ANNOTATION_TEMPLATE = """\
# Filled by the supervisor during the trial, then checked against analysis/review.mp4.
# Definitions: docs/modules/bench_session_20261109.md, "What is counted".
operator: ""           # E1..E3 (expert) or N1..N3 (novice); no names
group: ""              # expert | novice
transferred: null      # triangles placed on a target-side peg, of 6
drops_in_view: null    # dropped before placement, still in the endoscope view
drops_out_of_view: null  # dropped out of the endoscope view (lost; not transferred)
failed_handoffs: null  # handoffs needing more than one attempt, without a drop
time_cap_reached: false  # true if stopped at 300 s
notes: ""
"""


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read(bag, topics):
    """{topic: [(receive_time_s, msg), ...]} for the topics present in the bag."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = sorted(t for t in topics if t in types)
    out = {t: [] for t in want}
    if not want:
        return out, types
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    while r.has_next():
        name, data, t = r.read_next()
        out[name].append((t * 1e-9, deserialize_message(data, cls[name])))
    return out, types


def episodes(marks):
    """[(start_s, end_s, id)] from EpisodeMark START/END pairs (press times)."""
    out, cur = [], None
    for _, m in marks:
        if m.mark_type == 0:
            cur = (stamp(m.header), m.episode_id)
        elif m.mark_type == 1 and cur is not None and m.episode_id == cur[1]:
            out.append((cur[0], stamp(m.header), cur[1]))
            cur = None
    return out


def edges(samples, t0, t1, get):
    """Rising edges and total true time of a latched bool topic inside [t0, t1]."""
    state, since, n, total = False, t0, 0, 0.0
    for t, m in samples:
        v = bool(get(m))
        tt = min(max(t, t0), t1)
        if v and not state:
            since = tt
            if t0 <= t <= t1:
                n += 1
        elif state and not v:
            total += tt - since
        state = v
    if state:
        total += t1 - since
    return n, total


def max_gap(ts):
    return float(np.max(np.diff(ts))) if len(ts) > 1 else float('nan')


def path_length_mm(ts, xyz):
    if len(ts) < 50:
        return float('nan')
    fs = 1.0 / np.median(np.diff(ts))
    b, a = butter(2, 5.0 / (fs / 2.0))
    f = filtfilt(b, a, xyz, axis=0)
    return 1e3 * float(np.sum(np.linalg.norm(np.diff(f, axis=0), axis=1)))


def debayer(msg, scale=0.5):
    import cv2
    raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
    # ROS bayer_rggb8 -> OpenCV's BayerBG code (OpenCV names the pattern from the second row).
    bgr = cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)
    bgr = cv2.rotate(bgr, cv2.ROTATE_180)  # as the display shows it (view rotated 180 deg)
    if scale != 1.0:
        bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return bgr


def images(trial, t0, t1, out_dir, video):
    """Frame checks from the meta topics, then review frames and video from the left camera."""
    import cv2
    bag = os.path.join(trial, 'images')
    meta, _ = read(bag, ['/ecm/stereo_left/meta', '/ecm/stereo_right/meta'])
    res = {}
    for side in ('left', 'right'):
        ms = meta.get(f'/ecm/stereo_{side}/meta', [])
        ct = np.array([m.camera_time for _, m in ms], dtype=np.float64) * 1e-9
        res[side] = {'frames': len(ms),
                     'max_gap_ms': 1e3 * max_gap(ct) if len(ct) > 1 else float('nan'),
                     'first_s': stamp(ms[0][1].header) if ms else float('nan'),
                     'last_s': stamp(ms[-1][1].header) if ms else float('nan')}
    if t0 is None:
        t0, t1 = res['left']['first_s'], res['left']['last_s']
    res['frames_left_in_trial'] = sum(1 for _, m in meta.get('/ecm/stereo_left/meta', [])
                                      if t0 <= stamp(m.header) <= t1)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/ecm/stereo_left/image_raw']))
    Image = get_message('sensor_msgs/msg/Image')
    ff = None
    if video:
        ff = subprocess.Popen(
            ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
             '-s', '768x512', '-r', '30', '-i', '-', '-c:v', 'libx264', '-crf', '23',
             '-pix_fmt', 'yuv420p', os.path.join(out_dir, 'review.mp4')],
            stdin=subprocess.PIPE)
    first = last = None
    sheet, next_sheet = [], 0.0
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        t = stamp(m.header)
        if t < t0 - 0.05 or t > t1 + 0.05:
            continue
        need_sheet = t - t0 >= next_sheet
        if not (video or need_sheet or first is None):
            last = m
            continue
        img = debayer(m)
        cv2.putText(img, f'{t - t0:6.1f} s', (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2, cv2.LINE_AA)
        if first is None:
            first = img
        if need_sheet:
            sheet.append(cv2.resize(img, (384, 256)))
            next_sheet += 15.0
        if ff:
            ff.stdin.write(img.tobytes())
        last = img if video else m
    if ff:
        ff.stdin.close()
        ff.wait()
    if first is not None:
        cv2.imwrite(os.path.join(out_dir, 'start.png'), first)
        cv2.imwrite(os.path.join(out_dir, 'end.png'),
                    last if isinstance(last, np.ndarray) else debayer(last))
    if sheet:
        cols = 4
        while len(sheet) % cols:
            sheet.append(np.zeros_like(sheet[0]))
        rows = [np.hstack(sheet[i:i + cols]) for i in range(0, len(sheet), cols)]
        cv2.imwrite(os.path.join(out_dir, 'contact.png'), np.vstack(rows))
    return res, t0, t1


def analyze(trial, video=True):
    out_dir = os.path.join(trial, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    ann_p = os.path.join(trial, 'annotation.yaml')
    if not os.path.exists(ann_p):
        with open(ann_p, 'w') as f:
            f.write(ANNOTATION_TEMPLATE)
    with open(ann_p) as f:
        ann = yaml.safe_load(f) or {}

    topics = ['/console/episode_mark', '/console/footswitch_teleop', '/ecm/camera_teleop_state',
              '/ecm/stereo_sync/pulse',
              '/ecm/joint_states', '/arts/collision_hold']
    for a in ARMS:
        topics += [f'/{a}/joint_states', f'/{a}/tool_tip_state', f'/{a}/rcm_state',
                   f'/{a}/teleop_state']
    st, _ = read(os.path.join(trial, 'teleop', 'bag'), topics)
    eps = episodes(st.get('/console/episode_mark', []))
    checks, notes = {}, []
    if len(eps) == 1:
        t0, t1 = eps[0][0], eps[0][1]
    elif len(eps) > 1:
        t0, t1 = eps[-1][0], eps[-1][1]
        notes.append(f'{len(eps)} episodes recorded; the LAST one is the trial')
    else:
        t0 = t1 = None
        notes.append('no episode marks in the state bag: trial = span of the recorded images')

    img, t0, t1 = images(trial, t0, t1, out_dir, video)
    checks['image counts equal'] = img['left']['frames'] == img['right']['frames'] > 0
    checks['image gaps <= 1.5 periods'] = all(
        img[s]['max_gap_ms'] <= 1.5 * 1000 / 30 for s in ('left', 'right'))
    # The sync box's own pulse reports (state bag): pulse_index must step by 1. A jump = pulses
    # not reported; a step back = the box rebooted, and it does not trigger while rebooting.
    pulses = [m for t, m in st.get('/ecm/stereo_sync/pulse', []) if t0 <= t <= t1]
    d = np.diff(np.array([m.pulse_index for m in pulses], dtype=np.int64))
    img['pulses'] = {'n': len(pulses), 'index_jumps': int(np.sum(d > 1)),
                     'restarts': int(np.sum(d < 1))}
    checks['sync pulses recorded'] = len(pulses) > 0
    if pulses:
        # A jump can be a best-effort report lost on the way to the recorder (the frames
        # still exist: the camera_time gap check above decides); a step back cannot.
        checks['sync box did not reboot'] = not np.any(d < 1)

    def inside(samples):
        return [(t, m) for t, m in samples if t0 <= t <= t1]

    res = {'trial': os.path.basename(os.path.abspath(trial)), 'start_s': t0, 'end_s': t1,
           'time_s': t1 - t0, 'images': img, 'arms': {}, 'annotation': ann, 'notes': notes}
    for a in ARMS + ('ecm',):
        js = inside(st.get(f'/{a}/joint_states', []))
        g = max_gap([t for t, _ in js]) if js else float('nan')
        checks[f'{a} joint_states gap <= 20 ms'] = bool(js) and g <= 0.020
        if a == 'ecm':
            res['ecm_joint_states_max_gap_ms'] = 1e3 * g
    for a in ARMS:
        tip = inside(st.get(f'/{a}/tool_tip_state', []))
        ts = np.array([t for t, _ in tip])
        xyz = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                        for _, m in tip]).reshape(-1, 3)
        n_eng, t_eng = edges(st.get(f'/{a}/teleop_state', []), t0, t1,
                             lambda m: m.clutch_engaged)
        rcm = inside(st.get(f'/{a}/rcm_state', []))
        dev = np.array([m.deviation for _, m in rcm if m.enabled and np.isfinite(m.deviation)])
        enabled_all = bool(rcm) and all(m.enabled for _, m in rcm)
        checks[f'{a} RCM on throughout'] = enabled_all
        if rcm:
            checks[f'{a} rcm_state gap <= 20 ms'] = max_gap([t for t, _ in rcm]) <= 0.020
        res['arms'][a] = {
            'path_mm': path_length_mm(ts, xyz),
            'engaged_s': t_eng, 'engagements': n_eng,
            'rcm_samples': len(dev),
            'rcm_median_mm': 1e3 * float(np.median(dev)) if len(dev) else float('nan'),
            'rcm_p95_mm': 1e3 * float(np.percentile(dev, 95)) if len(dev) else float('nan'),
            'rcm_max_mm': 1e3 * float(np.max(dev)) if len(dev) else float('nan'),
            'insertion_mm': ([1e3 * min(m.insertion_depth for _, m in rcm),
                              1e3 * max(m.insertion_depth for _, m in rcm)] if rcm else None)}
    n_cam, t_cam = edges(st.get('/ecm/camera_teleop_state', []), t0, t1, lambda m: m.data)
    n_hold, t_hold = edges(st.get('/arts/collision_hold', []), t0, t1, lambda m: m.data)
    res.update(camera_moves=n_cam, camera_s=t_cam, collision_holds=n_hold, collision_hold_s=t_hold,
               collision_topic_recorded='/arts/collision_hold' in st)
    if '/arts/collision_hold' not in st:
        notes.append('/arts/collision_hold not in the state bag: holds not counted')
    res['checks'] = checks
    res['data_ok'] = all(checks.values())

    with open(os.path.join(out_dir, 'result.json'), 'w') as f:
        json.dump(res, f, indent=1, default=float)
    md = report(res)
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(md)
    print(md)
    return res


def fmt(x, d=1):
    return '-' if x is None or (isinstance(x, float) and not np.isfinite(x)) else f'{x:.{d}f}'


def report(r):
    a = r['annotation']
    L = [f"# Peg transfer trial {r['trial']}", '',
         f"Operator {a.get('operator') or '?'} ({a.get('group') or '?'}); "
         f"time **{fmt(r['time_s'])} s** (record pedal START -> END).", '',
         '## Data checks', '']
    L += [f"- {'ok  ' if v else 'FAIL'} {k}" for k, v in r['checks'].items()]
    i = r['images']
    L += ['', f"Images: left {i['left']['frames']}, right {i['right']['frames']} frames; "
          f"max gap {fmt(i['left']['max_gap_ms'])} / {fmt(i['right']['max_gap_ms'])} ms. "
          f"Sync pulses in the trial {i['pulses']['n']} (index jumps {i['pulses']['index_jumps']}, "
          f"restarts {i['pulses']['restarts']}) for {i['frames_left_in_trial']} "
          f"left frames.", '',
          '## Behaviour', '',
          '| | psm_left | psm_right |', '|---|---|---|']
    rows = [('tip path (mm)', 'path_mm', 0), ('engaged (s)', 'engaged_s', 1),
            ('engagements', 'engagements', 0), ('RCM median (mm)', 'rcm_median_mm', 3),
            ('RCM p95 (mm)', 'rcm_p95_mm', 3), ('RCM max (mm)', 'rcm_max_mm', 3)]
    for label, k, d in rows:
        v = [r['arms'][x][k] for x in ARMS]
        L.append(f'| {label} | ' + ' | '.join(str(x) if d == 0 and isinstance(x, int)
                                               else fmt(x, d) for x in v) + ' |')
    L += ['', f"Camera moves {r['camera_moves']} ({fmt(r['camera_s'])} s); collision holds "
          f"{r['collision_holds']} ({fmt(r['collision_hold_s'])} s).", '',
          '## Supervisor counts (annotation.yaml)', '']
    for k in ('transferred', 'drops_in_view', 'drops_out_of_view', 'failed_handoffs',
              'time_cap_reached', 'notes'):
        L.append(f'- {k}: {a.get(k)}')
    if r['notes']:
        L += ['', '## Notes', ''] + [f'- {n}' for n in r['notes']]
    L += ['', f"Data OK: **{'yes' if r['data_ok'] else 'NO'}**", '']
    return '\n'.join(L)


def summary(session):
    rs = []
    for p in sorted(glob.glob(os.path.join(session, '*', 'analysis', 'result.json'))):
        with open(p) as f:
            r = json.load(f)
        # The counts may have been corrected after the analysis ran: read them fresh.
        ann_p = os.path.join(os.path.dirname(os.path.dirname(p)), 'annotation.yaml')
        if os.path.exists(ann_p):
            with open(ann_p) as f:
                r['annotation'] = yaml.safe_load(f) or {}
        rs.append(r)
    L = ['| trial | op | group | time (s) | moved /6 | drops in / out | failed handoffs | '
         'path L / R (mm) | engagements L / R | camera moves | RCM median L / R (mm) | data |',
         '|' + '---|' * 12]
    for r in rs:
        a, al, ar = r['annotation'], r['arms']['psm_left'], r['arms']['psm_right']
        L.append(f"| {r['trial']} | {a.get('operator')} | {a.get('group')} | {fmt(r['time_s'])} | "
                 f"{a.get('transferred')} | {a.get('drops_in_view')} / {a.get('drops_out_of_view')} | "
                 f"{a.get('failed_handoffs')} | {fmt(al['path_mm'], 0)} / {fmt(ar['path_mm'], 0)} | "
                 f"{al['engagements']} / {ar['engagements']} | {r['camera_moves']} | "
                 f"{fmt(al['rcm_median_mm'], 3)} / {fmt(ar['rcm_median_mm'], 3)} | "
                 f"{'ok' if r['data_ok'] else 'FAIL'} |")
    print('\n'.join(L))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('path')
    ap.add_argument('--summary', action='store_true')
    ap.add_argument('--no-video', action='store_true')
    args = ap.parse_args()
    if args.summary:
        summary(args.path)
        return 0
    return 0 if analyze(args.path, video=not args.no_video)['data_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
