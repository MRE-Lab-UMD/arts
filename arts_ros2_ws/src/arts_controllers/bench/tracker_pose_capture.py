#!/usr/bin/env python3
"""Capture a tracked marker's pose on a key press, once the marker is completely static.

Platform-neutral: it reads only tracker topics (the fusionTrack's /atracsys/<tool>/measured_cp,
geometry_msgs/PoseStamped), so the ARTS arm and the dVRK-Si are measured by the same code.
Run it in the tracker's shell (source ~/atracsys_ws/install/setup.bash), not the ARTS one:

    python3 tracker_pose_capture.py OUT.jsonl --topic /atracsys/<tool>/measured_cp
                                    [--reference-topic /atracsys/<ref>/measured_cp]

Keys: SPACE capture, u undo the last capture, q quit. A capture waits until the last
`window` s of samples are static (position std <= static_std_mm on every axis, and the means of
the window's two halves within static_drift_mm), then stores the window MEAN and its std. The
first capture is labelled `reference` (the pose jogged to and logged; not evaluated), every later
one `return N`. With --reference-topic, every pose is the tool's pose in the reference marker's
frame (a bumped tripod then does not move the data).

After each return it prints the return's distance from the barycentre of all returns so far
and the running ISO 9283 RP (mean l + 3 S_l) with RP_a,b,c (+-3 S of the rotation about each
axis), the numbers the analyzer computes for one pose. One JSON object per line in OUT.
"""
import argparse
import json
import select
import sys
import termios
import time
import tty
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation


def pose_mat(m):
    p, q = m.pose.position, m.pose.orientation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return t


class Capture(Node):

    def __init__(self, args):
        super().__init__('tracker_pose_capture')
        self.args = args
        self.buf = deque()        # (receive time, 4x4 pose)
        self.ref = None
        self.ref_time = 0.0
        self.create_subscription(PoseStamped, args.topic, self._on_tool, 100)
        if args.reference_topic:
            self.create_subscription(PoseStamped, args.reference_topic, self._on_ref, 100)

    def _on_ref(self, m):
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return    # the tracker node keeps publishing the LAST pose with a zero stamp while the marker is not seen (2026-11-07)
        self.ref, self.ref_time = pose_mat(m), time.monotonic()

    def _on_tool(self, m):
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return    # not tracked: zero stamp (see _on_ref)
        now = time.monotonic()
        t = pose_mat(m)
        if self.args.reference_topic:
            if self.ref is None or now - self.ref_time > 0.1:
                return                              # the reference is not in view
            t = np.linalg.inv(self.ref) @ t
        self.buf.append((now, t))
        while self.buf and now - self.buf[0][0] > self.args.window:
            self.buf.popleft()

    def static(self):
        """(ok, mean 4x4, std mm xyz, why) over the current window."""
        a = self.args
        if not self.buf or time.monotonic() - self.buf[-1][0] > 0.1:
            return False, None, None, 'marker not visible'
        ts = np.array([b[0] for b in self.buf])
        if ts[-1] - ts[0] < 0.9 * a.window or len(ts) < 10:
            return False, None, None, 'filling the window'
        pos = np.array([b[1][:3, 3] for b in self.buf])
        std = 1e3 * pos.std(axis=0)
        half = len(pos) // 2
        drift = 1e3 * np.linalg.norm(pos[:half].mean(axis=0) - pos[half:].mean(axis=0))
        rots = Rotation.from_matrix(np.array([b[1][:3, :3] for b in self.buf]))
        mean = np.eye(4)
        mean[:3, :3] = rots.mean().as_matrix()
        mean[:3, 3] = pos.mean(axis=0)
        if std.max() > a.static_std_mm or drift > a.static_drift_mm:
            return False, mean, std, f'moving: std {std.max():.4f} mm, drift {drift:.4f} mm'
        return True, mean, std, f'{len(pos)} samples, std {std.max():.4f} mm, drift {drift:.4f} mm'


def rp(returns):
    x = np.array([r['position'] for r in returns])
    l = np.linalg.norm(x - x.mean(axis=0), axis=1)
    rots = Rotation.from_quat([r['quaternion'] for r in returns])
    rel = (rots.mean().inv() * rots).as_rotvec()
    return (1e3 * (l.mean() + 3 * l.std(ddof=1)), 1e3 * l,
            3 * np.degrees(rel.std(axis=0, ddof=1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out')
    ap.add_argument('--topic', required=True)
    ap.add_argument('--reference-topic')
    ap.add_argument('--window', type=float, default=1.0, help='s of static samples averaged')
    ap.add_argument('--static-std-mm', type=float, default=0.03)
    ap.add_argument('--static-drift-mm', type=float, default=0.02)
    ap.add_argument('--timeout', type=float, default=30.0, help='s to wait for stillness')
    args = ap.parse_args()

    records = []
    try:
        with open(args.out) as f:
            records = [json.loads(s) for s in f if s.strip()]
        print(f'{args.out}: continuing after {len(records)} captures')
    except FileNotFoundError:
        pass

    rclpy.init()
    n = Capture(args)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print('SPACE capture   u undo   q quit')
    last_status = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.005)
            if time.monotonic() - last_status > 1.0:
                ok, _, _, why = n.static()
                print(f'\r  [{"STATIC" if ok else "      "}] {why:60s}', end='', flush=True)
                last_status = time.monotonic()
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.read(1)
            if key == 'q':
                break
            if key == 'u' and records:
                gone = records.pop()
                with open(args.out, 'w') as f:
                    f.writelines(json.dumps(r) + '\n' for r in records)
                print(f'\n  removed "{gone["label"]}"')
            if key != ' ':
                continue
            print('\n  waiting for the marker to be static ...')
            end = time.monotonic() + args.timeout
            while True:
                rclpy.spin_once(n, timeout_sec=0.005)
                ok, mean, std, why = n.static()
                if ok or time.monotonic() > end:
                    break
            if not ok:
                print(f'  NOT captured after {args.timeout:.0f} s: {why}')
                continue
            k = sum(1 for r in records if r['label'] != 'reference')
            label = 'reference' if not records else f'return {k + 1}'
            rec = {'label': label, 'wall_time': time.time(), 'topic': args.topic,
                   'reference_topic': args.reference_topic, 'samples': len(n.buf),
                   'position': mean[:3, 3].tolist(),
                   'quaternion': Rotation.from_matrix(mean[:3, :3]).as_quat().tolist(),
                   'std_mm': std.tolist()}
            records.append(rec)
            with open(args.out, 'a') as f:
                f.write(json.dumps(rec) + '\n')
            print(f'  captured "{label}" ({why})')
            returns = [r for r in records if r['label'] != 'reference']
            if len(returns) >= 3:
                rp_mm, l, abc = rp(returns)
                print(f'  this return {l[-1]:.3f} mm from the barycentre of {len(returns)}; '
                      f'RP {rp_mm:.3f} mm, RPa/b/c {abc[0]:.3f}/{abc[1]:.3f}/{abc[2]:.3f} deg')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
