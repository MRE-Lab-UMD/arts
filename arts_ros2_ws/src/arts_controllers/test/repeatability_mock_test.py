#!/usr/bin/env python3
"""The taught-pose repeatability runner, recorder and analyzer end to end, on mock hardware.

    ARTS_ARM=psm_left ARTS_WRIST=470093_prograsp_forceps.yaml \\
        run_servo_smoke_test.sh $PWD/repeatability_mock_test.py

Plays a NORMAL / program-running /<arm>/status (latched), as the real UR driver does, and a fake
tracker: /atracsys/Fake/measured_cp = the kinematic tip pose seen from a fixed tracker frame,
through a fixed marker offset, plus 0.005 mm noise (the fusionTrack's static noise), at 300 Hz.
The poses file is written here (P1 = where the arm stands, P2..P5 = 10 mm and 12-15 deg away),
as `teach` would write it. The protocol is a fast copy (min_hold 1 s, window 0.5 s,
settle_timeout 5 s). Checks:
  0. teach, driven through a pty as a keyboard: SPACE with the marker hidden is refused,
     SPACE, SPACE, u, SPACE, q writes 2 poses (the undo took one back) with their joints.
  1. No tracker: the runner refuses to start and the arm does not move.
  2. Tracker on, 3 cycles (15 stops); the marker is HIDDEN for 7 s from the arrival at cycle 2
     P3: that stop is logged as a miss (failure path of the static rule), the run goes on to DONE.
  3. The analyzer finds 14 complete stops and 1 miss, reports P1..P5 from the tracker with RP
     below 0.05 mm, and every measured stop is within 0.5 mm and 0.5 deg of its taught pose
     (kinematic window mean).
"""
import json
import os
import pty
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, ToolTipState

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, '..', 'bench')
sys.path.insert(0, HERE)
from arm_io import ARM, JOINT_STATES, TOOL_TIP_STATE  # noqa: E402

TRACKER = '/atracsys/Fake/measured_cp'
FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


class Rig(Node):

    def __init__(self):
        super().__init__('repeatability_mock_test')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status = self.create_publisher(ArmStatus, f'/{ARM}/status', latched)
        self.status.publish(ArmStatus(safety_mode=SafetyMode.NORMAL, program_running=True))
        self.tip = None
        self.q = None
        self.tracker_on = False
        self.hide_until = 0.0
        self.last = None
        self.rng = np.random.default_rng(1)
        # Tracker frame in the base, and the marker in the tip frame: fixed, arbitrary.
        self.cam = (Rotation.from_euler('xyz', [20, -35, 110], degrees=True), np.array([0.4, -0.9, 1.1]))
        self.marker = (Rotation.from_euler('xyz', [5, 80, -15], degrees=True), np.array([0.0, 0.02, -0.03]))
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(JointState, JOINT_STATES, self._on_joints, 10)
        self.create_subscription(String, f'/{ARM}/bench_event', self._on_event, 10)
        self.trk = self.create_publisher(PoseStamped, TRACKER, 10)
        self.create_timer(1.0 / 300.0, self._publish_tracker)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))

    def _on_joints(self, m):
        self.q = [p for n, p in zip(m.name, m.position) if '_tool_' not in n]

    def _on_event(self, m):
        e = json.loads(m.data)
        if e['event'] == 'arrive' and (e['cycle'], e['pose']) == (2, 3):
            self.hide_until = time.monotonic() + 7.0

    def _publish_tracker(self):
        if not self.tracker_on or self.tip is None:
            return
        if time.monotonic() < self.hide_until:
            # As the real tracker node does when it loses the marker (2026-11-07): it keeps
            # publishing the LAST pose, with a zero stamp.
            if self.last is not None:
                self.last.header.stamp.sec, self.last.header.stamp.nanosec = 0, 0
                self.trk.publish(self.last)
            return
        r_tip = Rotation.from_quat(self.tip[1])
        r_m = r_tip * self.marker[0]
        p_m = self.tip[0] + r_tip.apply(self.marker[1])
        r_c, p_c = self.cam
        r = r_c.inv() * r_m
        p = r_c.inv().apply(p_m - p_c) + self.rng.normal(0.0, 5e-6, 3)
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fake_tracker'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = r.as_quat()
        self.trk.publish(m)
        self.last = m


def main():
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    time.sleep(3.0)
    out_dir = tempfile.mkdtemp(prefix='repeatability_mock_')
    runner = os.path.join(BENCH, 'repeatability_real_arm.py')
    try:
        pos0, quat0 = rig.tip
        r0 = Rotation.from_quat(quat0)
        offs = [(np.zeros(3), np.zeros(3)), ([0.01, 0, 0], [12, 0, 0]), ([0, 0.01, 0], [0, 12, 0]),
                ([0, 0, 0.01], [0, 0, 15]), ([-0.007, -0.007, 0], [-8, 8, 0])]
        poses = [{'position': (pos0 + np.array(d)).tolist(),
                  'quaternion': (r0 * Rotation.from_rotvec(np.radians(a))).as_quat().tolist(),
                  'ur_joints': list(rig.q)} for d, a in offs]
        pfile = os.path.join(out_dir, 'poses.yaml')
        with open(pfile, 'w') as f:
            yaml.safe_dump({'arm': ARM, 'frame': f'{ARM}_base_link', 'tracker_topic': TRACKER,
                            'poses': poses}, f)
        proto = yaml.safe_load(open(os.path.join(BENCH, 'repeatability_protocol.yaml')))
        proto.update(min_hold=1.0, window=0.5, settle_timeout=5.0)
        prfile = os.path.join(out_dir, 'protocol.yaml')
        yaml.safe_dump(proto, open(prfile, 'w'))
        cmd = [sys.executable, runner, 'run', pfile, '--tracker-topic', TRACKER,
               '--protocol', prfile, '--cycles', '3']

        print('[0] teach through a pty')
        tfile = os.path.join(out_dir, 'taught.yaml')
        pid, fd = pty.fork()
        if pid == 0:
            os.execv(sys.executable, [sys.executable, runner, 'teach', tfile,
                                      '--tracker-topic', TRACKER])
        out = b''

        def feed(keys, wait):
            nonlocal out
            for k in keys:
                time.sleep(wait)
                os.write(fd, k)
            t_end = time.monotonic() + wait
            while time.monotonic() < t_end:
                try:
                    import select as sel
                    if sel.select([fd], [], [], 0.1)[0]:
                        out += os.read(fd, 4096)
                except OSError:
                    break
        feed([b' '], 4.0)              # tracker off: refused
        rig.tracker_on = True
        feed([b' ', b' ', b'u', b' ', b'q'], 1.0)
        os.waitpid(pid, 0)
        rig.tracker_on = False
        text = out.decode(errors='replace')
        taught = yaml.safe_load(open(tfile)) if os.path.exists(tfile) else {'poses': []}
        check('NOT saved' in text, 'teach refused a pose the tracker cannot see')
        check(len(taught['poses']) == 2 and len(taught['poses'][0]['ur_joints']) == 6,
              f'teach wrote {len(taught["poses"])} poses (expected 2 after the undo)')

        print('[1] no tracker')
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        check(r.returncode != 0 and 'no fresh' in r.stdout + r.stderr,
              f'runner refused without the tracker (exit {r.returncode})')
        check(np.linalg.norm(rig.tip[0] - pos0) < 1e-4,
              f'arm did not move: {1e3 * np.linalg.norm(rig.tip[0] - pos0):.3f} mm')

        print('[2] tracker on, 3 cycles, marker hidden at cycle 2 P3')
        rig.tracker_on = True
        time.sleep(1.0)
        bag = os.path.join(out_dir, 'bag')
        rec = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag, f'/{ARM}/bench_event',
             TOOL_TIP_STATE, TRACKER], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        time.sleep(2.0)
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        print('\n'.join('    ' + s for s in r.stdout.strip().splitlines()[-6:]))
        time.sleep(1.0)
        os.killpg(rec.pid, 2)
        rec.wait(timeout=10)
        check(r.returncode == 0 and 'DONE (1 misses)' in r.stdout,
              f'runner finished with exactly 1 miss (exit {r.returncode})')
        check('MISS cycle 2 P3' in r.stdout, 'the miss is the hidden stop (cycle 2 P3)')

        print('[3] analyzer')
        js = os.path.join(out_dir, 'results.json')
        a = subprocess.run([sys.executable, os.path.join(BENCH, 'repeatability_analyze.py'), bag,
                            '--arm', ARM, '--json', js], capture_output=True, text=True, timeout=120)
        print('\n'.join('    ' + s for s in a.stdout.strip().splitlines()))
        check(a.returncode == 0 and '14 complete stops' in a.stdout and 'misses (no static tracked '
              'window): 1' in a.stdout, 'analyzer: 14 complete stops, 1 miss')
        res = json.load(open(js)) if a.returncode == 0 else {}
        trk = res.get(TRACKER, {}).get('poses', {})
        check(sorted(trk) == ['P1', 'P2', 'P3', 'P4', 'P5'], f'tracker poses {sorted(trk)}')
        worst = max((v['rp_mm'] for v in trk.values()), default=float('nan'))
        check(worst < 0.05, f'tracker RP on mock {worst:.4f} mm (< 0.05)')
        check('overall' in res.get(TRACKER, {}) and res[TRACKER]['overall'].get('n') == 14,
              "A's pooled statistics over the 14 stops")

        sys.path.insert(0, BENCH)
        from repeatability_analyze import read  # noqa: E402
        data, _ = read(bag, {f'/{ARM}/bench_event', TOOL_TIP_STATE})
        wins = {}
        for t, m in data[f'/{ARM}/bench_event']:
            e = json.loads(m.data)
            if e['event'] == 'window_start':
                wins[(e['cycle'], e['pose'])] = [t]
            elif e['event'] == 'window_end':
                wins[(e['cycle'], e['pose'])].append(t)
        ts = np.array([t for t, _ in data[TOOL_TIP_STATE]])
        ps = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                       for _, m in data[TOOL_TIP_STATE]])
        qs = np.array([[m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                        m.pose.orientation.w] for _, m in data[TOOL_TIP_STATE]])
        perr, aerr = [], []
        for (c, k), w in wins.items():
            if len(w) != 2:
                continue
            sel = (ts >= w[0]) & (ts <= w[1])
            perr.append(np.linalg.norm(ps[sel].mean(axis=0) - np.array(poses[k - 1]['position'])))
            rm = Rotation.from_quat(qs[sel]).mean()
            aerr.append(np.degrees((rm.inv() * Rotation.from_quat(poses[k - 1]['quaternion'])).magnitude()))
        check(len(perr) == 14 and max(perr) < 5e-4 and max(aerr) < 0.5,
              f'every measured stop at its taught pose: max {1e3 * max(perr, default=np.nan):.3f} mm, '
              f'{max(aerr, default=np.nan):.3f} deg')
    finally:
        rig.tracker_on = False
        shutil.rmtree(out_dir, ignore_errors=True)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
