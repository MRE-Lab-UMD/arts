#!/usr/bin/env python3
"""The RCM-drift tools end to end on mock hardware: pivot motion, live pivot monitor, recorded
drift trial and its analyzer.

    ARTS_ARM=psm_left ARTS_WRIST=470093_prograsp_forceps.yaml \\
        run_servo_smoke_test.sh $PWD/rcm_drift_mock_test.py

Plays a NORMAL / program-running /<arm>/status (latched) and a fake tracker: a marker rigidly on
the instrument SHAFT (the tip pose taken back through the measured wrist pitch and yaw with the
controller's wrist geometry: a tip-frame marker would turn with the wrist, which absorbs part of
the orientation while the arm lags the moving target) whose ORIGIN sits on the shaft 2.0 mm from the RCM
point, seen from a fixed tracker frame with 0.005 mm noise, at 300 Hz; optionally with a slow
drift added to every tracked position (an RCM that walks, as the tracker would see it). Checks:
  1. RCM off: rcm_motion_real_arm.py refuses and the arm does not move.
  2. RCM on (100 mm): wobble for 16 s while pivot_monitor.py fits the pivot live: it sees the
     origin where the TRUTH says (the held RCM point in the noise-free marker frame) within
     0.1 mm, with >= 10 deg of rotation, and does NOT say aligned
     (threshold 0.5 mm); the kinematic RCM deviation stays under 0.05 mm; the arm ends at the start.
  3. A 45 s drift trial, recorded: the analyzer reports pivot error < 0.05 mm (mean), |d| 2.0 +-
     0.2 mm, drift slope < 0.1 mm/min, kinematic median < 0.01 mm.
  4. Failure path: the same trial with 1.0 mm/min of drift injected into the tracker: the
     analyzer's slope reads 1.0 +- 0.3 mm/min.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, RcmState, ToolTipState
from arts_interfaces.srv import SetRcm

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, '..', 'bench')
sys.path.insert(0, HERE)
from arm_io import ARM, TOOL_TIP_STATE  # noqa: E402

TRACKER = '/atracsys/FakeShaft/measured_cp'
OFFSET = 0.002            # m, marker origin from the RCM along the shaft (toward the flange)
PITCH_TO_YAW = 0.009      # m, the controller's wrist.pitch_to_yaw (its default)
FAILS = []


def check(ok, msg):
    print(f'{"PASS" if ok else "FAIL"} {msg}')
    if not ok:
        FAILS.append(msg)


class Rig(Node):

    def __init__(self):
        super().__init__('rcm_drift_mock_test')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status = self.create_publisher(ArmStatus, f'/{ARM}/status', latched)
        self.status.publish(ArmStatus(safety_mode=SafetyMode.NORMAL, program_running=True))
        self.tip = None
        self.wrist = (0.0, 0.0)             # measured pitch, yaw
        self.rcm = None
        self.max_dev = 0.0
        self.marker_in_tip = None           # (Rotation, offset) once the RCM is known
        self.drift_m_s = 0.0
        self.drift_t0 = 0.0
        self.truth = []                     # the held RCM point in the marker frame (noise free)
        self.rng = np.random.default_rng(2)
        self.cam = (Rotation.from_euler('xyz', [-30, 25, 160], degrees=True), np.array([0.3, 1.2, 0.9]))
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        self.create_subscription(JointState, f'/{ARM}/tool_joint_states', self._on_joints, 10)
        self.set_rcm = self.create_client(SetRcm, f'/{ARM}/set_rcm')
        self.trk = self.create_publisher(PoseStamped, TRACKER, 10)
        self.create_timer(1.0 / 300.0, self._publish_tracker)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))

    def _on_joints(self, m):
        j = dict(zip(m.name, m.position))
        pitch, yaw = j.get(f'{ARM}_tool_wrist_pitch'), j.get(f'{ARM}_tool_wrist_yaw')
        if pitch is not None and yaw is not None:
            self.wrist = (pitch, yaw)

    def shaft(self):
        """The shaft frame (at the pitch axis, not turned by the wrist) from the tip pose:
        tip = shaft * Rx(pitch) * T(0, 0, pitch_to_yaw) * R(-y axis, yaw)."""
        p, r = self.tip
        pitch, yaw = self.wrist
        rp = Rotation.from_euler('x', pitch)
        r_s = r * (rp * Rotation.from_euler('y', -yaw)).inv()
        return p - (r_s * rp).apply([0.0, 0.0, PITCH_TO_YAW]), r_s

    def _on_rcm(self, m):
        self.rcm = m
        if m.enabled and np.isfinite(m.deviation):
            self.max_dev = max(self.max_dev, m.deviation)

    def attach_marker(self):
        """Fix the marker on the shaft: origin on the shaft axis, OFFSET from the RCM."""
        p, r = self.shaft()
        c = np.array([self.rcm.point.x, self.rcm.point.y, self.rcm.point.z])
        axis = r.apply([0.0, 0.0, 1.0])
        origin = c - OFFSET * axis
        r_marker = Rotation.from_euler('xyz', [10, -70, 35], degrees=True)
        self.marker_in_tip = (r.inv() * r_marker, r.inv().apply(origin - p))

    def _publish_tracker(self):
        if self.marker_in_tip is None or self.tip is None:
            return
        p, r = self.shaft()
        r_m = r * self.marker_in_tip[0]
        p_m = p + r.apply(self.marker_in_tip[1])
        if self.rcm is not None and self.rcm.enabled:
            c = np.array([self.rcm.point.x, self.rcm.point.y, self.rcm.point.z])
            self.truth.append(r_m.inv().apply(c - p_m))
        if self.drift_m_s:
            p_m = p_m + np.array([1.0, 0.0, 0.0]) * self.drift_m_s * (time.monotonic() - self.drift_t0)
        r_c, p_c = self.cam
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fake_tracker'
        pos = r_c.inv().apply(p_m - p_c) + self.rng.normal(0.0, 5e-6, 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = (r_c.inv() * r_m).as_quat()
        self.trk.publish(m)

    def rcm_switch(self, on):
        fut = self.set_rcm.call_async(SetRcm.Request(enable=on, from_current_shaft=True,
                                                     insertion_depth=0.10))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        return fut.result()


def record(bag):
    return subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag, f'/{ARM}/bench_event',
                             f'/{ARM}/rcm_state', TRACKER], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def trial(rig, out_dir, name, drift_m_s):
    bag = os.path.join(out_dir, name)
    rec = record(bag)
    time.sleep(2.0)
    rig.drift_m_s, rig.drift_t0 = drift_m_s, time.monotonic()
    r = subprocess.run([sys.executable, os.path.join(BENCH, 'rcm_motion_real_arm.py'), 'drift',
                        '--minutes', '0.75', '--seed', '3'], capture_output=True, text=True,
                       timeout=180)
    rig.drift_m_s = 0.0
    time.sleep(1.0)
    os.killpg(rec.pid, 2)
    rec.wait(timeout=10)
    js = os.path.join(out_dir, name + '.json')
    a = subprocess.run([sys.executable, os.path.join(BENCH, 'rcm_drift_analyze.py'), bag,
                        '--arm', ARM, '--window-s', '10', '--json', js], capture_output=True, text=True, timeout=120)
    print('\n'.join('    ' + s for s in (r.stdout.strip().splitlines()[-2:]
                                         + a.stdout.strip().splitlines())))
    res = json.load(open(js))[0] if a.returncode == 0 and os.path.exists(js) else {}
    return r.returncode, res


def main():
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    rig.set_rcm.wait_for_service(timeout_sec=10.0)
    time.sleep(2.0)
    out_dir = tempfile.mkdtemp(prefix='rcm_drift_mock_')
    mover = os.path.join(BENCH, 'rcm_motion_real_arm.py')
    try:
        print('[1] RCM off')
        rig.rcm_switch(False)
        time.sleep(1.0)
        p0 = rig.tip[0].copy()
        r = subprocess.run([sys.executable, mover, 'wobble', '--seconds', '5'], capture_output=True,
                           text=True, timeout=60)
        check(r.returncode != 0 and 'RCM is off' in r.stdout + r.stderr,
              f'mover refused without the RCM (exit {r.returncode})')
        check(np.linalg.norm(rig.tip[0] - p0) < 1e-4,
              f'arm did not move: {1e3 * np.linalg.norm(rig.tip[0] - p0):.3f} mm')

        print('[2] RCM on, wobble + live pivot monitor')
        resp = rig.rcm_switch(True)
        check(resp is not None and resp.success, f'RCM on ({resp.message if resp else "no response"})')
        time.sleep(1.0)
        rig.attach_marker()
        rig.max_dev = 0.0
        start = rig.tip[0].copy()
        mon_json = os.path.join(out_dir, 'monitor.json')
        rig.truth = []
        mon = subprocess.Popen([sys.executable, os.path.join(BENCH, 'pivot_monitor.py'), '--topic',
                                TRACKER, '--duration', '15', '--json', mon_json],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        r = subprocess.run([sys.executable, mover, 'wobble', '--seconds', '16'], capture_output=True,
                           text=True, timeout=90)
        mout, _ = mon.communicate(timeout=30)
        print('\n'.join('    ' + s for s in mout.strip().splitlines()[-3:]))
        check(r.returncode == 0 and 'DONE' in r.stdout, f'wobble finished (exit {r.returncode})')
        m = json.load(open(mon_json)) if os.path.exists(mon_json) else {}
        tr = 1e3 * np.array(rig.truth)
        true_d = float(np.linalg.norm(tr.mean(axis=0)))
        print(f'    truth: the held RCM point is {true_d:.3f} mm from the marker origin in the marker '
              f'frame (placed at {1e3 * OFFSET:.1f}), moving {tr.std(axis=0).max():.4f} mm (std) in it')
        check(abs(m.get('d_norm_mm', 99) - true_d) < 0.1 and m.get('range_deg', 0) >= 10,
              f'monitor: origin {m.get("d_norm_mm", float("nan")):.2f} mm from the pivot '
              f'(truth {true_d:.2f}), {m.get("range_deg", 0):.0f} deg of rotation')
        check(abs(abs(m.get('along_mm', 99)) - true_d) < 0.1 and m.get('across_mm', 99) < 0.1,
              f'monitor splits it: along {m.get("along_mm", float("nan")):+.2f} mm (slide), across '
              f'{m.get("across_mm", float("nan")):.2f} mm (the origin is on the axis)')
        check(m.get('aligned') is False, 'monitor does not call a 2 mm offset aligned')
        check(rig.max_dev < 5e-5, f'kinematic RCM deviation max {1e3 * rig.max_dev:.4f} mm')
        time.sleep(0.5)
        check(np.linalg.norm(rig.tip[0] - start) < 5e-4,
              f'back at the start: {1e3 * np.linalg.norm(rig.tip[0] - start):.3f} mm')

        print('[3] drift trial, 45 s')
        code, res = trial(rig, out_dir, 'bag_clean', 0.0)
        e, dr, k = res.get('pivot_error_mm', {}), res.get('drift', {}), res.get('kinematic_mm', {})
        check(code == 0 and res.get('result') == 'done', f'drift trial finished (exit {code})')
        check(e.get('mean', 99) < 0.05, f'pivot error {e.get("mean", float("nan")):.4f} mm (< 0.05)')
        check(abs(res.get('d_norm_mm', 99) - true_d) < 0.1,
              f'|d| {res.get("d_norm_mm", float("nan")):.2f} mm (truth {true_d:.2f})')
        check(dr.get('slope_mm_per_min', 99) < 0.1,
              f'drift slope {dr.get("slope_mm_per_min", float("nan")):.4f} mm/min (< 0.1)')
        check(k.get('median', 99) < 0.01, f'kinematic median {k.get("median", float("nan")):.4f} mm')

        print('[4] drift trial with 1.0 mm/min injected into the tracker')
        code, res = trial(rig, out_dir, 'bag_drift', 1.0e-3 / 60.0)
        dr = res.get('drift', {})
        check(abs(dr.get('slope_mm_per_min', 99) - 1.0) < 0.3,
              f'analyzer sees the injected drift: {dr.get("slope_mm_per_min", float("nan")):.3f} '
              f'mm/min (injected 1.0)')
    finally:
        rig.rcm_switch(False)
        if not os.environ.get("KEEP"):
            shutil.rmtree(out_dir, ignore_errors=True)
        else:
            print("kept", out_dir)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
