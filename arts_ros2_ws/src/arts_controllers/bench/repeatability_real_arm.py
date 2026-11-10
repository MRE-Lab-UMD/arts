#!/usr/bin/env python3
"""Pose repeatability (ISO 9283) on an ARTS arm, through the same input teleop uses.

Collaborator A's procedure (repeatability_protocol.yaml): teach 5 poses by hand, then drive
P1 -> P5 `cycles` times in POSE mode and log each pose once the instrument is static and still
tracked. The arm under External Control with its instrument engaged and zeroed; no cannula, so
the RCM may stay off (if it is on, it is kept, and a QP failure aborts).

    # 1. teach: jog with the gamepad (hold RB), SPACE saves the pose, u undoes, q writes and quits
    python3 repeatability_real_arm.py teach POSES.yaml [--tracker-topic /atracsys/<tool>/measured_cp]
    # 2. run (under run_repeatability.sh, which records the bag); a hand on the pendant e-stop
    python3 repeatability_real_arm.py run POSES.yaml --tracker-topic ... [--cycles N]

Teach stores the tip pose from <arm>/tool_tip_state and the UR joints at each SPACE; with a
tracker topic it shows whether the marker is visible and refuses a pose where it is not.

Run: one approach move from where the arm stands to P1 (not evaluated), then the schedule. Each
move is a straight line in position with slerp in orientation on one trapezoid
(repeatability_protocol.pose_segment), streamed at 500 Hz as <arm>/tool_tip_target with linear
and angular feedforward; the jaw is never commanded (the marker clip holds it closed). At each
pose the target is held; after min_hold s the static rule runs on the tracker samples (on the
kinematic tip if no tracker topic is given: the mock test), and the measured window is the next
`window` s, which must pass again. Events on /<arm>/bench_event (std_msgs/String, JSON):
start, arrive (cycle, pose, goal), window_start / window_end (with settle_s and, with a tracker,
the window's mean marker pose), window_discard (moved inside the window), miss (no static window
within settle_timeout: logged, the run goes on), done. The numbers come from the bag
(repeatability_analyze.py).

Stops publishing (the controller then holds) and exits on: the arm leaving NORMAL / the program
stopping, the RCM (if on) switched off or its QP failing, the measured tip more than 15 mm from
the commanded one, or a UR joint more than 15 deg outside the range the taught poses span.
"""
import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import ArmStatus, RcmState, ToolTipState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'test'))
sys.path.insert(0, HERE)
from arm_io import (ARM, JOINT_STATES, STATUS, TARGET_TOPIC, TOOL_TIP_STATE,  # noqa: E402
                    ToolTipTarget, target)
import repeatability_protocol as proto  # noqa: E402

ABORT_TRACKING = 0.015              # m, measured tip vs commanded
ABORT_JOINT = math.radians(15.0)    # beyond the taught poses' joint range
RATE_HZ = 500.0


class Runner(Node):

    def __init__(self, tracker_topic, p):
        super().__init__('repeatability_real_arm')
        self.tip = None
        self.tip_time = 0.0
        self.q = None
        self.qd = None
        self.safety = None
        self.running = None
        self.rcm = None
        self.lock = threading.Lock()
        self.static = proto.Static(p)
        self.tracker_topic = tracker_topic
        self.tracker_time = 0.0
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.create_subscription(JointState, JOINT_STATES, self._on_joints, 10)
        self.create_subscription(RcmState, f'/{ARM}/rcm_state', self._on_rcm, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ArmStatus, STATUS, self._on_status, latched)
        if tracker_topic:
            self.create_subscription(PoseStamped, tracker_topic, self._on_tracker, 100)
        self.pub = self.create_publisher(ToolTipTarget, TARGET_TOPIC, 10)
        self.event_pub = self.create_publisher(String, f'/{ARM}/bench_event', 10)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        with self.lock:
            self.tip = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))
            self.tip_time = time.monotonic()
            if not self.tracker_topic:
                self.static.add(self.tip_time, *self.tip)

    def _on_tracker(self, m):
        # Zero stamp: the tracker node keeps publishing the LAST pose with a zero stamp while the marker is not seen (2026-11-07); not a measurement.
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return
        p, o = m.pose.position, m.pose.orientation
        with self.lock:
            self.tracker_time = time.monotonic()
            self.static.add(self.tracker_time, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def _on_joints(self, m):
        # The UR joints only; joint_states also carries the instrument's (decision 18).
        ur = [(p, v) for n, p, v in zip(m.name, m.position, list(m.velocity) or [0.0] * len(m.name))
              if '_tool_' not in n]
        with self.lock:
            self.q = np.array([p for p, _ in ur])
            self.qd = np.array([v for _, v in ur])

    def _on_rcm(self, m):
        self.rcm = m

    def _on_status(self, m):
        self.safety, self.running = m.safety_mode, m.program_running

    def tool(self):
        with self.lock:
            if self.tip is None or time.monotonic() - self.tip_time > 0.2:
                raise RuntimeError(f'no fresh {TOOL_TIP_STATE}')
            return self.tip[0].copy(), self.tip[1].copy()

    def tracker_visible(self):
        return time.monotonic() - self.tracker_time < 0.1

    def static_test(self):
        with self.lock:
            return self.static.test(time.monotonic())

    def send(self, pos, quat, v=None, w=None):
        m = target(self.get_clock().now().to_msg(), pos, quat, v)
        if w is not None:
            m.velocity.angular.x, m.velocity.angular.y, m.velocity.angular.z = map(float, w)
        self.pub.publish(m)

    def event(self, **kw):
        self.event_pub.publish(String(data=json.dumps(kw)))

    def ready(self):
        return self.safety == SafetyMode.NORMAL and self.running


EXECUTOR = []


def start_node(tracker_topic, p):
    rclpy.init()
    n = Runner(tracker_topic, p)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    EXECUTOR.append((ex, n))

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()
    time.sleep(2.0)
    if n.q is None or n.tip is None:
        sys.exit(f'no {JOINT_STATES} or {TOOL_TIP_STATE}: is the arm up?')
    return n


def teach(args, p):
    n = start_node(args.tracker_topic, p)
    poses = []
    if os.path.exists(args.poses):
        sys.exit(f'{args.poses} exists: teach writes a NEW file')
    os.makedirs(os.path.dirname(os.path.abspath(args.poses)), exist_ok=True)

    def write():
        # After every change, so a crash or a closed terminal loses nothing taught.
        tmp = args.poses + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump({'arm': ARM, 'frame': f'{ARM}_base_link',
                            'tracker_topic': args.tracker_topic,
                            'taught': time.strftime('%Y-%m-%dT%H:%M:%S'), 'poses': poses}, f)
        os.replace(tmp, args.poses)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print(f'teach {p["poses"]} poses on {ARM}: jog (gamepad, hold RB), SPACE save, u undo, '
          f'q write and quit')
    last = 0.0
    try:
        while len(poses) < p['poses']:
            if time.monotonic() - last > 0.5:
                pos, _ = n.tool()
                vis = ('' if not args.tracker_topic else
                       ('  marker VISIBLE' if n.tracker_visible() else '  marker NOT visible'))
                print(f'\r  P{len(poses) + 1}: tip {np.round(1e3 * pos, 1)} mm{vis}      ',
                      end='', flush=True)
                last = time.monotonic()
            if not select.select([sys.stdin], [], [], 0.02)[0]:
                continue
            key = sys.stdin.read(1)
            if key == 'q':
                break
            if key == 'u' and poses:
                poses.pop()
                print(f'\n  removed P{len(poses) + 1}')
                write()
            if key != ' ':
                continue
            if args.tracker_topic and not n.tracker_visible():
                print('\n  NOT saved: the tracker does not see the marker here')
                continue
            pos, quat = n.tool()
            poses.append({'position': pos.tolist(), 'quaternion': quat.tolist(),
                          'ur_joints': n.q.tolist()})
            write()
            print(f'\n  saved P{len(poses)} (written to {args.poses})')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print()
    if len(poses) < 2:
        sys.exit(f'fewer than 2 poses ({args.poses} holds {len(poses)}): too few to run')
    print(f'{len(poses)} poses in {args.poses}')
    return 0


def run(args, p):
    with open(args.poses) as f:
        taught = yaml.safe_load(f)
    if taught['arm'] != ARM:
        sys.exit(f'{args.poses} was taught on {taught["arm"]}, ARTS_ARM is {ARM}')
    goals = [(np.array(g['position']), np.array(g['quaternion'])) for g in taught['poses']]
    p['poses'] = len(goals)
    jt = np.array([g['ur_joints'] for g in taught['poses']])
    j_lo, j_hi = jt.min(axis=0) - ABORT_JOINT, jt.max(axis=0) + ABORT_JOINT

    n = start_node(args.tracker_topic, p)
    if n.safety is None:
        sys.exit(f'no {STATUS}')
    if not n.ready():
        sys.exit(f'arm not ready: safety mode {n.safety}, program running {n.running}')
    rcm_on = bool(n.rcm and n.rcm.enabled)
    if rcm_on and not n.rcm.constraint_ok:
        sys.exit('the RCM is on but its QP is failing')
    if args.tracker_topic and not n.tracker_visible():
        sys.exit(f'no fresh {args.tracker_topic}: is the tracker running and the marker in view?')

    state = {'cmd': None}

    def fault():
        here, _ = n.tool()
        if state['cmd'] is not None and np.linalg.norm(here - state['cmd']) > ABORT_TRACKING:
            return f'tip {1e3 * np.linalg.norm(here - state["cmd"]):.1f} mm from the command'
        if np.any(n.q < j_lo) or np.any(n.q > j_hi):
            return 'a UR joint left the taught range + 15 deg'
        if rcm_on and not (n.rcm.enabled and n.rcm.constraint_ok):
            return 'RCM switched off or its QP failed'
        if not n.ready():
            return f'safety mode {n.safety}, program running {n.running}'
        return None

    def move(a, b):
        dur, f = proto.pose_segment(a[0], a[1], b[0], b[1], p)
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < dur:
            pos, quat, v, w = f(t)
            state['cmd'] = pos
            n.send(pos, quat, v, w)
            if (why := fault()):
                return why
            time.sleep(1.0 / RATE_HZ)
        state['cmd'] = b[0]
        return None

    def hold_until(goal, end_cond, timeout):
        """Hold the goal until end_cond() is truthy or timeout; (result, fault)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            n.send(goal[0], goal[1])
            if (why := fault()):
                return None, why
            if (r := end_cond()):
                return r, None
            time.sleep(1.0 / RATE_HZ)
        return None, None

    def settle(cycle, k, goal):
        """Hold until a static, tracked window is measured; True, False (miss) or a fault."""
        t_arrive = time.monotonic()
        _, why = hold_until(goal, lambda: False, p['min_hold'])
        if why:
            return why
        while time.monotonic() - t_arrive < p['settle_timeout']:
            left = p['settle_timeout'] - (time.monotonic() - t_arrive)
            ok, why = hold_until(goal, lambda: n.static_test()[0], left)
            if why:
                return why
            if not ok:
                break
            t_ws = time.monotonic()
            n.event(event='window_start', cycle=cycle, pose=k, settle_s=t_ws - t_arrive)
            with n.lock:
                n.static.clear()
            _, why = hold_until(goal, lambda: time.monotonic() - t_ws >= p['window'] * 1.02,
                                p['window'] * 1.5)
            if why:
                return why
            ok, mean, quat, std, desc = n.static_test()
            if ok:
                n.event(event='window_end', cycle=cycle, pose=k, settle_s=t_ws - t_arrive,
                        tracker=bool(args.tracker_topic), mean=mean.tolist(),
                        quaternion=quat.tolist(), std_mm=std.tolist())
                return True
            n.event(event='window_discard', cycle=cycle, pose=k, why=desc)
        _, _, _, _, desc = n.static_test()
        n.event(event='miss', cycle=cycle, pose=k, why=desc)
        print(f'\n  MISS cycle {cycle} P{k}: no static tracked window in '
              f'{p["settle_timeout"]:.0f} s ({desc})')
        return False

    sched = proto.schedule(p)
    print(f'{ARM}: {len(goals)} taught poses, {p["cycles"]} cycles ({len(sched)} stops), '
          f'{p["speed"] * 1e3:.0f} mm/s / {p["angular_speed"]:.0f} deg/s, RCM '
          f'{"on" if rcm_on else "off"}, static from '
          f'{args.tracker_topic or TOOL_TIP_STATE + " (no tracker)"}')
    n.event(event='start', arm=ARM, goals=[[g[0].tolist(), g[1].tolist()] for g in goals],
            protocol=p, tracker_topic=args.tracker_topic, rcm=rcm_on, poses_file=args.poses)
    here = n.tool()
    why = move(here, goals[0])            # approach P1, not evaluated
    if why:
        print(f'ABORT: {why}; stopped publishing')
        return 1
    here = goals[0]
    misses = 0
    t_run = time.monotonic()
    for cycle, k in sched:
        goal = goals[k - 1]
        if (why := move(here, goal)):
            print(f'ABORT: {why}; stopped publishing')
            return 1
        n.event(event='arrive', cycle=cycle, pose=k, goal=goal[0].tolist())
        r = settle(cycle, k, goal)
        if isinstance(r, str):
            print(f'ABORT: {r}; stopped publishing')
            return 1
        misses += not r
        here = goal
        if k == p['poses']:
            print(f'  cycle {cycle}/{p["cycles"]} done ({time.monotonic() - t_run:.0f} s, '
                  f'{misses} misses)')
    n.event(event='done', misses=misses)
    print(f'\nDONE ({misses} misses)')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['teach', 'run'])
    ap.add_argument('poses', help='taught poses (YAML): written by teach, read by run')
    ap.add_argument('--tracker-topic', help='geometry_msgs/PoseStamped of the marker')
    ap.add_argument('--protocol', default=os.path.join(HERE, 'repeatability_protocol.yaml'))
    ap.add_argument('--cycles', type=int, help='override the protocol (a short dry run)')
    args = ap.parse_args()
    p = proto.load(args.protocol)
    if args.cycles:
        p['cycles'] = args.cycles
    return teach(args, p) if args.mode == 'teach' else run(args, p)


if __name__ == '__main__':
    try:
        code = main()
    except SystemExit as e:
        code = e.code
    finally:
        # Stop the spinning executor before rclpy goes away under it (a callback racing the
        # shutdown aborts the process).
        for ex, node in EXECUTOR:
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if isinstance(code, str):
        print(code, file=sys.stderr)
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # os._exit: the daemon spin thread can still abort the interpreter's own teardown.
    os._exit(code or 0)
