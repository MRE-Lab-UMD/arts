#!/usr/bin/env python3
"""Live readout while teaching repeatability poses: watch the numbers move as you jog.

    python3 teach_monitor.py --arm psm_left --tracker-topic /atracsys/Prograsp/measured_cp \
        [--saved x,y,z ...] [--target roll,pitch,yaw]

Prints twice a second, on one line: the instrument roll / pitch / yaw (deg) from
<arm>/tool_joint_states, with the distance to --target if given; whether the marker is tracked
and its registration error; and the marker's distance (mm, tracker frame) from each --saved
marker position (taken at the poses already saved). Wrist margins: pitch within +-55 deg and
yaw within +-65 deg keep 15 deg from the ProGrasp limits; outside them the line says LIMIT.
Read-only: it subscribes and prints.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation
import os
import yaml

from arts_interfaces.msg import ToolTipState


class Mon(Node):

    def __init__(self, a):
        super().__init__('teach_monitor')
        self.arm = a.arm
        self.q = {}
        self.marker = None
        self.marker_t = 0.0
        self.reg = float('nan')
        self.tip = None
        self.create_subscription(JointState, f'/{a.arm}/tool_joint_states', self._on_q, 10)
        self.create_subscription(ToolTipState, f'/{a.arm}/tool_tip_state', self._on_tip, 10)
        self.create_subscription(PoseStamped, a.tracker_topic, self._on_m, 10)
        self.create_subscription(Float64, a.tracker_topic.rsplit('/', 1)[0] + '/registration_error',
                                 self._on_reg, 10)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))

    def _on_q(self, m):
        for n, p in zip(m.name, m.position):
            self.q[n.replace(f'{self.arm}_tool_', '')] = math.degrees(p)

    def _on_m(self, m):
        if m.header.stamp.sec == 0 and m.header.stamp.nanosec == 0:
            return    # the tracker node keeps publishing the LAST pose with a zero stamp while the marker is not seen (2026-11-07)
        p = m.pose.position
        self.marker = np.array([p.x, p.y, p.z])
        self.marker_t = time.monotonic()

    def _on_reg(self, m):
        self.reg = m.data


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm', default='psm_left')
    ap.add_argument('--tracker-topic', required=True)
    ap.add_argument('--saved', nargs='*', default=[], help='x,y,z marker positions (m)')
    ap.add_argument('--target', help='roll,pitch,yaw (deg)')
    ap.add_argument('--poses', help='the teach file: a refreshing table of how far the tip is '
                    '(mm, deg) from each pose already saved in it')
    ap.add_argument('--goals', nargs='*', default=[],
                    help='name:roll,pitch,yaw:x,y,z per goal pose (deg; marker m): a refreshing '
                         'table of what is left to each goal')
    a = ap.parse_args()
    saved = [np.array([float(v) for v in s.split(',')]) for s in a.saved]
    target = [float(v) for v in a.target.split(',')] if a.target else None
    goals = []
    for g in a.goals:
        name, ang, xyz = g.split(':')
        goals.append((name, [float(v) for v in ang.split(',')],
                      np.array([float(v) for v in xyz.split(',')])))
    rclpy.init()
    n = Mon(a)
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - last < 0.5:
                continue
            last = time.monotonic()
            r, p, y = (n.q.get(k, float('nan')) for k in ('roll', 'wrist_pitch', 'wrist_yaw'))
            line = f'roll {r:+6.0f}  pitch {p:+5.0f}  yaw {y:+5.0f}'
            if target:
                line += (f'  (to target: roll {target[0] - r:+4.0f} pitch {target[1] - p:+4.0f} '
                         f'yaw {target[2] - y:+4.0f})')
            if abs(p) > 55 or abs(y) > 65:
                line += '  LIMIT'
            vis = n.marker is not None and time.monotonic() - n.marker_t < 0.2
            line += f'  | marker {"OK " if vis else "LOST"}'
            if vis:
                line += f' err {1e3 * n.reg:.2f} mm'
                for i, s in enumerate(saved):
                    line += f'  P{i + 1} {1e3 * np.linalg.norm(n.marker - s):4.0f} mm'
            if a.poses:
                vis = n.marker is not None and time.monotonic() - n.marker_t < 0.2
                saved_p = []
                if os.path.exists(a.poses):
                    try:
                        with open(a.poses) as fh:
                            saved_p = (yaml.safe_load(fh) or {}).get('poses', [])
                    except (OSError, yaml.YAMLError):
                        pass
                out = ['\033[2J\033[H' + time.strftime('%H:%M:%S') + f'   {len(saved_p)} poses saved',
                       f'NOW   roll {r:+5.0f}  pitch {p:+5.0f}  yaw {y:+5.0f}   marker '
                       + (f'OK (err {1e3 * n.reg:.2f} mm)   SPACE allowed' if vis
                          else 'LOST  <-- turn back toward the tracker')
                       + ('   WRIST NEAR LIMIT' if abs(p) > 55 or abs(y) > 65 else ''), '']
                if saved_p and n.tip is not None:
                    out.append('from saved pose:  tip distance   orientation change')
                    for i, sp in enumerate(saved_p):
                        d = 1e3 * np.linalg.norm(n.tip[0] - np.array(sp['position']))
                        ang = np.degrees((Rotation.from_quat(sp['quaternion']).inv()
                                          * n.tip[1]).magnitude())
                        far = d >= 50 or ang >= 30
                        out.append(f'  P{i + 1}             {d:6.0f} mm      {ang:6.0f} deg   '
                                   + ('ok, different' if far else '<-- too similar'))
                    out += ['', 'save when: marker OK, and every row says "ok, different" '
                            '(>= 50 mm OR >= 30 deg)']
                print('\n'.join(out), flush=True)
                continue
            if not goals:
                print(line, flush=True)
                continue
            vis = n.marker is not None and time.monotonic() - n.marker_t < 0.2
            out = ['\033[2J\033[H' + time.strftime('%H:%M:%S'),
                   f'NOW   roll {r:+5.0f}  pitch {p:+5.0f}  yaw {y:+5.0f}   marker '
                   + (f'OK (err {1e3 * n.reg:.2f} mm)' if vis else 'LOST  <-- turn back')
                   + ('   WRIST NEAR LIMIT' if abs(p) > 55 or abs(y) > 65 else ''),
                   '',
                   'goal   still to go:  roll  pitch   yaw    marker distance',
                   ]
            for name, (gr, gp, gy), gx in goals:
                dist = (f'{1e3 * np.linalg.norm(n.marker - gx):5.0f} mm' if vis else '    ? ')
                ok = (vis and abs(gr - r) <= 5 and abs(gp - p) <= 5 and abs(gy - y) <= 5
                      and 1e3 * np.linalg.norm(n.marker - gx) <= 10)
                out.append(f'{name:5s}            {gr - r:+6.0f} {gp - p:+6.0f} {gy - y:+6.0f}    '
                           f'{dist}   {"<-- AT THIS GOAL: tell Claude" if ok else ""}')
            print('\n'.join(out), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
