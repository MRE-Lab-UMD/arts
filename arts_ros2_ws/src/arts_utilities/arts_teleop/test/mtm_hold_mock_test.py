#!/usr/bin/env python3
"""mtm_hold.py: the Inverse3 handles are held while the head is out of the viewer.

    arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/mtm_hold_mock_test.py

Run it through the wrapper: it isolates the ROS domain, and a live haply_bridge would otherwise
send these targets to the real handles. Plays both cursors (500 Hz) and /console/head_sensor
(20 Hz) and reads /mtm_<side>/cursor_position_target. Checks, in order:
  1. head out: both targets at the handles' start; the left handle pushed 5 mm: its target stays
  2. head in: no targets (the bridge then frees the handles)
  3. head out again: held where the handles now are (no jump back)
  4. head in, then the sensor goes SILENT: held again after head_stale_s
"""
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

FAILS = []


def check(name, ok, detail):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        FAILS.append(name)


class Rig(Node):

    def __init__(self):
        super().__init__('mtm_hold_mock_test')
        self.cur = {s: self.create_publisher(PointStamped, f'/mtm_{s}/cursor', 10)
                    for s in ('left', 'right')}
        self.head_pub = self.create_publisher(Joy, '/console/head_sensor', 10)
        self.h = {'left': np.array([0.0, 0.1, 0.1]), 'right': np.array([0.0, 0.1, 0.1])}
        self.head = 0          # 1 present, 0 absent, None silent
        self.target = {'left': None, 'right': None}
        self.n = {'left': 0, 'right': 0}
        for s in ('left', 'right'):
            self.create_subscription(PointStamped, f'/mtm_{s}/cursor_position_target',
                                     lambda m, s=s: self._on_target(s, m), 10)
        self.create_timer(0.002, self._tick)
        self.create_timer(0.05, self._tick_head)

    def _on_target(self, s, m):
        self.target[s] = np.array([m.point.x, m.point.y, m.point.z])
        self.n[s] += 1

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for s in ('left', 'right'):
            m = PointStamped()
            m.header.stamp, m.header.frame_id = now, f'mtm_{s}_base'
            m.point.x, m.point.y, m.point.z = map(float, self.h[s])
            self.cur[s].publish(m)

    def _tick_head(self):
        if self.head is not None:
            j = Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.buttons = [int(self.head)]
            self.head_pub.publish(j)


def spin_for(node, s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.001)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    rclpy.init()
    rig = Rig()
    node = subprocess.Popen([sys.executable, os.path.join(here, '..', 'scripts', 'mtm_hold.py')])
    try:
        spin_for(rig, 3.0)

        print('[1] head out; then the left handle pushed 5 mm along +x')
        start = {k: v.copy() for k, v in rig.h.items()}
        ok = all(rig.target[k] is not None and np.allclose(rig.target[k], start[k]) for k in start)
        check('both held at their start', ok, f'{rig.target}')
        rig.h['left'] = rig.h['left'] + [0.005, 0, 0]
        spin_for(rig, 0.3)
        check('left target stays (the device pulls back)',
              np.allclose(rig.target['left'], start['left']), f'{rig.target["left"]}')

        print('[2] head in')
        rig.head = 1
        spin_for(rig, 0.3)
        n0 = dict(rig.n)
        spin_for(rig, 0.5)
        check('no targets: handles free', rig.n == n0,
              f'{rig.n["left"] - n0["left"]} / {rig.n["right"] - n0["right"]} targets')
        rig.h['left'] = rig.h['left'] + [0, 0.02, 0]
        spin_for(rig, 0.3)

        print('[3] head out again')
        rig.head = 0
        spin_for(rig, 0.3)
        check('held where it now is (no jump back)', np.allclose(rig.target['left'], rig.h['left']),
              f'{rig.target["left"]} vs handle {rig.h["left"]}')

        print('[4] head in, then the sensor goes silent')
        rig.head = 1
        spin_for(rig, 0.3)
        rig.h['left'] = rig.h['left'] + [0, 0, 0.01]
        spin_for(rig, 0.2)
        rig.head = None
        spin_for(rig, 0.5)
        n0 = rig.n['left']
        spin_for(rig, 0.2)
        check('silent sensor: held at the new point',
              rig.n['left'] > n0 and np.allclose(rig.target['left'], rig.h['left']),
              f'{rig.target["left"]}')

        print('[dup] a second mtm_hold started while this one runs')
        dup = subprocess.Popen(
            [sys.executable, os.path.join(here, '..', 'scripts', 'mtm_hold.py')],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            out = dup.communicate(timeout=15)[0].decode()
        except subprocess.TimeoutExpired:
            dup.kill()
            out = 'still running after 15 s'
        check('the second copy refuses to start', dup.returncode not in (0, None) and
              'already running' in out, out.strip().splitlines()[-1] if out.strip() else out)
        check('the first copy keeps running', node.poll() is None, f'poll {node.poll()}')
    finally:
        node.send_signal(2)
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
