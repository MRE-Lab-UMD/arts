#!/usr/bin/env python3
"""Two-handed camera control (camera_teleop.py) on a mock camera arm.

    ARTS_ARM=ecm arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/camera_teleop_mock_test.py

The test publishes the optical frame under ecm_base_link looking down base -z (Rx(180)),
50 mm below the CURRENT tip (50 Hz, moving with the scope as the real frame does), so the tip
lies on the optical axis. camera_teleop runs in clutch mode enabled by the service (no head
sensor; the left pedal is the clutch), tilt 0, scale 0.5, the measured left/right axis maps. Handles start 200 mm
apart along the screen x. With the display's 180 deg rotation the screen x maps to base -x.
  1. pedal up: both hands move, the camera stays
  2. pedal down, both hands 40 mm screen-right: camera +20 mm along base x (camera_direction
     -1: against the hands), orientation kept;
     then the left hand alone 30 mm screen-right and back: the camera does not move;
     2c. clutch (left pedal) held during camera control: both hands 30 mm screen-right, the
     camera stays and the bar force is zero; clutch released: no jump, then both hands 20 mm
     screen-right move the camera +10 mm (control resumed from the new hand positions)
  3. a small twist (left 5 mm down, right 5 mm up, 2.9 deg) inside the 5 deg dead band:
     no roll; steering to 21.8 deg (40 mm each, over 2 s so the hands stay under the 15 deg/s
     rate limit): the scope turns (21.8 - 5) x 1.0 = 16.8 deg (roll_direction -1)
     about base -z (the optical axis), the tip stays on the axis
     (< 1 mm), and it stops when the hands stop (< 0.5 deg more over the next 2 s)
  4. right handle stream stops: nothing is commanded while one hand is stale
  5. pedal released: the camera stays
Bar coupling (/mtm_<side>/cursor_force, device frame; with tilt 0 the screen force is
haply_to_operator x the device force): no force after the hands translated together (2); left
alone 30 mm toward the right shortens the 200 mm bar, so each hand is pushed back out along it
by 150 x 0.030 / 2 = 2.25 N (2b); after steering (made here by vertical moves, which lengthen
the bar to 215.4 mm) each hand is pulled in along the bar by 150 x 0.0154 / 2 = 1.15 N (3b);
zero force with a stale hand (4) and after the release (5).
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import TransformBroadcaster

from arts_interfaces.msg import ToolTipState, ToolTipTarget

FAILS = []


def check(name, ok, detail):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        FAILS.append(name)


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(r):
    a = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w * (0.5 if a < 1e-9 else a / (2 * math.sin(a)))


class Rig(Node):

    def __init__(self):
        super().__init__('camera_teleop_mock_test')
        self.cur = {s: self.create_publisher(PointStamped, f'/mtm_{s}/cursor', 10)
                    for s in ('left', 'right')}
        self.vel = {s: self.create_publisher(Vector3Stamped, f'/mtm_{s}/cursor_velocity', 10)
                    for s in ('left', 'right')}
        self.joy = self.create_publisher(Joy, '/console/footswitch_teleop', 10)
        # left at screen x -0.1 (left map: x = -h_y), right at +0.1 (right map: x = +h_y)
        self.h = {'left': np.array([0.0, 0.1, 0.1]), 'right': np.array([0.0, 0.1, 0.1])}
        self.stream = {'left': True, 'right': True}
        self.pedals = [0, 0, 0]
        self.tip = None
        self.cmds = 0
        self.force = {'left': None, 'right': None}
        for s in ('left', 'right'):
            self.create_subscription(Vector3Stamped, f'/mtm_{s}/cursor_force',
                                     lambda m, s=s: self.force.__setitem__(
                                         s, np.array([m.vector.x, m.vector.y, m.vector.z])), 10)
        self.create_subscription(ToolTipState, '/ecm/tool_tip_state', self._on_tip, 10)
        self.create_subscription(ToolTipTarget, '/ecm/tool_tip_target',
                                 lambda m: setattr(self, 'cmds', self.cmds + 1), 10)
        self.tfb = TransformBroadcaster(self)
        self.follow = False
        self.create_timer(0.02, self._tick_optical)
        self.create_timer(0.002, self._tick)
        self.create_timer(0.02, self._tick_pedals)

    def _on_tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), quat_to_matrix(o.x, o.y, o.z, o.w))

    def optical_below_tip(self):
        # From now on the optical frame is published 50 mm below the CURRENT tip, looking down
        # base -z, as the real one moves with the scope (cart_tf / the description).
        self.follow = True

    def _tick_optical(self):
        if not self.follow or self.tip is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'ecm_base_link', 'ecm_stereo_optical_frame'
        c = self.tip[0] - np.array([0.0, 0.0, 0.05])
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, c)
        t.transform.rotation.x = 1.0  # Rx(180): optical z = base -z
        t.transform.rotation.w = 0.0
        self.tfb.sendTransform(t)

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for s in ('left', 'right'):
            if not self.stream[s]:
                continue
            m = PointStamped()
            m.header.stamp, m.header.frame_id = now, f'mtm_{s}_base'
            m.point.x, m.point.y, m.point.z = map(float, self.h[s])
            self.cur[s].publish(m)
            v = Vector3Stamped()
            v.header.stamp = now
            self.vel[s].publish(v)

    def _tick_pedals(self):
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.buttons = list(self.pedals)
        self.joy.publish(j)


H = {'left': np.array([0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0]).reshape(3, 3),
     'right': np.array([0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]).reshape(3, 3)}


def screen_force(node, side):
    f = node.force[side]
    return None if f is None else H[side] @ f


def spin_for(node, s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.001)


def move(node, dl, dr, seconds):
    s0 = {k: v.copy() for k, v in node.h.items()}
    n = max(int(seconds / 0.004), 1)
    for k in range(1, n + 1):
        node.h['left'] = s0['left'] + np.asarray(dl) * k / n
        node.h['right'] = s0['right'] + np.asarray(dr) * k / n
        spin_for(node, 0.004)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mapping = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    mapping.write('footswitch_teleop:\n  device: none\n  pedals: {}\n'
                  '  actions: {left: clutch, middle: camera, right: record_toggle}\n')
    mapping.close()
    rclpy.init()
    rig = Rig()
    spin_for(rig, 3.0)
    rig.optical_below_tip()
    node = subprocess.Popen(
        [sys.executable, os.path.join(here, '..', 'scripts', 'camera_teleop.py'),
         '--ros-args', '-p', 'camera_arm:=ecm', '-p', 'mode:=clutch', '-p', 'enable_source:=service',
         '-p', 'goovis_tilt_deg:=0.0', '-p', 'camera_scale:=0.5',
         '-p', 'right_origin_in_left:=[0.0, 0.0, 0.0]',  # the rig's handles share one origin
         '-p', f'mapping_file:={mapping.name}'])
    try:
        spin_for(rig, 3.0)

        print('[1] pedal up, both hands move')
        p, r = rig.tip
        move(rig, [0, -0.02, 0], [0, 0.02, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.tip[0] - p) * 1e3
        check('camera still without the pedal', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')

        print('[2] pedal down, both hands 40 mm screen-right')
        rig.pedals = [0, 1, 0]
        spin_for(rig, 0.5)
        p, r = rig.tip
        move(rig, [0, -0.04, 0], [0, 0.04, 0], 1.0)  # screen x +0.04: left h_y -, right h_y +
        spin_for(rig, 2.0)
        d = (rig.tip[0] - p) * 1e3
        turn = math.degrees(np.linalg.norm(rotvec(rig.tip[1] @ r.T)))
        check('camera +20 mm along base x', abs(d[0] - 20.0) < 1.0 and np.linalg.norm(d[1:]) < 1.0,
              f'{np.round(d, 2)} mm')
        check('orientation kept while translating', turn < 0.3, f'{turn:.2f} deg')
        fl, fr = screen_force(rig, 'left'), screen_force(rig, 'right')
        check('no bar force after translating together', fl is not None and fr is not None and
              np.linalg.norm(fl) < 0.1 and np.linalg.norm(fr) < 0.1, f'left {fl}, right {fr} N')

        print('[2b] left hand alone 30 mm screen-right, then back')
        p, r = rig.tip
        move(rig, [0, -0.03, 0], [0, 0, 0], 0.5)
        spin_for(rig, 1.0)
        d = (rig.tip[0] - p) * 1e3
        check('one hand alone: camera still', np.linalg.norm(d) < 0.5, f'{np.round(d, 2)} mm')
        fl, fr = screen_force(rig, 'left'), screen_force(rig, 'right')
        check('bar shortened: hands pushed apart, 2.25 N each', abs(fl[0] + 2.25) < 0.1 and
              abs(fr[0] - 2.25) < 0.1 and np.linalg.norm(fl[1:]) < 0.1 and
              np.linalg.norm(fr[1:]) < 0.1, f'left {np.round(fl, 2)}, right {np.round(fr, 2)} N')
        move(rig, [0, 0.03, 0], [0, 0, 0], 0.5)
        spin_for(rig, 0.5)

        print('[2c] clutch held during camera control, hands move; clutch released, hands move')
        rig.pedals = [1, 1, 0]
        spin_for(rig, 0.3)
        p, r = rig.tip
        move(rig, [0, -0.03, 0], [0, 0.03, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.tip[0] - p) * 1e3
        check('clutched: camera still', np.linalg.norm(d) < 0.5, f'{np.round(d, 2)} mm')
        check('clutched: no bar force', not rig.force['left'].any() and
              not rig.force['right'].any(), f'{rig.force}')
        rig.pedals = [0, 1, 0]
        spin_for(rig, 0.5)
        d = (rig.tip[0] - p) * 1e3
        check('clutch released: no jump', np.linalg.norm(d) < 0.5, f'{np.round(d, 2)} mm')
        move(rig, [0, -0.02, 0], [0, 0.02, 0], 0.5)
        spin_for(rig, 1.5)
        d = (rig.tip[0] - p) * 1e3
        check('resumed: camera +10 mm along base x', abs(d[0] - 10.0) < 1.0 and
              np.linalg.norm(d[1:]) < 1.0, f'{np.round(d, 2)} mm')

        print('[3a] small twist: left 5 mm down, right 5 mm up (2.9 deg, inside the dead band)')
        p, r = rig.tip
        move(rig, [0, 0, -0.005], [0, 0, 0.005], 0.5)  # h_z = screen y
        spin_for(rig, 1.5)
        w = np.degrees(rotvec(rig.tip[1] @ r.T))
        check('no roll inside the dead band', np.linalg.norm(w) < 0.3, f'{np.round(w, 2)} deg')

        print('[3b] steering on to 21.8 deg (40 mm each)')
        move(rig, [0, 0, -0.035], [0, 0, 0.035], 2.0)
        spin_for(rig, 0.3)
        w_stop = np.degrees(rotvec(rig.tip[1] @ r.T))
        spin_for(rig, 2.0)
        w = np.degrees(rotvec(rig.tip[1] @ r.T))
        d = (rig.tip[0] - p) * 1e3
        want = (math.degrees(math.atan2(0.08, 0.2)) - 5.0) * 1.0
        check(f'scope turned {want:.1f} deg about base -z', abs(w[2] + want) < 0.5 and
              np.linalg.norm(w[:2]) < 0.5, f'{np.round(w, 2)} deg')
        check('tip stayed on the optical axis', np.linalg.norm(d) < 1.0, f'{np.round(d, 2)} mm')
        fr = screen_force(rig, 'right')
        u = np.array([0.2, 0.08, 0.0]) / math.hypot(0.2, 0.08)
        check('bar lengthened: hands pulled in along it, 1.15 N each',
              abs(fr @ u + 1.15) < 0.1 and np.linalg.norm(fr - (fr @ u) * u) < 0.1 and
              np.allclose(screen_force(rig, 'left'), -fr, atol=0.05), f'right {np.round(fr, 2)} N')
        # 0.30-0.31 deg measured twice at dead band 10 / gain 0.5: the ECM controller's tracking
        # lag behind a command at its own 15 deg/s cap (0.3 s after the stop it is still that far
        # behind), not an overrun, which before the rate limit was several degrees.
        check('stops with the hands (no overrun)', abs(w[2] - w_stop[2]) < 0.5,
              f'{w_stop[2]:.2f} deg 0.3 s after the hands stopped, {w[2]:.2f} deg 2 s later')

        print('[4] right handle stream stops')
        rig.stream['right'] = False
        spin_for(rig, 0.3)
        n0 = rig.cmds
        spin_for(rig, 0.5)
        check('nothing commanded with one hand stale', rig.cmds == n0, f'{rig.cmds - n0} commands')
        check('no force with one hand stale', not rig.force['left'].any() and
              not rig.force['right'].any(), f'{rig.force}')
        rig.stream['right'] = True

        print('[5] pedal released, hands move')
        rig.pedals = [0, 0, 0]
        spin_for(rig, 0.5)
        p, r = rig.tip
        move(rig, [0, -0.02, 0], [0, 0.02, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.tip[0] - p) * 1e3
        check('camera stays after release', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')
        check('no force after release', not rig.force['left'].any() and
              not rig.force['right'].any(), f'{rig.force}')

        print('[dup] a second camera_teleop started while this one runs')
        dup = subprocess.Popen(
            [sys.executable, os.path.join(here, '..', 'scripts', 'camera_teleop.py'),
             '--ros-args', '-p', f'mapping_file:={mapping.name}'],
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
        os.unlink(mapping.name)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
