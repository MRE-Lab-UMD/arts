#!/usr/bin/env python3
"""Two-handed camera control: both MTMs move the camera arm while the camera pedal is held.

The da Vinci scheme (2026-11-05; collaborator A's camera control was one handle,
translation only, rotation disabled). While the pedal that config/input_mapping.yaml maps to
`camera` (the middle one) is held, and the operator is enabled (clutch mode: head in), the two
handles drive the camera arm's tip; each tool arm's teleop_follower disengages on the same
pedal. All of it in the camera's own view (the screen), latched at the press so the mapping
does not chase the moving view:

  translation  the motion BOTH hands share, x camera_scale, into the camera arm's base; per
               screen axis, zero unless both hands moved the same way since the press, else
               the smaller of the two (2026-11-05: the mean let one hand alone move the
               camera, at half its motion; dVRK and collaborator B average too but tie the
               handles together with a spring force instead, see the note at the end); target
               inside +-camera_max_offset of the tip at the press. Steering (hands opposite)
               therefore rolls without translating.
  clutch       the clutch pedal held during camera control (clutch mode) stops the camera and
               the bar force; released, control resumes from the new hand positions.
  roll         the line between the hands turned in the screen plane (a steering wheel):
               the scope turns about its OPTICAL axis through the optical centre, so the view
               spins in place; within +-max_roll_deg of the press. Two hands never move
               exactly together, so the twist passes a dead band (roll_deadband_deg) and a
               gain (roll_gain) first, and the commanded roll moves toward that at most
               roll_rate_deg_s, the camera controller's own angular cap, so the command never
               runs ahead of the scope and it stops when the hands stop (2026-11-05: 1:1 and
               unlimited, the view kept turning and overran).

camera_direction +1: the camera moves WITH the hands (the image moves against them); -1
(default) drags the image instead (A's sign for translation); roll has its own roll_direction. Position and roll are commanded
together as one tool_tip_target (orientation absolute, no feedforward); the controller's tip
caps bound the speed. A handle faster than drop_speed, a stale handle (either), the pedal
released or the enable lost stop commanding (the camera controller holds after its 0.1 s
reference timeout); a dropped handle needs a fresh camera-pedal press.

Mapping per hand: handle -> operator (haply_to_operator_<side>, MEASURED per device; the
right Inverse3 no longer mirrored since Hub 1.5) -> screen (tilted goovis_tilt_deg about x) -> the
optical frame (camera_frame; turned 180 deg about x, and about z when the display rotates the
image, display_rotate_180, as in teleop_follower.py) -> the camera arm's base (tf).

Topics: /mtm_left/cursor, /mtm_right/cursor (+ cursor_velocity), /console/footswitch_teleop,
/console/head_sensor (enable_source head_sensor), /<camera_arm>/tool_tip_state in;
/<camera_arm>/tool_tip_target, /<camera_arm>/camera_teleop_state (std_msgs/Bool, latched) out.

Coupling (2026-11-05, re-derived from dVRK's mtsTeleOperationECM): while camera control is ON,
each handle is pulled toward its end of a bar through the CURRENT midpoint of the hands and along
their CURRENT left-right direction, at the length latched at the press: force per hand
coupling_stiffness x (goal - hand) (dVRK 150 N/m) plus coupling_damping on the hands' relative
speed along the bar, capped at coupling_force_max, sent as /mtm_<side>/cursor_force (device
frame; haply_bridge sends it only while fresh and caps it again). It holds the bar's length, not
its attitude: hands moving together or steering feel no force; only a change in their distance
is resisted, so the handles feel like the two ends of one rigid bar. Collaborator B instead
springs the two displacements toward each other, which resists every differential motion (B has
no roll; it would fight steering). Off (pedal up, head out, a stale hand, a drop) sends zero.
The bar needs the hands' TRUE positions, so right_origin_in_left (operator frame: the right
device's origin from the left's, m) is added to the right handle; measured by touching the two
handle tips together (both then at one point). Translation is unaffected by it; the steering
angle is.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from arts_interfaces.msg import ToolTipState, ToolTipTarget
from teleop_follower import PEDALS, matrix_to_quat, quat_to_matrix, rot_x, rot_z

SIDES = ('left', 'right')


def axis_angle(axis, a):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


class CameraTeleop(Node):

    def __init__(self):
        super().__init__('camera_teleop')
        p = self.declare_parameter
        self.arm = p('camera_arm', 'ecm').value
        self.base = f'{self.arm}_base_link'
        self.camera_frame = p('camera_frame', 'ecm_stereo_optical_frame').value
        self.mode = p('mode', 'clutch').value
        self.enable_source = p('enable_source', 'head_sensor').value
        self.head_stale = p('head_stale_s', 0.2).value
        self.scale = p('camera_scale', 0.5).value
        self.max_offset = p('camera_max_offset', 0.05).value
        self.max_roll = math.radians(p('max_roll_deg', 45.0).value)
        self.roll_deadband = math.radians(p('roll_deadband_deg', 5.0).value)
        self.roll_gain = p('roll_gain', 1.0).value
        self.roll_rate = math.radians(p('roll_rate_deg_s', 15.0).value)
        self.roll_cmd = 0.0
        self.roll_t = None
        # Both set by the PI at the console. Translation -1 (2026-11-06; +1 the day before felt
        # reversed). Roll -1 (2026-11-05, confirmed 2026-11-06: +1 felt reversed), i.e. the
        # image turns with the hands.
        self.direction = p('camera_direction', -1.0).value
        self.roll_direction = p('roll_direction', -1.0).value
        self.tilt = p('goovis_tilt_deg', 30.0).value
        self.rotate_180 = p('display_rotate_180', True).value
        self.drop_speed = p('drop_speed', 0.8).value
        self.k_bar = p('coupling_stiffness', 150.0).value
        self.b_bar = p('coupling_damping', 10.0).value
        self.f_max = p('coupling_force_max', 2.5).value
        # Measured 2026-11-05 at the console: handles held level side by side with the VerseGrip
        # ball joints 15 cm apart; the devices then read right - left = [-0.4738, -0.0182,
        # -0.0120] m (operator frame), so the right origin sits 0.15 + 0.4738 m to the right.
        # Valid while neither Inverse3 is moved on the console.
        self.offset = {'left': np.zeros(3),
                       'right': np.array(p('right_origin_in_left', [0.6238, 0.0182, 0.0120]).value)}
        self.forcing = False
        self.stale = p('stale_timeout', 0.05).value
        self.H = {s: np.array(p(f'haply_to_operator_{s}', d).value).reshape(3, 3) for s, d in (
            ('left', [0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0]),
            ('right', [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]))}  # see teleop.yaml (2026-11-05)
        with open(p('mapping_file', '').value) as f:
            actions = yaml.safe_load(f)['footswitch_teleop']['actions']
        cam = [k for k, v in actions.items() if v == 'camera']
        if len(cam) != 1:
            raise ValueError(f'exactly one pedal must map to camera, got {cam}')
        self.pedal_index = PEDALS.index(cam[0])
        clutch = [k for k, v in actions.items() if v == 'clutch']
        self.clutch_index = PEDALS.index(clutch[0]) if len(clutch) == 1 else None
        self.clutched = False
        # screen <- handle, per side; the screen is x right, y up, z toward the operator
        self.M = {s: rot_x(self.tilt) @ self.H[s] for s in SIDES}

        self.h = {s: None for s in SIDES}      # (time s, position)
        self.v = {s: np.zeros(3) for s in SIDES}
        self.pedal = False
        self.head_t = None
        self.tip = None
        self.origin = None
        self.box = None
        self.dropped = False
        self.active = None
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(ToolTipTarget, f'/{self.arm}/tool_tip_target', 10)
        self.pub_state = self.create_publisher(Bool, f'/{self.arm}/camera_teleop_state', latched)
        self.pub_force = {s: self.create_publisher(Vector3Stamped, f'/mtm_{s}/cursor_force', 10)
                          for s in SIDES}
        for s in SIDES:
            self.create_subscription(PointStamped, f'/mtm_{s}/cursor',
                                     lambda m, s=s: self._on_cursor(s, m), 10)
            self.create_subscription(Vector3Stamped, f'/mtm_{s}/cursor_velocity',
                                     lambda m, s=s: self._on_vel(s, m), 10)
        self.create_subscription(Joy, '/console/footswitch_teleop', self._on_pedal, 10)
        if self.mode == 'clutch' and self.enable_source == 'head_sensor':
            self.create_subscription(Joy, '/console/head_sensor', self._on_head, 10)
        self.create_subscription(ToolTipState, f'/{self.arm}/tool_tip_state', self._on_tip, 10)
        # The collision guard (arts_safety): the controllers hold every arm; the camera stops
        # like on a dropped handle, until the camera pedal is pressed again after the release.
        self.collision_hold = False
        self.create_subscription(Bool, '/arts/collision_hold', self._on_collision_hold, latched)
        self._set_active(False)
        self.get_logger().info(
            f'camera_teleop: both MTMs -> {self.arm} on the {cam[0]} pedal, scale {self.scale}, '
            f'box +-{self.max_offset * 1e3:.0f} mm, roll +-{math.degrees(self.max_roll):.0f} deg '
            f'(dead band {math.degrees(self.roll_deadband):.0f}, gain {self.roll_gain}, '
            f'direction {self.roll_direction:+.0f}, '
            f'{math.degrees(self.roll_rate):.0f} deg/s), '
            f'direction {self.direction:+.0f}, tilt {self.tilt} deg, {self.mode}/{self.enable_source}, '
            f'bar coupling {self.k_bar} N/m, {self.b_bar} N s/m, cap {self.f_max} N, '
            f'right origin {self.offset["right"].tolist()} m')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_vel(self, side, m):
        self.v[side] = np.array([m.vector.x, m.vector.y, m.vector.z])

    def _on_tip(self, m):
        self.tip = (self._now(), np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                    quat_to_matrix(m.pose.orientation.x, m.pose.orientation.y,
                                   m.pose.orientation.z, m.pose.orientation.w))

    def _on_head(self, m):
        if len(m.buttons) > 0 and m.buttons[0]:
            self.head_t = self._now()

    def _on_collision_hold(self, m):
        self.collision_hold = m.data
        if m.data:
            self.dropped = True
            self.get_logger().warn('collision guard hold: camera stopped until the camera pedal '
                                   'is pressed again after the release')

    def _on_pedal(self, m):
        # The clutch inside camera control (dVRK's ECM teleop, as on the da Vinci; clutch mode
        # only, where that pedal IS a clutch): held, the camera stops and the bar force drops,
        # so the hands can be repositioned; released, control resumes from the new hand
        # positions (re-latched, no jump).
        if self.mode == 'clutch' and self.clutch_index is not None:
            self.clutched = len(m.buttons) > self.clutch_index and bool(m.buttons[self.clutch_index])
        if len(m.buttons) <= self.pedal_index:
            return
        pressed = bool(m.buttons[self.pedal_index])
        if pressed and not self.pedal and not self.collision_hold:
            self.dropped = False
            self.origin = None
        self.pedal = pressed

    def _enabled(self):
        if self.mode != 'clutch' or self.enable_source != 'head_sensor':
            return True
        return self.head_t is not None and self._now() - self.head_t <= self.head_stale

    def _set_active(self, a):
        if a != self.active:
            self.active = a
            self.pub_state.publish(Bool(data=a))
            self.get_logger().info(f'camera control {"ON (both hands)" if a else "off"}')
        if not a:
            self.origin = None
            self._send_forces(None)

    def _screen(self, side):
        return rot_x(self.tilt) @ (self.H[side] @ self.h[side][1] + self.offset[side])

    def _send_forces(self, f):
        # f: screen-frame force per side, or None for zero (sent once per release)
        if f is None and not self.forcing:
            return
        self.forcing = f is not None
        stamp = self.get_clock().now().to_msg()
        for s in SIDES:
            v = np.zeros(3) if f is None else self.M[s].T @ f[s]
            m = Vector3Stamped()
            m.header.stamp, m.header.frame_id = stamp, f'mtm_{s}_base'
            m.vector.x, m.vector.y, m.vector.z = map(float, v)
            self.pub_force[s].publish(m)

    def _coupling(self):
        d = self._screen('right') - self._screen('left')
        n = np.linalg.norm(d)
        if n < 1e-3:
            self._send_forces(None)
            return
        u = d / n
        v_rel = self.M['right'] @ self.v['right'] - self.M['left'] @ self.v['left']
        # right goal - right hand = (length at the press - length now) / 2 along the bar
        f = (self.k_bar * (self.origin['w'] - n) / 2.0 - self.b_bar * float(v_rel @ u) / 2.0) * u
        nf = np.linalg.norm(f)
        if nf > self.f_max:
            f = f * (self.f_max / nf)
        self._send_forces({'right': f, 'left': -f})

    def _latch(self):
        now = self._now()
        if self.tip is None or now - self.tip[0] > 0.2:
            self.get_logger().warn(f'no fresh /{self.arm}/tool_tip_state', throttle_duration_sec=2.0)
            return False
        try:
            t = self.tf.lookup_transform(self.base, self.camera_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001 -- no camera pose, no camera command
            self.get_logger().warn(f'no tf {self.base} <- {self.camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return False
        q, tr = t.transform.rotation, t.transform.translation
        r_bo = quat_to_matrix(q.x, q.y, q.z, q.w)
        r_os = rot_x(180.0) @ (rot_z(180.0) if self.rotate_180 else np.eye(3))
        d = self._screen('right') - self._screen('left')
        self.origin = dict(
            left=self._screen('left'), right=self._screen('right'),
            theta=math.atan2(d[1], d[0]), w=float(np.linalg.norm(d)), p=self.tip[1], r=self.tip[2],
            centre=np.array([tr.x, tr.y, tr.z]), axis=r_bo[:, 2], r_bs=r_bo @ r_os)
        self.roll_cmd, self.roll_t = 0.0, self._now()
        if self.box is None:
            self.box = self.tip[1]
        return True

    def _on_cursor(self, side, m):
        self.h[side] = (self._now(), np.array([m.point.x, m.point.y, m.point.z]))
        if side != 'left':
            return
        now = self._now()
        fresh = all(self.h[s] is not None and now - self.h[s][0] <= self.stale for s in SIDES)
        if self.pedal and fresh and max(np.linalg.norm(self.v[s]) for s in SIDES) > self.drop_speed:
            self.dropped = True
            self.get_logger().warn('handle too fast: dropped? camera stopped until the camera '
                                   'pedal is pressed again')
        go = self.pedal and fresh and not self.dropped and not self.clutched and self._enabled()
        self._set_active(go)
        if not go:
            return
        if self.origin is None:
            self._latch()
            return
        o = self.origin
        k = self.direction * self.scale
        dl, dr = self._screen('left') - o['left'], self._screen('right') - o['right']
        common = np.where(dl * dr > 0.0, np.sign(dl) * np.minimum(np.abs(dl), np.abs(dr)), 0.0)
        dp = k * (o['r_bs'] @ common)
        d = self._screen('right') - self._screen('left')
        dth = math.atan2(d[1], d[0]) - o['theta']
        dth = math.atan2(math.sin(dth), math.cos(dth))
        want = math.copysign(max(0.0, abs(dth) - self.roll_deadband) * self.roll_gain, dth)
        want = max(-self.max_roll, min(self.max_roll, want))
        step = self.roll_rate * max(0.0, now - self.roll_t)
        self.roll_t = now
        self.roll_cmd += max(-step, min(step, want - self.roll_cmd))
        dth = self.roll_cmd
        # roll_direction +1: the scope turns WITH the hands, about the screen's z (toward the
        # operator), which is the optical axis reversed; -1 (default) the image does.
        rot = axis_angle(o['axis'], -self.roll_direction * dth)
        p = o['centre'] + rot @ (o['p'] - o['centre']) + dp
        p = np.clip(p, self.box - self.max_offset, self.box + self.max_offset)
        out = ToolTipTarget()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.base
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p)
        (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
         out.pose.orientation.w) = map(float, matrix_to_quat(rot @ o['r']))
        out.velocity_valid = False
        self.pub.publish(out)
        self._coupling()


def refuse_duplicate(name, wait_s=1.5):
    """Exit before acting if a node called `name` already runs. Two copies of a node that
    commands the camera arm or the handles fight each other: 2026-11-06 a forgotten second
    camera_teleop kept moving the camera through the clutch and its bar force, latched at
    another moment, shook the handles against the first one's. Only the newcomer checks, so the
    running copy keeps going."""
    probe = rclpy.create_node(f'{name}_probe')
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.05)
    found = [n for n, _ in probe.get_node_names_and_namespaces() if n == name]
    probe.destroy_node()
    if found:
        raise SystemExit(f'{name}: another {name} is already running; refusing to start')


def main():
    rclpy.init()
    refuse_duplicate('camera_teleop')
    node = CameraTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
