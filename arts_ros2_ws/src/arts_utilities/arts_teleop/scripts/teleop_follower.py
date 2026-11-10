#!/usr/bin/env python3
"""Teleop follower, position only: one MTM drives one arm (decisions 17, 18).

The minimal reference source of decision 17: the tool-tip target is the arm's pose at
engagement plus the scaled, rotated handle displacement since engagement. Orientation is held
at the engagement pose. Published as ONE message per MTM cursor message on
/<arm>/tool_tip_target (arts_interfaces/ToolTipTarget, base frame <arm>_base_link), stamped
with that cursor message's stamp (the bridge's receive time), so a bag can trace every command
back to the handle frame that caused it. The message carries the feedforward too
(velocity_valid): the Haply's own measured cursor velocity, scaled and rotated like the
position, zero along any axis where the target is held at the box edge. The controller uses it
in rate mode and ignores it in position mode. Engagement changes go out on
/<arm>/teleop_state (latched).

Topics: /<mtm>/cursor, /<mtm>/cursor_velocity, /console/footswitch_teleop, /<arm>/status,
/<arm>/tool_tip_state, /<arm>/rcm_state in; /<arm>/tool_tip_target, /<arm>/teleop_state out.
Parameters arm (psm_left) and mtm (mtm_left).

The arm pose taken at engagement is the controller's own /<arm>/tool_tip_state (FK of the
measured joints INCLUDING the tool-tip offset), because the target is a tool-tip pose. It was
the tf flange (tool0) until 2026-11-01, which equals the tip only with a zero offset: with an
instrument mounted the first engagement would have commanded the tip to the flange.

Engagement, from the pedal that config/input_mapping.yaml maps to `clutch` (/console/footswitch_teleop):
  deadman  follow ONLY while the pedal is held; releasing it stops. For first tests: the
           arm can only move while the operator is actively pressing.
  clutch   the surgical convention: follow while ENABLED and pressing the pedal DISCONNECTS
           the handle so the hand can be repositioned. Releasing it re-engages from the new
           hand position (through the alignment gate in orientation mode). ENABLED comes from
           `enable_source`: `service` (~/enable, std_srvs/SetBool: the GUI or a terminal) or
           `head_sensor` (collaborator A's: the GOOVIS head-presence sensor,
           /console/head_sensor, enabled while a head is in and the topic is fresh, within
           head_stale_s; silence or a lost device is "no head").

Each (re-)engagement re-indexes: the handle and arm poses at that instant become the new
origin, so the arm never jumps. Guards: the target stays inside a box of +-max_offset around
the pose at the FIRST engagement of this run; a handle faster than drop_speed (a dropped
handle) disengages until the pedal is pressed again; so does any UR safety stop
(the safety mode in /<arm>/status: protective, safeguard -- the ARM HOLD box -- or emergency
stop), the External Control program stopping while it ran, and any RCM switch
(/<arm>/rcm_state.enabled changing): with the RCM on, the shaft orientation follows from the
tip position, so the orientation held since engagement is stale after either switch. Without the latter,
a hand moved during an ARM HOLD would become a target the arm rushes to on reset, because
the controller is re-activated on resume and follows the next fresh target. A cursor
stream older than
stale_timeout stops publishing, and the controller then holds after its own 0.1 s reference
timeout. Speed is capped again downstream by the controller's Cartesian governor (250 mm/s).

The handle-to-arm map is the product of two MEASURED 3x3 matrices, both row-major:
  haply_to_operator  handle displacement (mtm_<side>_base) -> the operator's frame, x to the
                     operator's right, y up, z toward the operator. Measured per device with the
                     bridge running (2026-10-30). The Hub's handedness setting MIRRORS one
                     device (left: toward = -x, right: toward = +x), so this may be a
                     reflection (det -1); it only has to be orthonormal.
  operator_to_base   the operator's frame -> the arm's base_link, a proper rotation. It depends
                     on where the operator stands relative to the arm, so it is measured at the
                     setup with first_motion_real_arm.py (which way each base axis moves).

ORIENTATION (collaborator A's design, decision 17 2026-11-03 amendment; `orientation: true`, for
an arm with the instrument wrist). Frames: the handle in the Haply base -> the SCREEN (the
operator frame tilted by `goovis_tilt_deg` about x, A's 45 deg for the downward-looking
GOOVIS) -> the camera frame `camera_frame`, an OPTICAL frame (z into the scene, x right, y
down; cart_tf's ecm_stereo_optical_frame), whose screen (x right, y up, z toward the viewer) is it
turned 180 deg about x -- A treats the screen as the camera's view; the pose comes from tf,
the ECM's forward kinematics and the nominal view frame -> the arm base. `display_rotate_180`
(default true) must match arts_display's `rotate_180`: the displayed image is the camera's
turned 180 deg about the view axis, so the screen is the optical frame turned about x, then z
(found at the first camera-frame teleop, 2026-11-05: left/right and up/down both reversed).
With `camera_frame` empty, the screen maps to the base through `operator_to_base` instead (no
ECM needed; mock tests). The stylus orientation (`<mtm>/stylus_state`) is A's: left-multiplied
by a fixed per-hand rotation (A: Rz(+90) right, Rz(-90) left; `stylus_to_haply`), mapped to the
screen, and kept a proper rotation by mirroring one stylus axis when the device's axis map is a
mirror (the Hub's handedness setting makes the right Inverse3's map one, 2026-10-30), then
right-multiplied by `stylus_calibration` (identity until set at the bench). The target is
R_tip = R_off R_stylus, absolute, 1:1, with R_off = R_tip R_stylus^T latched at engagement;
engagement additionally needs the two within `align_gate_deg` (A: 10 deg), so the latched
residual is small. While waiting for alignment the arm holds and the angle is published on
`<arm>/teleop_alignment` (deg) for the overlay. Holding VerseGrip button b ("bring to
stylus", A's helper, which A had on the middle pedal) makes the arm turn the instrument to the
stylus at the current position until the gate opens; release it and teleop engages. Holding
VerseGrip button a ("straighten", A's grasper reset, which A had on button b) brings the jaw
axis back onto the shaft, roll kept, position held, so a session starts from a straight
instrument and a natural hand pose (2026-11-05: aligning to an instrument left bent ~60 deg
forced an awkward grip). Both helpers act only while disengaged. The
pedals stay left = clutch, middle = camera, right = record (PI, 2026-11-03). A's other helper,
straighten, is not ported. Angular feedforward (A's): the stylus orientation in the screen
differenced message to message (spatial rate), a median over `angular_ff_median` samples and a
first-order low-pass at `angular_ff_cutoff_hz` (A: median 7 + 2nd-order 30 Hz on a 1 kHz
resampled stream; here on the VerseGrip's own event stream), rotated like the target
(R_bs R_off) and capped at `angular_ff_max` rad/s; zero once no stylus message has come for
`angular_ff_stale` s (a still pen sends nothing). Without it the wrist followed only at the
controller's tracking gain (2026-11-05: "too slow").

The pedal that config/input_mapping.yaml maps to `camera` (the middle one) always DISENGAGES
this arm while held; the camera itself is moved by BOTH handles in camera_teleop.py (the
clinical scheme). One-handed camera control here (`camera_arm`, default empty = off), from
this follower's own handle: the handle moves the camera arm's tip, position only, orientation held: target = tip at the
press + camera_direction * camera_scale * (handle displacement mapped like the arm's, from the
screen into the camera arm's base), inside +-camera_max_offset of the tip at the first camera
press. The mapping is LATCHED at the press, so it does not chase the moving view; releasing
the pedal stops commanding (the camera controller holds) and the arm re-engages through the
gate in the new view. In clutch mode it needs the enable (head in) too; the drop guard applies.
camera_direction +1: the camera moves with the hand (the image content moves against it).

Bend limit (`max_bend_deg`, 60; 0 = off), RCM off only: the target's jaw axis (tip z) is kept
within that angle of the instrument shaft (tf <arm>_tool0 +y, the shaft at zero wrist angles);
pen rotation past it is dropped and the angular feedforward zeroed. The VerseGrip's ball joint
turns without end, and once the wrist (+-80 deg pitch and yaw) cannot make a rotation the QP
makes it with the UR, sweeping the arm toward its neighbours. Roll about the jaw axis is not
limited (the instrument rolls +-270 deg). With the RCM on the arm pivots at the port and cannot
take over the bend, so the limit is not applied. A bench safety setting like the +-50 mm box,
reported with the experiments, not a surgical workspace.
"""
import collections
import math

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PointStamped, PoseArray, Quaternion, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool
from ur_dashboard_msgs.msg import SafetyMode

from arts_interfaces.msg import (ArmStatus, RcmState, StylusState, TeleopState, ToolTipState,
                                 ToolTipTarget)

PEDALS = ('left', 'middle', 'right')
# SafetyMode values that stop the arm. Others (NORMAL, REDUCED, and whatever mock hardware
# reports) do not latch.
STOP_MODES = {SafetyMode.PROTECTIVE_STOP, SafetyMode.SAFEGUARD_STOP,
              SafetyMode.SYSTEM_EMERGENCY_STOP, SafetyMode.ROBOT_EMERGENCY_STOP,
              SafetyMode.VIOLATION, SafetyMode.FAULT, SafetyMode.AUTOMATIC_MODE_SAFEGUARD_STOP,
              SafetyMode.SYSTEM_THREE_POSITION_ENABLING_STOP}


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s,
                0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotvec(r):
    """Rotation vector (axis * angle, rad) of a rotation matrix; small and moderate angles."""
    a = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w * (0.5 if a < 1e-6 else a / (2 * math.sin(a)))


def angle_deg(r):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2))))


class TeleopFollower(Node):

    def __init__(self):
        super().__init__('teleop_follower')
        p = self.declare_parameter
        self.arm = p('arm', 'psm_left').value
        self.mtm = p('mtm', 'mtm_left').value
        self.base_frame = f'{self.arm}_base_link'
        self.mode = p('mode', 'deadman').value
        self.scale = p('scale', 0.25).value
        self.H = np.array(p('haply_to_operator', list(np.eye(3).ravel())).value).reshape(3, 3)
        self.B = np.array(p('operator_to_base', list(np.eye(3).ravel())).value).reshape(3, 3)
        self.max_offset = p('max_offset', 0.05).value
        self.drop_speed = p('drop_speed', 0.8).value
        self.stale_timeout = p('stale_timeout', 0.05).value
        mapping_file = p('mapping_file', '').value
        self.orientation = p('orientation', False).value
        self.camera_frame = p('camera_frame', '').value
        self.tilt = p('goovis_tilt_deg', 45.0).value
        self.display_rotate_180 = p('display_rotate_180', True).value
        side_rot = 90.0 if self.mtm.endswith('right') else -90.0
        c, s_ = math.cos(math.radians(side_rot)), math.sin(math.radians(side_rot))
        self.S = np.array(p('stylus_to_haply', [c, -s_, 0.0, s_, c, 0.0, 0.0, 0.0, 1.0]).value
                          ).reshape(3, 3)
        self.C = np.array(p('stylus_calibration', list(np.eye(3).ravel())).value).reshape(3, 3)
        self.gate = p('align_gate_deg', 10.0).value
        self.ff_median = p('angular_ff_median', 5).value
        self.ff_cutoff = p('angular_ff_cutoff_hz', 15.0).value
        self.ff_max = p('angular_ff_max', math.radians(90.0)).value
        self.ff_stale = p('angular_ff_stale', 0.1).value
        self.max_bend = math.radians(p('max_bend_deg', 60.0).value)
        if self.mode not in ('deadman', 'clutch'):
            raise ValueError(f'mode must be deadman or clutch, not {self.mode!r}')
        if not np.allclose(self.H @ self.H.T, np.eye(3), atol=1e-6):
            raise ValueError(f'haply_to_operator is not orthonormal: {self.H.tolist()}')
        if not np.allclose(self.B @ self.B.T, np.eye(3), atol=1e-6) or \
                abs(np.linalg.det(self.B) - 1.0) > 1e-6:
            raise ValueError(f'operator_to_base is not a rotation: {self.B.tolist()}')
        # Position (and orientation) map, handle -> screen -> base. In camera mode the screen
        # is tilted by the GOOVIS angle and the base<-camera rotation comes from tf.
        self.M = (rot_x(self.tilt) if self.camera_frame else np.eye(3)) @ self.H
        self.D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(self.M)))])
        self.R_bs = self.B if not self.camera_frame else None  # base <- screen
        self.R = self.B @ self.H
        with open(mapping_file) as f:
            actions = yaml.safe_load(f)['footswitch_teleop']['actions']
        pedal = [k for k, v in actions.items() if v == 'clutch']
        if len(pedal) != 1:
            raise ValueError(f'{mapping_file}: exactly one pedal must map to clutch, got {pedal}')
        self.pedal_index = PEDALS.index(pedal[0])
        cam = [k for k, v in actions.items() if v == 'camera']
        self.camera_arm = p('camera_arm', '').value
        self.camera_index = PEDALS.index(cam[0]) if cam else None
        self.camera_scale = p('camera_scale', 0.5).value
        self.camera_max_offset = p('camera_max_offset', 0.05).value
        self.camera_direction = p('camera_direction', 1.0).value
        self.camera_pedal = False
        self.cam_origin = None    # (handle, camera tip position, orientation, map) at the press
        self.cam_box = None
        self.cam_tip = None       # (stamp s, position, orientation) of the camera arm's tip
        self.bring = False        # VerseGrip button b held: bring the instrument to the stylus
        self.straighten = False   # VerseGrip button a held: jaw axis back onto the shaft
        self.stylus = None        # raw stylus rotation (VerseGrip, service frame)
        self.w_hist = collections.deque(maxlen=self.ff_median)
        self.w_screen = np.zeros(3)   # filtered stylus angular rate, screen frame (rad/s)
        self.w_prev = None            # (stamp s, stylus in screen) of the previous message
        self.r_off = None         # latched at engagement (orientation mode)
        self.tf_buffer = None
        if self.camera_arm or (
                self.orientation and (self.camera_frame or self.max_bend > 0.0)):
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self._tf_listener = TransformListener(self.tf_buffer, self)

        self.pedal = False
        self.enabled = False      # clutch mode only
        self.enable_source = p('enable_source', 'service').value
        if self.enable_source not in ('service', 'head_sensor'):
            raise ValueError(f'enable_source must be service or head_sensor, got {self.enable_source}')
        self.head_stale = p('head_stale_s', 0.2).value
        self.head_t = None        # last /console/head_sensor message with a head present
        self.dropped = False      # latched by the drop guard until the pedal is pressed again
        self.safety_stop = False  # the arm is in a UR safety stop now
        self.held = False         # latched by a safety stop until the pedal is pressed after it
        self.program_running = None
        self.origin = None        # (handle position, arm position, arm orientation) at engagement
        self.box_center = None
        self.last_cursor = None
        self.last_speed = 0.0

        self.tip = None           # (stamp s, position, orientation) from tool_tip_state
        self.rcm_enabled = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(ToolTipTarget, f'/{self.arm}/tool_tip_target', 10)
        self.pub_state = self.create_publisher(TeleopState, f'/{self.arm}/teleop_state', latched)
        if self.camera_arm:
            self.pub_cam = self.create_publisher(
                ToolTipTarget, f'/{self.camera_arm}/tool_tip_target', 10)
            self.create_subscription(
                ToolTipState, f'/{self.camera_arm}/tool_tip_state', self._on_cam_tip, 10)
        self.last_velocity = np.zeros(3)
        self.create_subscription(PointStamped, f'/{self.mtm}/cursor', self._on_cursor, 10)
        self.create_subscription(
            Vector3Stamped, f'/{self.mtm}/cursor_velocity', self._on_velocity, 10)
        self.create_subscription(Joy, '/console/footswitch_teleop', self._on_pedal, 10)
        self.create_subscription(ArmStatus, f'/{self.arm}/status', self._on_status, latched)
        # The collision guard (arts_safety): while it holds, the controllers hold every arm;
        # the follower disengages like on a safety stop, so it cannot jump on the release.
        self.collision_hold = False
        self.create_subscription(Bool, '/arts/collision_hold', self._on_collision_hold, latched)
        self.create_subscription(
            ToolTipState, f'/{self.arm}/tool_tip_state', self._on_tip, 10)
        self.create_subscription(RcmState, f'/{self.arm}/rcm_state', self._on_rcm, 10)
        if self.enable_source == 'service':
            self.create_service(SetBool, '~/enable', self._on_enable)
        else:
            self.create_subscription(Joy, '/console/head_sensor', self._on_head, 10)
        if self.orientation:
            self.create_subscription(StylusState, f'/{self.mtm}/stylus_state', self._on_stylus, 10)
            self.pub_align = self.create_publisher(Float64, f'/{self.arm}/teleop_alignment', 10)
            # For the display's alignment overlay (A's two triads): poses[0] the instrument,
            # poses[1] the stylus, orientations in the screen frame; only while waiting.
            self.pub_align_frames = self.create_publisher(
                PoseArray, f'/{self.arm}/teleop_alignment_frames', 10)
        self.create_timer(0.02, self._watchdog)
        self._publish_state()  # a defined (disengaged) state from the start, latched
        self.get_logger().info(
            f'{self.mtm} -> {self.arm}, mode {self.mode}, scale {self.scale}, '
            f'clutch pedal {pedal[0]}, box +-{self.max_offset * 1e3:.0f} mm, '
            f'haply_to_operator {self.H.tolist()}, operator_to_base {self.B.tolist()}, '
            f'orientation {self.orientation}, camera_frame {self.camera_frame or "(none)"}, '
            f'tilt {self.tilt if self.camera_frame else 0} deg, gate {self.gate} deg')

    def _following(self):
        if self.dropped or self.held:
            return False
        if self.camera_pedal:
            return False
        return self.pedal if self.mode == 'deadman' else (self.enabled and not self.pedal)

    def _camera_active(self):
        return (self.camera_pedal and not self.dropped and
                (self.mode == 'deadman' or self.enabled))

    def _on_cam_tip(self, msg):
        self.cam_tip = (self.get_clock().now().nanoseconds * 1e-9,
                        np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                        msg.pose.orientation)

    def _on_pedal(self, msg):
        if len(msg.buttons) <= self.pedal_index:
            return
        pressed = bool(msg.buttons[self.pedal_index])
        if pressed and self.dropped:
            self.dropped = False
            self.get_logger().info('drop guard cleared by the pedal')
        if (pressed and self.held and not self.safety_stop and not self.collision_hold
                and self.program_running is not False):
            self.held = False
            self.get_logger().info('safety hold cleared by the pedal')
        was = self._following()
        self.pedal = pressed
        if self.camera_index is not None and len(msg.buttons) > self.camera_index:
            cam = bool(msg.buttons[self.camera_index])
            if cam != self.camera_pedal:
                self.camera_pedal = cam
                self.cam_origin = None
                self.get_logger().info(f'camera pedal {"down: disengaged" if cam else "up"}')
        if self._following() != was:
            self._state_changed()

    def _on_enable(self, req, res):
        self._set_enabled(req.data)
        res.success, res.message = True, f'enabled={self.enabled}'
        return res

    def _set_enabled(self, value):
        if value == self.enabled:
            return
        was = self._following()
        self.enabled = value
        self.get_logger().info(f'teleop {"enabled" if value else "disabled"} ({self.enable_source})')
        if self._following() != was:
            self._state_changed()

    def _on_head(self, msg):
        present = len(msg.buttons) > 0 and bool(msg.buttons[0])
        if present:
            self.head_t = self.get_clock().now()
        self._set_enabled(present)

    def _hold(self, why):
        was = self._following()
        self.held = True
        self.origin = None
        self.get_logger().warn(f'{why}: disengaged until the pedal is pressed after it clears')
        if was:
            self._state_changed()

    def _on_collision_hold(self, msg):
        self.collision_hold = msg.data
        if msg.data:
            self._hold('collision guard hold')

    def _on_status(self, msg):
        self.safety_stop = msg.safety_mode in STOP_MODES
        if self.safety_stop:
            self._hold(f'UR safety mode {msg.safety_mode}')
        if self.program_running and not msg.program_running:
            self._hold('External Control program stopped')
        self.program_running = msg.program_running

    def _on_tip(self, msg):
        p = msg.pose.position
        self.tip = (self.get_clock().now().nanoseconds * 1e-9, np.array([p.x, p.y, p.z]),
                    msg.pose.orientation)

    def _on_rcm(self, msg):
        if self.rcm_enabled is not None and msg.enabled != self.rcm_enabled:
            self._hold(f'RCM switched {"on" if msg.enabled else "off"}')
        self.rcm_enabled = msg.enabled

    def _on_velocity(self, msg):
        v = msg.vector
        self.last_velocity = np.array([v.x, v.y, v.z])
        self.last_speed = float(np.linalg.norm(self.last_velocity))

    def _on_stylus(self, msg):
        o = msg.orientation
        self.stylus = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.bring = len(msg.buttons) > 1 and bool(msg.buttons[1])  # a, b, c
        self.straighten = len(msg.buttons) > 0 and bool(msg.buttons[0])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r = self._stylus_screen()
        if self.w_prev is not None and 0.0 < t - self.w_prev[0] <= self.ff_stale:
            dt = t - self.w_prev[0]
            self.w_hist.append(rotvec(r @ self.w_prev[1].T) / dt)
            w = np.median(np.array(self.w_hist), axis=0)
            k = 1.0 - math.exp(-2.0 * math.pi * self.ff_cutoff * dt)
            self.w_screen = self.w_screen + k * (w - self.w_screen)
        else:
            self.w_hist.clear()
            self.w_screen = np.zeros(3)
        self.w_prev = (t, r)

    def _angular_ff(self, r_bs):
        """Stylus angular rate as the target's, in the base frame, or zero when stale."""
        if (self.w_prev is None or self.r_off is None or
                self.get_clock().now().nanoseconds * 1e-9 - self.w_prev[0] > self.ff_stale):
            return np.zeros(3)
        w = r_bs @ self.r_off @ self.w_screen
        n = float(np.linalg.norm(w))
        return w if n <= self.ff_max else w * (self.ff_max / n)

    def _base_from_screen(self):
        """base <- screen rotation: operator_to_base, or tf base <- camera (A's camera frame)."""
        if not self.camera_frame:
            return self.R_bs
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame,
                                                rclpy.time.Time())
        except Exception as e:  # noqa: BLE001 -- any tf failure means no mapping yet
            self.get_logger().warn(f'no tf {self.base_frame} <- {self.camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        q = t.transform.rotation
        # The camera frame is optical; the screen is it turned 180 deg about x (and about z
        # when the display shows the image rotated 180 deg).
        r_cs = rot_x(180.0) @ (rot_z(180.0) if self.display_rotate_180 else np.eye(3))
        return quat_to_matrix(q.x, q.y, q.z, q.w) @ r_cs

    def _stylus_screen(self):
        """The stylus orientation in the screen frame, a proper rotation (see the docstring)."""
        if self.stylus is None:
            # Found at the first real E9 run (2026-11-05): a silent VerseGrip left the follower
            # waiting with no message; button b, the gate and the jaw all need the stylus.
            self.get_logger().warn(f'no /{self.mtm}/stylus_state yet: the VerseGrip is not '
                                   'sending (asleep or off?); orientation teleop waits',
                                   throttle_duration_sec=3.0)
            return None
        return self.M @ (self.S @ self.stylus) @ self.D @ self.C

    def _state_changed(self):
        self.origin = None  # re-index on the next cursor message
        self.r_off = None
        self.get_logger().info('following' if self._following() else 'disengaged')
        self._publish_state()

    def _publish_state(self):
        # Engaged = following AND, in orientation mode, through the alignment gate. Reporting
        # the wish to follow hid the overlay's triads exactly while the operator aligned (the
        # display draws them only while not engaged) and let the jaw follow before alignment;
        # seen in clutch mode, where head-in + pedal-up is that wish (2026-11-05).
        m = TeleopState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.clutch_engaged = self._following() and (not self.orientation or self.r_off is not None)
        m.scaling_factor = 1.0 / self.scale
        m.active_arm = self.arm
        self.pub_state.publish(m)

    def _arm_pose(self):
        if self.tip is None or self.get_clock().now().nanoseconds * 1e-9 - self.tip[0] > 0.2:
            self.get_logger().warn(f'no fresh /{self.arm}/tool_tip_state: no command',
                                   throttle_duration_sec=2.0)
            return None
        return self.tip[1], self.tip[2]

    def _on_cursor(self, msg):
        self.last_cursor = self.get_clock().now()
        if self.camera_arm and self._camera_active():
            self._camera_step(msg)
            return
        if not self._following():
            return
        if self.last_speed > self.drop_speed:
            self.dropped = True
            self.origin = None
            self.get_logger().warn(f'handle at {self.last_speed:.2f} m/s > {self.drop_speed}: '
                                   'dropped? disengaged until the pedal is pressed again')
            return
        h = np.array([msg.point.x, msg.point.y, msg.point.z])
        r_bs = None
        if self.orientation:
            r_bs = self._base_from_screen()
            r_ss = self._stylus_screen()
            if r_bs is None or r_ss is None:
                return
            self.R = r_bs @ self.M
        if self.origin is None:
            arm = self._arm_pose()
            if arm is None:
                return
            if self.orientation:
                # A's gate: engage only with the stylus within align_gate_deg of the tip,
                # both in the screen frame; latch the small residual as R_off.
                o = arm[1]
                r_tip_screen = r_bs.T @ quat_to_matrix(o.x, o.y, o.z, o.w)
                r_off = r_tip_screen @ r_ss.T
                ang = angle_deg(r_off)
                self.pub_align.publish(Float64(data=ang))
                if ang > self.gate:
                    self.get_logger().info(
                        f'align the stylus with the instrument: {ang:.1f} deg > {self.gate} deg',
                        throttle_duration_sec=2.0)
                    return
                self.r_off = r_off
                self.get_logger().info(f'aligned ({ang:.1f} deg): following')
                self._publish_state()
            self.origin = (h, arm[0], arm[1])
            if self.box_center is None:
                self.box_center = arm[0]
            return
        h0, p0, q0 = self.origin
        bent = False
        if self.orientation and self.r_off is not None:
            r_t, bent = self._bend_limit(r_bs @ (self.r_off @ r_ss))
            q0 = Quaternion()
            (q0.x, q0.y, q0.z, q0.w) = map(float, matrix_to_quat(r_t))
        unclamped = p0 + self.scale * (self.R @ (h - h0))
        target = np.clip(unclamped, self.box_center - self.max_offset,
                         self.box_center + self.max_offset)
        v_ff = self.scale * (self.R @ self.last_velocity)
        v_ff[target != unclamped] = 0.0
        out = ToolTipTarget()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, target)
        out.pose.orientation = q0
        out.velocity.linear.x, out.velocity.linear.y, out.velocity.linear.z = map(float, v_ff)
        if self.orientation and self.r_off is not None and not bent:
            (out.velocity.angular.x, out.velocity.angular.y,
             out.velocity.angular.z) = map(float, self._angular_ff(r_bs))
        out.velocity_valid = True
        self.pub.publish(out)

    def _bend_limit(self, r_t):
        """The target orientation with its jaw axis kept within max_bend of the shaft (RCM off);
        returns (rotation, whether it was limited). See the docstring."""
        if self.max_bend <= 0.0 or self.rcm_enabled:
            return r_t, False
        r, bent = self._bend_to(r_t, self.max_bend)
        if bent:
            self.get_logger().warn(f'jaw axis past the {math.degrees(self.max_bend):.0f} deg bend '
                                   'limit: held at it (RCM off)', throttle_duration_sec=2.0)
        return r, bent

    def _bend_to(self, r_t, limit):
        """r_t turned so its jaw axis (z) is within `limit` rad of the shaft (tf tool0 +y),
        about the axis normal to both; (rotation, whether it was turned)."""
        try:
            q = self.tf_buffer.lookup_transform(self.base_frame, f'{self.arm}_tool0',
                                                rclpy.time.Time()).transform.rotation
        except Exception as e:  # noqa: BLE001 -- no shaft, no command past a known limit
            self.get_logger().warn(f'bend limit: no tf {self.arm}_tool0: {e}',
                                   throttle_duration_sec=2.0)
            return r_t, False
        shaft = quat_to_matrix(q.x, q.y, q.z, q.w)[:, 1]
        z = r_t[:, 2]
        bend = math.acos(max(-1.0, min(1.0, float(z @ shaft))))
        if bend <= limit:
            return r_t, False
        axis = np.cross(z, shaft)
        n = float(np.linalg.norm(axis))
        if n < 1e-9:
            return r_t, False
        axis /= n
        a = bend - limit
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        back = np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)  # Rodrigues, toward shaft
        return back @ r_t, True

    def _camera_step(self, msg):
        """Camera pedal held: the handle moves the camera arm's tip (see the docstring)."""
        if self.last_speed > self.drop_speed:
            self.dropped = True
            self.cam_origin = None
            self.get_logger().warn(f'handle at {self.last_speed:.2f} m/s: dropped? camera '
                                   'stopped until the clutch pedal is pressed again')
            return
        h = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.cam_origin is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.cam_tip is None or now - self.cam_tip[0] > 0.2:
                self.get_logger().warn(f'no fresh /{self.camera_arm}/tool_tip_state: camera '
                                       'not commanded', throttle_duration_sec=2.0)
                return
            r_bs = self._base_from_screen() if self.camera_frame else self.R_bs
            try:
                q = self.tf_buffer.lookup_transform(f'{self.camera_arm}_base_link', self.base_frame,
                                                    rclpy.time.Time()).transform.rotation
            except Exception as e:  # noqa: BLE001 -- no camera base, no camera command
                self.get_logger().warn(f'camera control: no tf {self.camera_arm}_base_link <- '
                                       f'{self.base_frame}: {e}', throttle_duration_sec=2.0)
                return
            if r_bs is None:
                return
            r_map = quat_to_matrix(q.x, q.y, q.z, q.w) @ r_bs @ self.M
            self.cam_origin = (h, self.cam_tip[1], self.cam_tip[2], r_map)
            if self.cam_box is None:
                self.cam_box = self.cam_tip[1]
            return
        h0, p0, q0, r_map = self.cam_origin
        k = self.camera_direction * self.camera_scale
        unclamped = p0 + k * (r_map @ (h - h0))
        target = np.clip(unclamped, self.cam_box - self.camera_max_offset,
                         self.cam_box + self.camera_max_offset)
        v_ff = k * (r_map @ self.last_velocity)
        v_ff[target != unclamped] = 0.0
        out = ToolTipTarget()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = f'{self.camera_arm}_base_link'
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, target)
        out.pose.orientation = q0
        out.velocity.linear.x, out.velocity.linear.y, out.velocity.linear.z = map(float, v_ff)
        out.velocity_valid = True
        self.pub_cam.publish(out)

    def _align_assist(self):
        """Disengaged, orientation mode: publish the alignment angle; with VerseGrip button b
        held, turn the instrument toward the stylus at the current position (A's "bring to
        stylus"), zero feedforward, until the gate opens."""
        r_bs = self._base_from_screen()
        r_ss = self._stylus_screen()
        arm = self._arm_pose() if r_bs is not None else None
        if arm is None:
            return
        o = arm[1]
        r_tip_screen = r_bs.T @ quat_to_matrix(o.x, o.y, o.z, o.w)
        if r_ss is None:
            # No stylus: the overlay still shows the instrument, and NaN = "NO STYLUS".
            r_ss = r_tip_screen
            self.pub_align.publish(Float64(data=float('nan')))
        else:
            self.pub_align.publish(Float64(data=angle_deg(r_tip_screen @ r_ss.T)))
        frames = PoseArray()
        frames.header.stamp = self.get_clock().now().to_msg()
        frames.header.frame_id = 'screen'
        for r in (r_tip_screen, r_ss):
            pose = Pose()
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = map(float, matrix_to_quat(r))
            frames.poses.append(pose)
        self.pub_align_frames.publish(frames)
        if self.held or self.dropped or self.stylus is None:
            return
        if self.bring:
            r_target = self._bend_limit(r_bs @ r_ss)[0]
        elif self.straighten:
            r_target = self._bend_to(quat_to_matrix(o.x, o.y, o.z, o.w), 0.0)[0]
            self.get_logger().info('straighten: jaw axis onto the shaft', throttle_duration_sec=2.0)
        else:
            return
        out = ToolTipTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, arm[0])
        (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
         out.pose.orientation.w) = map(float, matrix_to_quat(r_target))
        out.velocity_valid = False
        self.pub.publish(out)

    def _watchdog(self):
        if (self.enable_source == 'head_sensor' and self.enabled and (
                self.head_t is None or
                (self.get_clock().now() - self.head_t).nanoseconds * 1e-9 > self.head_stale)):
            self.get_logger().warn('head sensor silent: disabled')
            self._set_enabled(False)
        if self.orientation and not (self._following() and self.r_off is not None):
            self._align_assist()
        if self.last_cursor is None or not self._following():
            return
        age = (self.get_clock().now() - self.last_cursor).nanoseconds * 1e-9
        if age > self.stale_timeout:
            self.get_logger().warn(f'no Haply cursor for {age * 1e3:.0f} ms; not commanding',
                                   throttle_duration_sec=2.0)
            self.origin = None  # re-index when the stream comes back


def main():
    rclpy.init()
    node = TeleopFollower()
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
