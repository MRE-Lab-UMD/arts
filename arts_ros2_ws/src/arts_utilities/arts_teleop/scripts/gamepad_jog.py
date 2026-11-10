#!/usr/bin/env python3
"""Gamepad jog: an Xbox controller as a reference source for one arm (decision 17).

Collaborator A's layout (the gamepad tool used on these arms), as a decision-17 source: it
publishes <arm>/tool_tip_target like the teleop follower does, so the servo controller and
its RCM stay the only thing that moves the arm. Differences from A's: the speeds and the
button/axis indices are parameters, the mode bumper is a DEADMAN (motion only while held;
releasing it stops at once), and buttons act on their press edge.

  Back / Start / Guide   select psm_left / psm_right / ecm (no motion while switching)
  hold RB                jog the selected arm:
    RCM on  (from <arm>/rcm_state): left stick = pivot the shaft about the RCM point,
            LT = insert, RT = retract, right stick X = roll about the shaft
    RCM off: left stick = move the tip across the shaft (tip-frame x/y),
            LT / RT = along the shaft, right stick X = roll about the shaft,
            right stick Y = pitch about the tip's x, d-pad X = yaw about the tip's y (both
            about the tip point, at wrist_angular_velocity; with the instrument wrist the
            controller shares them between the wrist and the arm)
  hold LB                disc jog on a PSM (A's tool-jog mode 0, for checking the adapter's
                         discs and for engaging an instrument by hand, as A does): disc 1 =
                         left stick X, disc 2 = left stick Y, disc 4 = right stick Y, disc 5 =
                         d-pad X, each at disc_jog_rpm (10) full scale; disc 3 held at 0
                         as in A's code
                         (jog_disc_3:=true puts it on right stick X). Published to
                         <arm>/tool_actuator_target for arts_dv_tool_driver's disc_drive;
                         zero velocities on release.
    while holding LB, A's tool-jog buttons: A = disc mode (above, the default), B =
                         instrument-joint mode, X = set the zero here (disc_drive set_zero,
                         A's manual zero). Joint mode (A's mode 1; needs disc_drive started
                         with the instrument file): roll = left stick X, wrist pitch = right
                         stick X, wrist yaw = d-pad X, each at tool_jog_rad_s full scale (A:
                         pi rad/s); RT = the unitless jaw command on <arm>/tool_jaw_target
                         (disc_drive): released -10 (full grip), the first third of the
                         travel -10 -> 0 (grip easing to just closed, the share the Haply
                         lever's Hall 0..10 has), the rest 0 -> 90 (fully open). Roll,
                         pitch and yaw are published to <arm>/tool_joint_target; disc_drive
                         applies the coupling and slows each joint near its limits.
                         Refused while another node publishes tool_joint_target (the arm's
                         servo controller with the wrist, active from the arm launch until
                         the pendant program first connects): the two would interleave.

Speeds (A's values): tool arms 0.1 rad/s, 20 mm/s; the ECM 0.2 rad/s, 50 mm/s. The target is
integrated from the arm's MEASURED tip pose at the moment RB is pressed and is published at
rate_hz with the jog velocity as feedforward; on release one last target with zero velocity
is sent at the integrated pose. A gamepad message older than stale_s while RB is held is a
release (A's 0.5 s jog timeout).

Inputs: /console/gamepad (sensor_msgs/Joy from joy_node, arts_bringup gamepad:=true),
<arm>/tool_tip_state, <arm>/rcm_state. Do not run it together with the teleop follower on the
same arm: both would publish <arm>/tool_tip_target.

Under the RCM the controller's default rcm.orientation_task is "none" (the da Vinci X
adapter's shaft cannot roll without the whole arm), so the roll input has no effect there
until the instrument roll disc is in.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from arts_interfaces.msg import RcmState, ToolTipState, ToolTipTarget

ARMS = ('psm_left', 'psm_right', 'ecm')


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def exp_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * kx + (1 - math.cos(th)) * kx @ kx


class GamepadJog(Node):

    def __init__(self):
        super().__init__('gamepad_jog')
        p = self.declare_parameter
        self.arm = p('arm', 'psm_left').value
        self.rate = p('rate_hz', 100.0).value
        self.stale_s = p('stale_s', 0.5).value
        self.fresh_s = p('state_fresh_s', 0.2).value
        self.ang = {a: p(f'{a}.max_angular_velocity', 0.2 if a == 'ecm' else 0.1).value
                    for a in ARMS}
        self.lin = {a: p(f'{a}.max_linear_velocity', 0.05 if a == 'ecm' else 0.02).value
                    for a in ARMS}
        # Indices: joy_node (SDL) on an Xbox 360 pad; triggers rest at +1.0, pressed -1.0.
        self.ax = {k: p(f'axes.{k}', v).value for k, v in
                   dict(left_x=0, left_y=1, lt=2, right_x=3, right_y=4, rt=5).items()}
        self.bt = {k: p(f'buttons.{k}', v).value for k, v in
                   dict(a=0, b=1, x=2, lb=4, rb=5, back=6, start=7, guide=8).items()}
        self.deadzone = p('deadzone', 0.1).value
        self.wrist_ang = p('wrist_angular_velocity', 0.3).value
        self.disc_rad_s = p('disc_jog_rpm', 10.0).value * 2 * math.pi / 60.0
        self.jog_disc_3 = p('jog_disc_3', False).value
        self.tool_rad_s = p('tool_jog_rad_s', math.pi).value
        self.rt_grip_share = p('rt_grip_share', 1.0 / 3.0).value
        self.tool_mode = 0   # A's jog_tool_mode: 0 = discs, 1 = instrument joints
        self.jaw = {}
        self.jaw_t = {}
        self.joy = None
        self.joy_t = None
        self.prev_buttons = []
        self.tip = {}
        self.tip_t = {}
        self.rcm = {}
        self.pubs = {}
        for a in ARMS:
            self.create_subscription(ToolTipState, f'/{a}/tool_tip_state',
                                     lambda m, a=a: self._on_tip(a, m), 10)
            self.create_subscription(RcmState, f'/{a}/rcm_state',
                                     lambda m, a=a: self.rcm.__setitem__(a, m), 10)
            self.pubs[a] = self.create_publisher(ToolTipTarget, f'/{a}/tool_tip_target', 10)
        self.create_subscription(Joy, '/console/gamepad', self._on_joy, 10)
        self.disc_pubs = {a: self.create_publisher(JointState, f'/{a}/tool_actuator_target', 10)
                          for a in ('psm_left', 'psm_right')}
        self.jaw_pubs = {a: self.create_publisher(Float64, f'/{a}/tool_jaw_target', 10)
                         for a in ('psm_left', 'psm_right')}
        # Last tool_joint_target message per arm from ANY node: activity, not registration, tells
        # whether something else commands the instrument (a deactivated controller keeps its
        # publisher registered; 2026-11-05).
        self.joint_seen = {}
        for a in self.disc_pubs:
            self.create_subscription(JointState, f'/{a}/tool_joint_target',
                                     lambda m, a=a: self.joint_seen.__setitem__(a, self._now()), 10)
        self.joint_pubs = {a: self.create_publisher(JointState, f'/{a}/tool_joint_target', 10)
                           for a in ('psm_left', 'psm_right')}
        self.zero_clients = {a: self.create_client(Trigger, f'/{a}/disc_drive/set_zero')
                             for a in ('psm_left', 'psm_right')}
        for a in ('psm_left', 'psm_right'):
            self.create_subscription(JointState, f'/{a}/tool_joint_states',
                                     lambda m, a=a: self._on_tool_joints(a, m), 10)
        self.disc_jogging = False
        self.jogging = False
        self.p = None
        self.r = None
        self.lb_warned = False
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f'gamepad jog: arm {self.arm}; Back/Start/Guide select '
                               'psm_left/psm_right/ecm, hold RB to move')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tip(self, arm, m):
        self.tip[arm] = m
        self.tip_t[arm] = self._now()

    def _on_tool_joints(self, arm, m):
        for n, q in zip(m.name, m.position):
            if n.endswith('tool_jaw'):
                self.jaw[arm] = q
                self.jaw_t[arm] = self._now()

    def _pressed(self, name):
        i = self.bt[name]
        b = self.joy.buttons
        return i < len(b) and b[i] == 1 and not (i < len(self.prev_buttons) and self.prev_buttons[i])

    def _held(self, name):
        i = self.bt[name]
        return self.joy is not None and i < len(self.joy.buttons) and self.joy.buttons[i] == 1

    def _axis(self, name):
        i = self.ax[name]
        v = self.joy.axes[i] if i < len(self.joy.axes) else 0.0
        return 0.0 if abs(v) < self.deadzone else v

    def _trigger(self, name):
        # 0 released .. 1 fully pressed (rest +1.0 -> 0, pressed -1.0 -> 1)
        i = self.ax[name]
        v = self.joy.axes[i] if i < len(self.joy.axes) else 1.0
        return min(max((1.0 - v) / 2.0, 0.0), 1.0)

    def _on_joy(self, m):
        self.joy = m
        self.joy_t = self._now()
        for name, arm in (('back', 'psm_left'), ('start', 'psm_right'), ('guide', 'ecm')):
            if self._pressed(name) and arm != self.arm:
                if self.jogging:
                    self._stop('arm switched')
                if self.disc_jogging and self.arm in self.disc_pubs:
                    self._tool_zero()
                    self.disc_jogging = False
                self.arm = arm
                self.get_logger().info(f'selected {arm}')
        if self._held('lb') and self.arm in self.disc_pubs:
            for name, mode in (('a', 0), ('b', 1)):
                if self._pressed(name) and mode != self.tool_mode:
                    self._tool_zero()
                    self.tool_mode = mode
                    self.get_logger().info(
                        f'tool jog mode {mode} ({"discs" if mode == 0 else "instrument joints"})')
            if self._pressed('x'):
                c = self.zero_clients[self.arm]
                if c.service_is_ready():
                    c.call_async(Trigger.Request()).add_done_callback(
                        lambda f: self.get_logger().info(f'set_zero: {f.result().message}'))
                else:
                    self.get_logger().warn(f'/{self.arm}/disc_drive/set_zero is not available')
        if self._pressed('lb') and self.arm not in self.disc_pubs and not self.lb_warned:
            self.get_logger().warn('LB (disc jog): the ECM has no instrument discs; ignored')
            self.lb_warned = True
        self.prev_buttons = list(m.buttons)

    def _publish(self, v_lin, v_ang):
        m = ToolTipTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f'{self.arm}_base_link'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, self.p)
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = map(float, matrix_to_quat(self.r))
        m.velocity.linear.x, m.velocity.linear.y, m.velocity.linear.z = map(float, v_lin)
        m.velocity.angular.x, m.velocity.angular.y, m.velocity.angular.z = map(float, v_ang)
        m.velocity_valid = True
        self.pubs[self.arm].publish(m)

    def _stop(self, why):
        self._publish(np.zeros(3), np.zeros(3))
        self.jogging = False
        self.get_logger().info(f'jog stopped ({why})')

    def _tool_zero(self):
        # Zero velocity on the active tool topic (mode switch, arm switch, release).
        if self.disc_jogging and self.arm in self.disc_pubs:
            if self.tool_mode == 0:
                self.disc_pubs[self.arm].publish(JointState(
                    name=[f'disc_{i}' for i in range(1, 6)], velocity=[0.0] * 5))
            else:
                self.joint_pubs[self.arm].publish(JointState(
                    name=['roll', 'wrist_pitch', 'wrist_yaw'], velocity=[0.0] * 3))

    def jaw_command(self, rt):
        # RT 0 (released) .. 1 (pressed) -> -10 (full grip) .. 0 (just closed) .. 90 (open).
        s = self.rt_grip_share
        if rt < s:
            return -10.0 + 10.0 * rt / s
        return 90.0 * min((rt - s) / (1.0 - s), 1.0)

    def _joint_tick(self, lb):
        m = JointState(name=['roll', 'wrist_pitch', 'wrist_yaw'])
        m.header.stamp = self.get_clock().now().to_msg()
        if lb:
            hat_x = self.joy.axes[6] if len(self.joy.axes) > 6 else 0.0
            m.velocity = [self.tool_rad_s * self._axis('left_x'),
                          self.tool_rad_s * self._axis('right_x'),
                          self.tool_rad_s * hat_x]
            self.jaw_pubs[self.arm].publish(Float64(data=self.jaw_command(self._trigger('rt'))))
        else:
            m.velocity = [0.0] * 3
        self.joint_pubs[self.arm].publish(m)

    def _disc_tick(self, stale):
        lb = self._held('lb') and not stale and self.arm in self.disc_pubs
        if not lb and not self.disc_jogging:
            return
        if self.tool_mode == 1:
            if lb and not self.disc_jogging:
                # Another node commanding the instrument joints (the arm's servo controller is
                # active from the arm launch until the pendant program first connects) would
                # interleave its 500 Hz hold with this jog: it moved, but slowly (2026-11-05).
                seen = self.joint_seen.get(self.arm)
                if seen is not None and self._now() - seen < 0.2:
                    others = sorted({f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in
                                     self.get_publishers_info_by_topic(
                                         f'/{self.arm}/tool_joint_target')
                                     if i.node_name != self.get_name()})
                    self.get_logger().warn(
                        f'instrument jog refused: /{self.arm}/tool_joint_target is being '
                        f'commanded (publishers: {", ".join(others) or "?"}; the arm controller '
                        'is active: stop it, or play then stop the pendant program)',
                        throttle_duration_sec=2.0)
                    return
            self._joint_tick(lb)
            if lb != self.disc_jogging:
                self.get_logger().info(f'instrument jog {"on" if lb else "off"} ({self.arm})')
            self.disc_jogging = lb
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [f'disc_{i}' for i in range(1, 6)]
        if lb:
            hat_x = self.joy.axes[6] if len(self.joy.axes) > 6 else 0.0
            d3 = self._axis('right_x') if self.jog_disc_3 else 0.0
            m.velocity = [self.disc_rad_s * v for v in (
                self._axis('left_x'), self._axis('left_y'), d3, self._axis('right_y'), hat_x)]
        else:
            m.velocity = [0.0] * 5
        self.disc_pubs[self.arm].publish(m)
        if lb != self.disc_jogging:
            self.get_logger().info(f'disc jog {"on" if lb else "off"} ({self.arm})')
        self.disc_jogging = lb

    def _tick(self):
        now = self._now()
        stale = self.joy_t is None or now - self.joy_t > self.stale_s
        self._disc_tick(stale)
        rb = self._held('rb') and not stale
        if not rb:
            if self.jogging:
                self._stop('gamepad stale' if stale and self._held('rb') else 'RB released')
            return
        if not self.jogging:
            t = self.tip.get(self.arm)
            if t is None or now - self.tip_t[self.arm] > self.fresh_s:
                self.get_logger().warn(f'no fresh /{self.arm}/tool_tip_state; not moving',
                                       throttle_duration_sec=2.0)
                return
            pp, o = t.pose.position, t.pose.orientation
            self.p = np.array([pp.x, pp.y, pp.z])
            self.r = quat_to_matrix(o.x, o.y, o.z, o.w)
            self.jogging = True
            self.get_logger().info(f'jogging {self.arm}')

        dt = 1.0 / self.rate
        ang, lin = self.ang[self.arm], self.lin[self.arm]
        ex, ey, s = self.r[:, 0], self.r[:, 1], self.r[:, 2]  # tip frame; z is the shaft
        insert = lin * (self._trigger('lt') - self._trigger('rt'))
        roll = -ang * self._axis('right_x')
        rcm = self.rcm.get(self.arm)
        if rcm is not None and rcm.enabled and math.isfinite(rcm.point.x):
            # A's arm jog: the left stick turns the shaft about axes normal to it, through
            # the RCM point; the tip follows on the sphere of the current depth.
            w = ang * (-self._axis('left_y') * ex - self._axis('left_x') * ey) + roll * s
            c = np.array([rcm.point.x, rcm.point.y, rcm.point.z])
            v = np.cross(w, self.p - c) + insert * s
        else:
            v = lin * (-self._axis('left_x') * ex + self._axis('left_y') * ey) + insert * s
            hat_x = self.joy.axes[6] if len(self.joy.axes) > 6 else 0.0
            w = roll * s + self.wrist_ang * (self._axis('right_y') * ex - hat_x * ey)
        self.p = self.p + v * dt
        self.r = exp_so3(w * dt) @ self.r
        self._publish(v, w)


def main():
    # SIGINT as KeyboardInterrupt: rclpy's own handler shuts the context down under spin()
    # and raised from inside the executor (seen on every Ctrl-C in the mock test).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GamepadJog()
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
