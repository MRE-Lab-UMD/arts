#!/usr/bin/env python3
"""Jaw follower: the VerseGrip's Hall-sensor grip lever drives one instrument's jaw.

Collaborator A's grasp path (haply_node, teleop_node, instrument_arm_node) with ARTS's grip
(PI, 2026-11-05). The Hall value h becomes the unitless jaw command on
/<arm>/tool_jaw_target, the same scale for every instrument and input (disc_drive):
  h >= hall_closed (10)  opening: 90 x clamp((h - hall_closed) / (hall_open - hall_closed))
                         (A's 10..29, about 5 % of the opening per Hall step)
  h <  hall_closed       grip: -10 x clamp((hall_closed - h) / (hall_closed - hall_grip_full))
                         with hall_grip_full 9, so 10 = just closed and the released lever
                         (7-9 in the 2026-11-02 bags, squeezed 6) = -10, full grip
disc_drive drives the jaw toward it at the instrument file's jaw_gain and caps the grip current
(1000 at -10). The lever has only ~1-3 Hall steps below 10, so the grip is light or full.

Published at rate_hz on /<arm>/tool_jaw_target (the jaw only), so the arm's controller can
drive roll, pitch and yaw at the same time (decision 17, 2026-11-03 amendment). Do not run it with the gamepad's instrument mode on the same arm. The bridge publishes the
pen only when a value changes, so a steady grip sends nothing: the last grip value is used
while the bridge's /<mtm>/cursor stream (500 Hz) is fresh; a stale bridge stops the jaw.

gate:
  teleop  follow only while /<arm>/teleop_state says clutch_engaged (the teleop follower);
          otherwise hold: keep sending the command the lever had at the disengage (a grip
          stays a grip), as A holds the jaw while the clutch is out
  always  follow whenever the bridge is live: for checking the lever with no arm running

Topics: /<mtm>/stylus_state, /<mtm>/cursor, /<arm>/tool_joint_states, /<arm>/teleop_state in;
/<arm>/tool_jaw_target out (std_msgs/Float64). Parameters arm (psm_left), mtm (mtm_left),
instrument_file (a file in arts_dv_tool_driver/config/instruments/; only to refuse an
instrument without a jaw), gate (teleop), hall_closed (10), hall_open (29), hall_grip_full (9),
rate_hz (100), stale_s (0.5).
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from arts_interfaces.msg import StylusState, TeleopState

class JawFollower(Node):

    def __init__(self):
        super().__init__('jaw_follower')
        p = self.declare_parameter
        self.arm = p('arm', 'psm_left').value
        mtm = p('mtm', 'mtm_left').value
        inst = p('instrument_file', '').value
        self.gate = p('gate', 'teleop').value
        self.h0 = p('hall_closed', 10.0).value
        self.h1 = p('hall_open', 29.0).value
        self.hg = p('hall_grip_full', 9.0).value
        rate = p('rate_hz', 100.0).value
        self.stale_s = p('stale_s', 0.5).value
        if self.gate not in ('teleop', 'always'):
            raise SystemExit(f'gate:={self.gate}: teleop | always')
        if not inst:
            raise SystemExit('instrument_file is required (config/instruments/ in '
                             'arts_dv_tool_driver)')
        with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config',
                               'instruments', inst)) as f:
            spec = yaml.safe_load(f)
        self.jaw_max = float(spec['limits'].get('jaw', {}).get('max', 0.0))
        if self.jaw_max <= 0.0:
            raise SystemExit(f'{inst}: the instrument has no jaw')
        self.grip = None
        self.grip_range = [None, None]
        self.cursor_t = None
        self.jaw = None
        self.jaw_t = None
        self.engaged = False
        self.following = False
        self.held = None   # the command latched at the last disengage
        self.create_subscription(StylusState, f'/{mtm}/stylus_state', self._on_stylus, 10)
        self.create_subscription(PointStamped, f'/{mtm}/cursor',
                                 lambda m: setattr(self, 'cursor_t', self._now()), 10)
        self.create_subscription(JointState, f'/{self.arm}/tool_joint_states', self._on_joints, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TeleopState, f'/{self.arm}/teleop_state',
                                 lambda m: setattr(self, 'engaged', m.clutch_engaged), latched)
        self.pub = self.create_publisher(Float64, f'/{self.arm}/tool_jaw_target', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'jaw follower: {mtm} grip -> /{self.arm}/tool_jaw_target ({spec["ref"]} '
            f'{spec["name"]}), gate {self.gate}, hall {self.hg:g} full grip, {self.h0:g} closed, '
            f'{self.h1:g} open')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_stylus(self, m):
        self.grip = m.grip
        lo, hi = self.grip_range
        self.grip_range = [m.grip if lo is None else min(lo, m.grip),
                           m.grip if hi is None else max(hi, m.grip)]

    def _on_joints(self, m):
        for n, q in zip(m.name, m.position):
            if n.endswith('tool_jaw'):
                self.jaw, self.jaw_t = q, self._now()

    def _report(self):
        if self.grip is not None:
            lo, hi = self.grip_range
            self.get_logger().info(f'grip {self.grip:g} (seen {lo:g}..{hi:g}), jaw '
                                   f'{self.jaw if self.jaw is None else round(self.jaw, 3)}')

    def command(self, h):
        if h >= self.h0:
            return 90.0 * min(max((h - self.h0) / (self.h1 - self.h0), 0.0), 1.0)
        return -10.0 * min(max((self.h0 - h) / (self.h0 - self.hg), 0.0), 1.0)

    def _tick(self):
        now = self._now()
        live = (self.cursor_t is not None and now - self.cursor_t < self.stale_s and
                self.grip is not None and self.jaw_t is not None and
                now - self.jaw_t < self.stale_s)
        follow = live and (self.gate == 'always' or self.engaged)
        if follow != self.following:
            self.get_logger().info('jaw following' if follow else 'jaw holding')
            self.following = follow
        if follow:
            self.held = self.command(self.grip)
        elif not live or self.held is None:
            return  # silent: disc_drive stops the jaw after its command timeout
        self.pub.publish(Float64(data=self.held))


def main():
    rclpy.init()
    node = JawFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
