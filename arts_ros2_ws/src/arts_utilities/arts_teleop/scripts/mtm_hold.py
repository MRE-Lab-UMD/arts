#!/usr/bin/env python3
"""Hold both Inverse3 handles where they are while the operator's head is out of the viewer.

The da Vinci keeps its masters in place while the surgeon's head is out of the console (PI,
2026-11-06); in gravity compensation alone an Inverse3 drifts. While /console/head_sensor
reports no head, or has been silent for head_stale_s (silence is not a head, as in
teleop_follower.py), each handle's position at the start of the hold is sent, at the cursor
rate, as /mtm_<side>/cursor_position_target (device frame); haply_bridge passes it to the Haply
service's set_cursor_position, whose position loop runs inside the service. Nothing is closed
through this PC: the first version rendered a spring here as a cursor force (300 N/m,
5 N s/m), and with the ROS round trip in the loop a small pull grew into a shake on the real
handles (2026-11-06). A head that returns ends the hold: the targets stop and the bridge drops
them after 50 ms, back to gravity compensation. The VerseGrip's orientation has no actuator and
is not held.

Topics: /mtm_<side>/cursor, /console/head_sensor in; /mtm_<side>/cursor_position_target out.
"""
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

SIDES = ('left', 'right')


class MtmHold(Node):

    def __init__(self):
        super().__init__('mtm_hold')
        self.head_stale = self.declare_parameter('head_stale_s', 0.2).value
        self.head_t = None
        self.holding = None
        self.p0 = {s: None for s in SIDES}
        self.pub = {s: self.create_publisher(PointStamped, f'/mtm_{s}/cursor_position_target', 10)
                    for s in SIDES}
        for s in SIDES:
            self.create_subscription(PointStamped, f'/mtm_{s}/cursor',
                                     lambda m, s=s: self._on_cursor(s, m), 10)
        self.create_subscription(Joy, '/console/head_sensor', self._on_head, 10)
        self.get_logger().info(
            f'mtm_hold: handles held (Haply position mode) while the head is out, head stale '
            f'after {self.head_stale} s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_head(self, m):
        if len(m.buttons) > 0 and m.buttons[0]:
            self.head_t = self._now()

    def _head_in(self):
        return self.head_t is not None and self._now() - self.head_t <= self.head_stale

    def _on_cursor(self, side, m):
        hold = not self._head_in()
        if hold != self.holding:
            self.holding = hold
            self.p0 = {s: None for s in SIDES}
            self.get_logger().info('head out: handles held' if hold else 'head in: handles free')
        if not hold:
            return
        if self.p0[side] is None:
            self.p0[side] = (m.point.x, m.point.y, m.point.z)
        out = PointStamped()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), f'mtm_{side}_base'
        out.point.x, out.point.y, out.point.z = self.p0[side]
        self.pub[side].publish(out)


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
    refuse_duplicate('mtm_hold')
    node = MtmHold()
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
