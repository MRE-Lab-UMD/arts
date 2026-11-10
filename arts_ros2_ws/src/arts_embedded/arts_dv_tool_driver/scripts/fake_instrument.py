#!/usr/bin/env python3
"""Fake instrument drive for mock hardware: what disc_drive does at the joint level.

Stands in for disc_drive (with an instrument file) when the arm runs on mock hardware, so the
arm's controller can be tested with the instrument wrist (decision 17, 2026-11-03 amendment):
joint velocity targets on <arm>/tool_joint_target (by short or full name, kept per joint, a
joint older than command_timeout stops) are integrated at rate_hz, held inside the instrument
file's limits, and published on <arm>/tool_joint_states as <arm>_tool_<joint>, the same
names and topics disc_drive uses. No discs, no coupling: the joints are exact. Setting the
parameter `paused` to true stops the state stream (a stalled drive, for the controller's
stale-wrist hold test).

    ros2 run arts_dv_tool_driver fake_instrument.py --ros-args -r __ns:=/psm_left \\
        -p instrument_file:=471006_large_needle_driver.yaml
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeInstrument(Node):

    def __init__(self):
        super().__init__('fake_instrument')
        inst = self.declare_parameter('instrument_file', '471006_large_needle_driver.yaml').value
        self.rate = self.declare_parameter('rate_hz', 333.0).value
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.declare_parameter('paused', False)
        with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config',
                               'instruments', inst)) as f:
            spec = yaml.safe_load(f)
        self.joints = spec['joints']
        lim = spec['limits']
        self.lo = [float(lim.get(j, {}).get('min', 0.0)) for j in self.joints]
        self.hi = [float(lim.get(j, {}).get('max', 0.0)) for j in self.joints]
        prefix = self.get_namespace().strip('/')
        self.full = [f'{prefix}_tool_{j}' if prefix else f'tool_{j}' for j in self.joints]
        self.q = [0.0] * len(self.joints)
        self.v = [0.0] * len(self.joints)
        self.t = [0.0] * len(self.joints)
        self.pub = self.create_publisher(JointState, 'tool_joint_states', 10)
        self.create_subscription(JointState, 'tool_joint_target', self._on_target, 10)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f'fake instrument {spec["ref"]} {spec["name"]}: {self.full}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_target(self, m):
        now = self._now()
        for n, v in zip(m.name, m.velocity):
            for j, (short, full) in enumerate(zip(self.joints, self.full)):
                if n in (short, full):
                    self.v[j] = v
                    self.t[j] = now

    def _tick(self):
        now, dt = self._now(), 1.0 / self.rate
        for j in range(len(self.q)):
            v = self.v[j] if now - self.t[j] < self.timeout else 0.0
            self.q[j] = min(max(self.q[j] + v * dt, self.lo[j]), self.hi[j])
        if self.get_parameter('paused').value:
            return
        m = JointState(name=self.full, position=list(self.q))
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeInstrument()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
