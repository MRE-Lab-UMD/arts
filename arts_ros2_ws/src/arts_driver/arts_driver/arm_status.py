"""<arm>/status: one ArmStatus message per arm (decision 18), latched, published on change.

The UR driver reports an arm's condition on four separate topics, under controller names a
user should not need to know. This node, started in the arm's namespace by ur_arm.launch.py,
merges them with whether the ARTS servo controller is active:

  io_and_status_controller/safety_mode            ur_dashboard_msgs/SafetyMode (latched)
  io_and_status_controller/robot_mode             ur_dashboard_msgs/RobotMode (latched)
  io_and_status_controller/robot_program_running  std_msgs/Bool (latched)
  speed_scaling_state_broadcaster/speed_scaling   std_msgs/Float64
  controller_manager/activity                     controller_manager_msgs/ControllerManagerActivity

All names are relative, so the namespace places them. The UR broadcaster reports speed
scaling in PERCENT (100.0 on mock hardware, 2026-11-01); ArmStatus carries it as 0..1. It is
continuous, so it triggers a publish only when it changes by more than 1 %.
"""
import rclpy
from controller_manager_msgs.msg import ControllerManagerActivity
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from arts_interfaces.msg import ArmStatus

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
SERVO = 'servo_controller'
ACTIVE = 3  # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE


class ArmStatusNode(Node):

    def __init__(self):
        super().__init__('arm_status')
        self.msg = ArmStatus(model=self.declare_parameter('model', '').value)
        self.pub = self.create_publisher(ArmStatus, 'status', LATCHED)
        status = 'io_and_status_controller'
        self.create_subscription(SafetyMode, f'{status}/safety_mode',
                                 lambda m: self._set('safety_mode', m.mode), LATCHED)
        self.create_subscription(RobotMode, f'{status}/robot_mode',
                                 lambda m: self._set('robot_mode', m.mode), LATCHED)
        self.create_subscription(Bool, f'{status}/robot_program_running',
                                 lambda m: self._set('program_running', m.data), LATCHED)
        self.create_subscription(Float64, 'speed_scaling_state_broadcaster/speed_scaling',
                                 self._on_scaling, 10)
        self.create_subscription(ControllerManagerActivity, 'controller_manager/activity',
                                 self._on_activity, LATCHED)
        self._publish()

    def _set(self, field, value):
        if getattr(self.msg, field) != value:
            setattr(self.msg, field, value)
            self._publish()

    def _on_scaling(self, m):
        scaling = m.data / 100.0
        if abs(scaling - self.msg.speed_scaling) > 0.01:
            self.msg.speed_scaling = scaling
            self._publish()

    def _on_activity(self, m):
        active = any(c.name == SERVO and c.state.id == ACTIVE for c in m.controllers)
        self._set('controller_active', active)

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = ArmStatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
