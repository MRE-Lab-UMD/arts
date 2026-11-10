"""<arm>/tool_info (latched): which instrument is on this arm (decision 18).

Started by ur_arm.launch.py in the arm's namespace with the `tool` parameter, a key of
config/instruments.yaml. An unknown key is an error at start-up, not a silent "none": the esu
node routes energy from this message.
"""
import hashlib
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from arts_interfaces.msg import ToolInfo

ENERGY = {'none': ToolInfo.ENERGY_NONE, 'monopolar': ToolInfo.ENERGY_MONOPOLAR,
          'bipolar': ToolInfo.ENERGY_BIPOLAR}


def main():
    rclpy.init()
    node = Node('tool_info')
    key = node.declare_parameter('tool', 'none').value
    with open(os.path.join(get_package_share_directory('arts_driver'), 'config',
                           'instruments.yaml')) as f:
        catalog = yaml.safe_load(f)
    if key not in catalog:
        raise SystemExit(f'tool:={key} is not in config/instruments.yaml {sorted(catalog)}')
    entry = catalog[key]
    msg = ToolInfo(name=key, family=entry.get('family', ''),
                   part_number=entry.get('part_number', ''),
                   energy_type=ENERGY[entry.get('energy_type', 'none')],
                   joint_names=entry.get('joint_names', []),
                   actuator_names=entry.get('actuator_names', []),
                   kinematics_file=entry.get('kinematics_file', ''))
    if msg.kinematics_file and not os.path.isabs(msg.kinematics_file):
        # A bare name is a file in arts_dv_tool_driver's config/instruments/.
        msg.kinematics_file = os.path.join(get_package_share_directory('arts_dv_tool_driver'),
                                           'config', 'instruments', msg.kinematics_file)
    if msg.kinematics_file:
        with open(msg.kinematics_file, 'rb') as f:
            msg.kinematics_sha256 = hashlib.sha256(f.read()).hexdigest()
    msg.header.stamp = node.get_clock().now().to_msg()
    pub = node.create_publisher(ToolInfo, 'tool_info', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    pub.publish(msg)
    node.get_logger().info(f'tool_info: {key} (energy {entry.get("energy_type", "none")})')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
