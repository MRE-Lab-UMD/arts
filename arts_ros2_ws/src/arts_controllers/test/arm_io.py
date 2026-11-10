"""Names every ARTS arm test and bench script uses (decision 18), in one place.

The arm under test is ARTS_ARM (default psm_left): its topics live under /<arm>, its frames
and joints carry the <arm>_ prefix. A script that moves an arm publishes ONE message,
arts_interfaces/ToolTipTarget, on /<arm>/tool_tip_target.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from arts_interfaces.msg import ToolTipTarget  # noqa: F401  (re-exported for the scripts)

ARM = os.environ.get('ARTS_ARM', 'psm_left')
BASE_FRAME = f'{ARM}_base_link'
TOOL_FRAME = f'{ARM}_tool0'
TOOL_TIP_FRAME = f'{ARM}_tool_tip'   # the controller's tip, on /tf_static
TARGET_TOPIC = f'/{ARM}/tool_tip_target'
JOINT_STATES = f'/{ARM}/joint_states'
TOOL_TIP_STATE = f'/{ARM}/tool_tip_state'
JOINT_COMMAND = f'/{ARM}/joint_command'
STATUS = f'/{ARM}/status'


def servo_param(*keys):
    """A value from the installed config/ur5e_servo.yaml, e.g. servo_param('cartesian',
    'max_linear_velocity'), so a test never disagrees with the controller it tests."""
    path = get_package_share_directory('arts_controllers') + '/config/ur5e_servo.yaml'
    with open(path) as f:
        node = yaml.safe_load(f)['servo_controller']['ros__parameters']
    for k in keys:
        node = node[k]
    return node


def target(stamp, position, orientation, velocity=None):
    """A ToolTipTarget in the arm's base frame. position: xyz; orientation: a
    geometry_msgs Quaternion or (x, y, z, w); velocity: optional linear xyz (m/s)."""
    m = ToolTipTarget()
    m.header.stamp = stamp
    m.header.frame_id = BASE_FRAME
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, position)
    if hasattr(orientation, 'w'):
        m.pose.orientation = orientation
    else:
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
         m.pose.orientation.w) = map(float, orientation)
    if velocity is not None:
        m.velocity.linear.x, m.velocity.linear.y, m.velocity.linear.z = map(float, velocity)
        m.velocity_valid = True
    return m
