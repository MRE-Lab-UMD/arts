"""The ESU-TAP board on the ROS graph: its micro-ROS agent and the esu node (decision 18).

  ros2 launch arts_esu esu.launch.py

The agent bridges the board's serial micro-ROS session (/esu/activation_state and the board's
services); the esu node turns that into the /esu contract. The node runs on rmw_fastrtps_cpp,
set here for that process only: the agent is Fast DDS and services do not cross to CycloneDDS,
which the rest of ARTS runs. Topics cross, so everything the node offers is a topic.
Both Fast DDS processes load config/fastdds_esu.xml (a 100 ms writer heartbeat): with the
3 s default, a CycloneDDS subscriber that joins late gets the first new message 3 s late.

device defaults to /dev/arts_esu, the udev symlink from deploy/modules/esu.sh (matched by the
bridge's serial number: the camera sync box has the same CP2102N).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    fake = LaunchConfiguration('fake')
    profile = PathJoinSubstitution([FindPackageShare('arts_esu'), 'config', 'fastdds_esu.xml'])
    fast = {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'FASTRTPS_DEFAULT_PROFILES_FILE': profile}
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/arts_esu',
                              description='Serial port of the ESU-TAP board'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='The firmware\'s fixed baud (board_profile.hpp)'),
        DeclareLaunchArgument('monopolar_arm', default_value='',
                              description='Override: the arm the monopolar output feeds '
                                          '(empty = from tool_info)'),
        DeclareLaunchArgument('bipolar_arm', default_value='',
                              description='Override: the arm the bipolar output feeds'),
        DeclareLaunchArgument('fake', default_value='false',
                              description='true: the fake board instead of agent + hardware'),
        Node(package='micro_ros_agent', executable='micro_ros_agent', name='esu_agent',
             output='screen',
             arguments=['serial', '--dev', device, '-b', LaunchConfiguration('baudrate')],
             additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': profile},
             condition=UnlessCondition(fake)),
        Node(package='arts_esu', executable='fake_esu_tap.py', name='fake_esu_tap',
             output='screen', additional_env=fast,
             condition=IfCondition(fake)),
        Node(package='arts_esu', executable='esu_node.py', name='esu', output='screen',
             additional_env=fast,
             parameters=[{'monopolar_arm': LaunchConfiguration('monopolar_arm'),
                          'bipolar_arm': LaunchConfiguration('bipolar_arm')}]),
    ])

