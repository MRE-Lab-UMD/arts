"""micro-ROS agent for the ARTS camera sync box.

The ESP32 publishes /ecm/stereo_sync/pulse (arts_interfaces/CameraSyncPulse; /arts_camera_sync/pulse
with firmware built before decision 18, 2026-11-01); the agent is
what bridges its serial session onto the DDS graph. Nothing appears on the ROS graph until
this is running, so an absent topic usually means an absent agent.

  ros2 launch arts_camera_sync camera_sync.launch.py

The board is a plain CP2102/CH340-class USB serial device, so /dev/ttyUSB* is whatever was
plugged in first. deploy/modules/camera_sync.sh installs a udev rule giving it the stable
/dev/arts_camera_sync, which is the default here; pass device:=/dev/ttyUSB0 to bypass it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/arts_camera_sync',
            description='Serial port of the ESP32 sync box (udev symlink; '
                        'deploy/modules/camera_sync.sh installs the rule)'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Must match CONFIG_MICRO_ROS_ESP_UART_BAUDRATE in the firmware'),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='camera_sync_agent',
            output='screen',
            arguments=['serial', '--dev', device, '-b', baudrate],
        ),
    ])
