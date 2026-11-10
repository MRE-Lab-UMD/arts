"""FLIR camera benchmark launch — camera node + MCAP recorder.

Starts one Blackfly S node and a rosbag2 MCAP recorder that captures
the image and camera_info topics.  The recorder starts after a short
delay so the camera has time to initialize.

Run inside the arts-flir:jazzy container (bench scripts volume-mounted):
  ros2 launch /bench/flir_bench.launch.py serial:='24057131' \
      output:='/bench_output/flir_bench'

Stop with SIGINT (Ctrl-C) or `docker stop`.  The run_flir_bench.sh
orchestration script handles duration timing automatically.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERA_PARAMS = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': True,
    'dump_node_map': False,
    'gain_auto': 'Continuous',
    'exposure_auto': 'Continuous',
    'frame_rate_auto': 'Off',
    'frame_rate': 40.0,
    'frame_rate_enable': True,
    'buffer_queue_size': 10,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}


def _launch_setup(context, *args, **kwargs):
    serial = LaunchConfiguration('serial').perform(context)
    output = LaunchConfiguration('output').perform(context)
    mcap_config = LaunchConfiguration('mcap_config').perform(context)

    param_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', 'blackfly_s.yaml']
    ).perform(context)

    camera_node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name='bench_camera',
        parameters=[
            CAMERA_PARAMS,
            {
                'parameter_file': param_file,
                'serial_number': serial,
            },
        ],
    )

    # No -s mcap: MCAP is the default rosbag2 storage plugin in Jazzy.
    record_cmd = ['ros2', 'bag', 'record']
    if mcap_config:
        record_cmd += ['--storage-config-file', mcap_config]
    record_cmd += [
        '-o', output,
        '/bench_camera/image_raw',
        '/bench_camera/camera_info',
    ]

    recorder = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(cmd=record_cmd, output='screen'),
        ],
    )

    return [camera_node, recorder]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial', default_value='24057131',
            description='Camera serial number'),
        DeclareLaunchArgument(
            'output', default_value='/bench_output/flir_bench',
            description='Bag output path'),
        DeclareLaunchArgument(
            'mcap_config', default_value='',
            description='Path to MCAP storage config YAML (empty = defaults)'),
        OpaqueFunction(function=_launch_setup),
    ])
