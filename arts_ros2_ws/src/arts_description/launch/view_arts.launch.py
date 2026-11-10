"""View the ARTS patient-cart description in RViz.

  ros2 launch arts_description view_arts.launch.py

Kinematics and visuals only — no ros2_control, no drivers. The three arms are
jogged with joint_state_publisher_gui from a singularity-safe start pose.
Mount transforms are the placeholders in config/mount_transforms.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _zeros_from_yaml(path):
    """joint_state_publisher wants a dict, not a file path."""
    import yaml
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return {str(k): float(v) for k, v in data.items()}


def _launch_setup(context, *args, **kwargs):
    description_file = LaunchConfiguration("description_file").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(context)

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str,
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"zeros": _zeros_from_yaml(initial_positions_file)}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_file],
        ),
    ]


def generate_launch_description():
    pkg = FindPackageShare("arts_description")
    return LaunchDescription([
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([pkg, "urdf", "arts.urdf.xacro"]),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([pkg, "rviz", "view_arts.rviz"]),
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution([pkg, "config", "initial_positions.yaml"]),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
