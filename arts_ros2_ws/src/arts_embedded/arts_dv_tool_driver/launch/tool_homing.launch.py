"""Automatic instrument homing for one PSM (the adapter designer's method; decision 17 tool module).

    ros2 launch arts_dv_tool_driver tool_homing.launch.py arm:=psm_right tool:=forceps
    ros2 launch arts_dv_tool_driver tool_homing.launch.py arm:=psm_right tool:=forceps verify:=true
    ros2 launch arts_dv_tool_driver tool_homing.launch.py arm:=psm_left tool:=needledriver \
        calibrate:=true config_dir:=$HOME/arts/arts_ros2_ws/src/arts_embedded/arts_dv_tool_driver/config/homing

On start the five discs go to the adapter's start positions (config/homing/adapter_<serial>.txt),
BEFORE the instrument goes on. Then: put the instrument on, press A on the gamepad. Roll, pitch
and the jaws home by motor current; Y = stop (torque off). A live gamepad is the deadman for the
jaw moves. On COMPLETE the home counts go to $ARTS_LOCAL/tool_homing_logs/home_<serial>.yaml;
Ctrl+C, then start disc_drive with zero_from_homing:=true to use them as the joint zero.

The bus comes from config/disc_buses.yaml (port + baud), the adapter's identity from its FTDI
serial in that port. tool:= names config/homing/<tool>.txt (forceps, scissors, hook,
needledriver); each tool runs only on the adapters the designer validated it on (TOOL_ADAPTERS,
his GUI's TOOL_ARMS). calibrate:=true is his learn-a-tool mode: it rewrites the tool file and its
calibration/<tool>_history.csv in the config dir, so point config_dir:= at the SOURCE tree (the
default is the installed copy, which a rebuild overwrites). Do not run it while disc_drive holds
the same bus. gamepad:=false when arts_bringup already runs the gamepad
(its gamepad_jog then sees the same buttons).
"""
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The designer's GUI tables (homing_gui.py, 2026-11-04 evening), keyed by adapter serial
# (his psm1 = FTBEO5G1, psm2 = FTBEQBRA). Scissors: his psm2 pitch range is shorter than the file
# expects, so not offered there.
TOOL_ADAPTERS = {'forceps': {'FTBEO5G1', 'FTBEQBRA'}, 'scissors': {'FTBEO5G1'},
                 'hook': {'FTBEO5G1', 'FTBEQBRA'}, 'needledriver': {'FTBEO5G1', 'FTBEQBRA'}}
TOOL_SLOW_JAWS = {'needledriver'}


def _setup(context, *args, **kwargs):
    arm = LaunchConfiguration('arm').perform(context)
    tool = LaunchConfiguration('tool').perform(context)
    calibrate = LaunchConfiguration('calibrate').perform(context).lower() == 'true'
    config_dir = LaunchConfiguration('config_dir').perform(context)
    share = get_package_share_directory('arts_dv_tool_driver')
    with open(os.path.join(share, 'config', 'disc_buses.yaml')) as f:
        buses = yaml.safe_load(f)
    if arm not in buses:
        raise RuntimeError(f'arm:={arm} has no bus in disc_buses.yaml ({sorted(buses)})')
    port = buses[arm]['port']
    m = re.search(r'Serial_Converter_([A-Za-z0-9]+)-', port)
    if not m:
        raise RuntimeError(f'cannot read the FTDI serial from {port}')
    serial = m.group(1)
    if tool not in TOOL_ADAPTERS:
        raise RuntimeError(f'tool:={tool} has no homing file ({sorted(TOOL_ADAPTERS)})')
    if serial not in TOOL_ADAPTERS[tool] and not calibrate:
        raise RuntimeError(f'tool:={tool} is not validated on adapter {serial} ({arm}); '
                           f'validated on {sorted(TOOL_ADAPTERS[tool])}')
    params = {'adapter': serial, 'tool': tool,
              'baud_rate': int(buses[arm].get('baud_rate', 2000000)),
              'calibrate': calibrate, 'slow_jaws': tool in TOOL_SLOW_JAWS,
              'jaw.verify': LaunchConfiguration('verify').perform(context).lower() == 'true'}
    if config_dir:
        params['adapter_config_dir'] = config_dir
    log_dir = os.path.join(os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local')),
                           'tool_homing_logs')
    actions = [Node(
        package='arts_dv_tool_driver', executable='tool_homing', name='tool_homing',
        namespace=arm, output='screen',
        parameters=[{**params, 'jaw.log_dir': log_dir}],
        remappings=[('joy', '/console/gamepad')])]
    if LaunchConfiguration('gamepad').perform(context).lower() == 'true':
        actions.append(Node(
            package='joy', executable='joy_node', name='gamepad', output='screen',
            parameters=[{'device_id': 0, 'deadzone': 0.1, 'autorepeat_rate': 50.0}],
            remappings=[('joy', '/console/gamepad'),
                        ('joy/set_feedback', '/console/gamepad/set_feedback')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', description='psm_left | psm_right'),
        DeclareLaunchArgument('tool',
                              description='config/homing/<tool>.txt: forceps | scissors | hook | needledriver'),
        DeclareLaunchArgument('verify', default_value='false',
                              description='pause after each jaw to record its true center'),
        DeclareLaunchArgument('calibrate', default_value='false',
                              description="designer's learn-a-tool mode (slow, verify pauses, A save / B discard)"),
        DeclareLaunchArgument('config_dir', default_value='',
                              description='homing config dir; calibrate:=true needs the source tree'),
        DeclareLaunchArgument('gamepad', default_value='true',
                              description='start joy_node on /console/gamepad'),
        OpaqueFunction(function=_setup),
    ])
