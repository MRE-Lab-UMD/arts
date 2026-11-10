"""The da Vinci X adapter's disc servos for one PSM, in its namespace (decision 18).

    ros2 launch arts_dv_tool_driver disc_drive.launch.py arm:=psm_left
    ros2 launch arts_dv_tool_driver disc_drive.launch.py arm:=psm_left allow_mode_write:=true

The serial port comes from config/disc_buses.yaml. The node refuses to start if a servo is
not already in velocity mode, unless allow_mode_write:=true (Operating Mode is an EEPROM
setting; collaborator A's node wrote it on every start). Inputs/outputs:
<arm>/tool_actuator_target (velocities, rad/s) and <arm>/tool_actuator_states.

Instrument engagement (collaborator A's scheme, docs/modules/dv_tool_driver.md): services
<arm>/disc_drive/{go_to_start, spin, find_limits, return_to_start, stop}, refused until
engage_tick_period is A's measured loop period (disc_buses.yaml; test/a_tick_period_probe.py).

zero_from_homing:=true takes the joint zero from tool_homing's last COMPLETE homing of this
adapter ($ARTS_LOCAL/tool_homing_logs/home_<serial>.yaml) instead of the discs' start-up counts
(the visual zero); the node refuses if a disc has moved from it (power cycle). It also allows the
operating-mode write, since the homing leaves the servos in current-based position mode.

instrument_file:=<file in config/instruments/, e.g. 471006_large_needle_driver.yaml> loads that
instrument's joints, coupling and limits: <arm>/tool_joint_states out, <arm>/tool_joint_target
(joint velocities) in.
"""
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# tool_homing's tool files (config/homing/<tool>.txt) and the instrument each was made for.
HOMING_TOOL_REF = {'forceps': '471205', 'scissors': '470179', 'hook': '470183',
                   'needledriver': '471006'}


def _setup(context, *args, **kwargs):
    arm = LaunchConfiguration('arm').perform(context)
    with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config',
                           'disc_buses.yaml')) as f:
        buses = yaml.safe_load(f)
    if arm not in buses:
        raise RuntimeError(f'arm:={arm} has no bus in disc_buses.yaml ({sorted(buses)})')
    params = {}
    inst = LaunchConfiguration('instrument_file').perform(context)
    if inst:
        with open(os.path.join(get_package_share_directory('arts_dv_tool_driver'), 'config',
                               'instruments', inst)) as f:
            spec = yaml.safe_load(f)
        joints = spec['joints']
        lim = spec['limits']
        params = {'instrument': f"{spec['ref']} {spec['name']}",
                  'instrument_joints': joints,
                  'instrument_discs': [int(d.split('_')[1]) for d in spec['discs']],
                  'instrument_coupling': [float(v) for row in spec['coupling'] for v in row],
                  # A joint with no limits (the hook's jaw) is held at 0, as A does.
                  'instrument_min': [float(lim.get(j, {}).get('min', 0.0)) for j in joints],
                  'instrument_max': [float(lim.get(j, {}).get('max', 0.0)) for j in joints],
                  'instrument_jaw_gain': float(spec.get('jaw_gain', 10.0))}
    zero = {}
    from_homing = LaunchConfiguration('zero_from_homing').perform(context).lower() == 'true'
    if from_homing:
        serial = re.search(r'Serial_Converter_([A-Za-z0-9]+)-', buses[arm]['port']).group(1)
        path = os.path.join(os.environ.get('ARTS_LOCAL', os.path.expanduser('~/arts_local')),
                            'tool_homing_logs', f'home_{serial}.yaml')
        if not os.path.exists(path):
            raise RuntimeError(f'zero_from_homing: no {path} (home the instrument first: '
                               f'tool_homing.launch.py arm:={arm})')
        with open(path) as f:
            home = yaml.safe_load(f)
        print(f"[disc_drive.launch] zero from homing: {path}: tool {home['tool']}, "
              f"{home['time']}, counts {home['counts']}, flags '{home['flags']}'")
        # The homing's tool file must be the instrument being driven: a zero homed with one
        # instrument means nothing for another.
        ref = HOMING_TOOL_REF.get(home['tool'])
        if inst and ref and not inst.startswith(ref + '_'):
            raise RuntimeError(f"zero_from_homing: {path} was homed as tool '{home['tool']}' "
                               f'({ref}), not {inst}; home this instrument first')
        zero = {'zero_counts': [int(c) for c in home['counts']]}
    return [Node(
        package='arts_dv_tool_driver', executable='disc_drive', namespace=arm, output='screen',
        parameters=[{'port': buses[arm]['port'],
                     'baud_rate': int(buses[arm].get('baud_rate', 2000000)),
                     'allow_mode_write': from_homing or LaunchConfiguration(
                         'allow_mode_write').perform(context).lower() == 'true',
                     'max_rpm': float(LaunchConfiguration('max_rpm').perform(context)),
                     'engage_tick_period': float(
                         LaunchConfiguration('engage_tick_period').perform(context) or
                         buses[arm].get('engage_tick_period', 0.0)),
                     **zero, **params}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', description='psm_left | psm_right'),
        DeclareLaunchArgument('allow_mode_write', default_value='false'),
        DeclareLaunchArgument('max_rpm', default_value='30.0',
                              description='cap on any commanded disc speed'),
        DeclareLaunchArgument('engage_tick_period', default_value='',
                              description="A's measured homing-loop period [s]; default from "
                                          'disc_buses.yaml; 0 refuses the engagement services'),
        DeclareLaunchArgument('zero_from_homing', default_value='false',
                              description="joint zero from tool_homing's home_<serial>.yaml"),
        DeclareLaunchArgument('instrument_file', default_value='',
                              description='config/instruments/<file>; empty = discs only'),
        OpaqueFunction(function=_setup),
    ])
