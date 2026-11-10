"""One UR arm in its role namespace (decision 18): /psm_left, /psm_right or /ecm.

    ros2 launch arts_driver ur_arm.launch.py arm:=psm_left use_fake_hardware:=true
    ros2 launch arts_driver ur_arm.launch.py arm:=psm_left          # the real arm

Everything comes from config/ur_arms.yaml for the named arm: model (ur5e | ur5), IP, the four
driver ports, and the isolated core its loop is pinned to. tip:= picks the controller's
starting tool-tip offset from config/tool_tips.yaml (default flange; arts_bringup passes the
arm's own preset, da_vinci_x_wrist for the PSMs). An explicit robot_ip:= that
disagrees with the file is refused. Inside the namespace it starts:

  robot_state_publisher   <arm>/robot_description (latched), /tf (global)
  controller_manager      500 Hz (ur5e) or 125 Hz (ur5), SCHED_FIFO on the arm's core
  UR controllers          joint_state_broadcaster -> <arm>/joint_states, io_and_status
                          controller, speed scaling, force/torque -> <arm>/wrench, ...
  servo_controller        arts_controllers/CartesianServoController: reads
                          <arm>/tool_tip_target, publishes <arm>/tool_tip_state,
                          <arm>/joint_command and <arm>/rcm_state; services
                          <arm>/set_rcm and <arm>/set_tool_tip_offset
  arm_status              <arm>/status (one ArmStatus message, latched)
  tool_info               <arm>/tool_info (latched), from config/instruments.yaml[tool]
  real hardware only      dashboard client, robot state helper, controller stopper, URScript
                          interface (the UR driver's own helpers)

Moving it, once active (one message; jaw and feedforward stay at their defaults):

    ros2 topic pub -1 /psm_left/tool_tip_target arts_interfaces/msg/ToolTipTarget \\
        '{pose: {position: {x: 0.4, y: 0.1, z: 0.3}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}'

WHY THIS IS NOT A WRAPPER AROUND ur_control.launch.py
-----------------------------------------------------
Phase 1 included the UR launch file whole. It cannot run in a namespace: its spawners call
the absolute /controller_manager, and its controller file keys parameters by bare node name,
which ROS 2 does not apply to a namespaced node (checked 2026-11-01: a key `demo_node:` left a
node at /psm_left/demo_node unconfigured). So this file starts the same nodes itself, and
re-keys the PINNED, unmodified ur_controllers.yaml under the arm's namespace when the launch
runs. The description still comes from the UR driver's own ur_rsp.launch.py, which works in a
namespace unchanged.
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

UPDATE_RATE_HZ = {'ur5e': 500, 'ur5': 125}
# Started active by the UR launch; tcp_pose_broadcaster needs the real robot. The servo
# controller is spawned separately (its own parameter file) and is NOT in the controller
# stopper's consistent list, so a stopped pendant program stops it and a resumed one restarts
# it, seeded from the measured joints.
UR_ACTIVE = ['joint_state_broadcaster', 'io_and_status_controller',
             'speed_scaling_state_broadcaster', 'ur_configuration_controller',
             'gravity_update_controller', 'friction_model_controller']
UR_ACTIVE_REAL_ONLY = ['tcp_pose_broadcaster']
UR_INACTIVE = ['scaled_joint_trajectory_controller', 'joint_trajectory_controller',
               'forward_velocity_controller', 'forward_position_controller']
CONSISTENT = ['io_and_status_controller', 'force_torque_sensor_broadcaster',
              'joint_state_broadcaster', 'speed_scaling_state_broadcaster',
              'tcp_pose_broadcaster', 'ur_configuration_controller', 'gravity_update_controller']

# Temporary parameter files must outlive launch_setup(): the nodes read them after it returns.
_KEEP_ALIVE = []


def _share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def _namespaced_params(path, ns, substitutions):
    """Load a ROS parameter YAML, expand $(var ...), and re-key every node under /ns/."""
    with open(path) as f:
        text = f.read()
    for key, value in substitutions.items():
        text = text.replace(f'$(var {key})', value)
    data = yaml.safe_load(text)
    keyed = {f'/{ns}/{node}': body for node, body in data.items()}
    out = tempfile.NamedTemporaryFile('w', prefix=f'arts_{ns}_', suffix='.yaml', delete=False)
    yaml.safe_dump(keyed, out)
    out.close()
    _KEEP_ALIVE.append(out.name)
    return out.name


def launch_setup(context, *args, **kwargs):
    arm = LaunchConfiguration('arm').perform(context)
    mock = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'
    with open(_share('arts_driver', 'config', 'ur_arms.yaml')) as f:
        registry = yaml.safe_load(f)
    if arm not in registry['arms']:
        raise RuntimeError(f'arm:={arm} is not in ur_arms.yaml (expected one of '
                           f'{sorted(registry["arms"])})')
    a = registry['arms'][arm]
    model = a['model']
    ip = LaunchConfiguration('robot_ip').perform(context)
    if not mock:
        if ip in ('', '0.0.0.0'):
            ip = a['ip']
        elif ip != a['ip']:
            raise RuntimeError(f'robot_ip:={ip} but {arm} is {a["ip"]} in ur_arms.yaml')
    tf_prefix = f'{arm}_'
    cm = f'/{arm}/controller_manager'

    # Controller manager: the UR controller declarations (pinned, unmodified file) plus the
    # rate and core for this arm.
    ur_params = _namespaced_params(
        _share('ur_robot_driver', 'config', 'ur_controllers.yaml'), arm,
        {'tf_prefix': tf_prefix})
    cm_params = {f'/{arm}/controller_manager': {'ros__parameters': {
        'update_rate': UPDATE_RATE_HZ[model],
        'cpu_affinity': [int(a['cpu_core'])],
        'lock_memory': True,
        'hardware_synchronization.expect_blocking_read_write': False,
        'overruns.print_warnings': True,
    }}}
    rate_file = tempfile.NamedTemporaryFile('w', prefix=f'arts_{arm}_rate_', suffix='.yaml',
                                            delete=False)
    yaml.safe_dump(cm_params, rate_file)
    rate_file.close()
    _KEEP_ALIVE.append(rate_file.name)

    # mesh_format:=stl (decision 13 amendment): SlicerROS2 cannot read DAE, so the visual
    # meshes are swapped for the STL meshes ur_description already installs (its collision
    # set, in metres). The vendor parameter file is read and rewritten here, never copied
    # into the tree; nothing is redistributed.
    visual_params = _share('ur_description', 'config', model, 'visual_parameters.yaml')
    if LaunchConfiguration('mesh_format').perform(context) == 'stl':
        with open(visual_params) as f:
            vis = yaml.safe_load(f)
        for part in vis['mesh_files'].values():
            if isinstance(part, dict) and 'collision' in part:
                part['visual'] = {'mesh': dict(part['collision']['mesh'])}
        out = tempfile.NamedTemporaryFile('w', prefix=f'arts_{arm}_visual_', suffix='.yaml',
                                          delete=False)
        yaml.safe_dump(vis, out)
        out.close()
        _KEEP_ALIVE.append(out.name)
        visual_params = out.name

    # Per arm: ur_arms.yaml command_interface (the ECM: position, 2026-11-07) unless given here.
    command_interface = LaunchConfiguration('command_interface').perform(context)
    if command_interface == '':
        command_interface = a.get('command_interface', 'velocity')
    servo_params = _namespaced_params(
        LaunchConfiguration('servo_config').perform(context), arm,
        {'tf_prefix': tf_prefix, 'command_interface': command_interface})
    # tip:=<key> sets the controller's starting tool-tip offset from config/tool_tips.yaml.
    tip = LaunchConfiguration('tip').perform(context)
    with open(_share('arts_driver', 'config', 'tool_tips.yaml')) as f:
        tips = yaml.safe_load(f)
    if tip not in tips:
        raise RuntimeError(f'tip:={tip} is not in tool_tips.yaml (expected one of {sorted(tips)})')
    with open(servo_params) as f:
        sp = yaml.safe_load(f)
    sp[f'/{arm}/servo_controller']['ros__parameters']['tool_tip_offset'] = [
        float(x) for x in tips[tip]['offset']]
    if 'rcm_orientation_task' in tips[tip]:
        sp[f'/{arm}/servo_controller']['ros__parameters'].setdefault('rcm', {})[
            'orientation_task'] = tips[tip]['rcm_orientation_task']
    # wrist_instrument:=<file in arts_dv_tool_driver/config/instruments/>: the instrument's roll,
    # pitch and yaw join the controller (decision 17, 2026-11-03 amendment), with that file's
    # limits; the QP solver with joint weights favoring the instrument (LOWER = cheaper: UR 1.0,
    # instrument 0.01; MOPS used 0.2 : 1, see below) resolves the redundancy
    # left with the RCM off. The drive (disc_drive, or the fake instrument on mock) must run.
    wrist_file = LaunchConfiguration('wrist_instrument').perform(context)
    if wrist_file:
        with open(_share('arts_dv_tool_driver', 'config', 'instruments', wrist_file)) as f:
            spec = yaml.safe_load(f)
        lim = spec['limits']
        wj = ['roll', 'wrist_pitch', 'wrist_yaw']
        rp = sp[f'/{arm}/servo_controller']['ros__parameters']
        # The file nests its parameters; set them nested so no dotted key collides.
        rp.setdefault('wrist', {}).update({
            'enabled': True,
            'joints': [f'{arm}_tool_{j}' for j in wj],
            'lower_limits': [float(lim[j]['min']) for j in wj],
            'upper_limits': [float(lim[j]['max']) for j in wj]})
        rp.setdefault('tracking', {})['solver'] = 'qp'
        # Tip angular cap 15 -> 90 deg/s with the wrist in (operator decision, 2026-11-05): the
        # 15 deg/s v1.4 cap was set for the UR wrist alone and made instrument orientation teleop
        # crawl; the instrument joints allow 3 rad/s, the UR joints stay at 1 rad/s.
        rp.setdefault('cartesian', {})['max_angular_velocity'] = math.radians(90.0)
        # Instrument 100x cheaper than the UR (was MOPS's 0.2 : 1). At 0.2 the least-norm split
        # gave the UR a visible share of every wrist rotation, which the posture term then pulled
        # back after the hand stopped: the elbow kept sweeping at 10-26 deg/s with the tip on
        # target (first orientation teleop on the real arm, 2026-11-05).
        rp.setdefault('task_qp', {})['joint_weights'] = [1.0] * 6 + [0.01] * 3
    with open(servo_params, 'w') as f:
        yaml.safe_dump(sp, f)

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(_share('ur_robot_driver', 'launch', 'ur_rsp.launch.py')),
        launch_arguments={
            'ur_type': model,
            'robot_ip': ip if not mock else '0.0.0.0',
            'tf_prefix': tf_prefix,
            'use_mock_hardware': 'true' if mock else 'false',
            'headless_mode': LaunchConfiguration('headless_mode'),
            'description_file': LaunchConfiguration('description_file'),
            'visual_params_file': visual_params,
            # Each arm runs with its own factory calibration (docs/modules/ur_arms.md).
            'kinematics_params_file': _share('arts_calibration', 'config',
                                             f'{arm}_kinematics.yaml'),
            'reverse_port': str(a['reverse_port']),
            'script_sender_port': str(a['script_sender_port']),
            'trajectory_port': str(a['trajectory_port']),
            'script_command_port': str(a['script_command_port']),
        }.items(),
    )

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[rate_file.name, ur_params], output='screen')

    def spawner(controllers, inactive=False, extra=()):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            parameters=[{'verify_payload_on_set': not mock}],
            arguments=['--controller-manager', cm, '--controller-manager-timeout', '30']
            + (['--inactive'] if inactive else []) + list(extra) + controllers)

    active = UR_ACTIVE + ([] if mock else UR_ACTIVE_REAL_ONLY)
    fake_instrument = []
    if wrist_file and mock:
        fake_instrument = [Node(
            package='arts_dv_tool_driver', executable='fake_instrument.py',
            output='screen', parameters=[{'instrument_file': wrist_file}])]  # in the arm's group
    nodes = [
        rsp,
        control_node,
        spawner(active),
        spawner(UR_INACTIVE, inactive=True),
        # The UR5e flange force/torque sensor -> <arm>/wrench (decision 18). The UR5 has no
        # sensor, so the ECM gets no wrench.
        *([spawner(['force_torque_sensor_broadcaster'],
                   extra=['--controller-ros-args', '-r ~/wrench:=wrench'])]
          if model == 'ur5e' else []),
        spawner(['servo_controller'], extra=['--param-file', servo_params]),
        Node(package='arts_driver', executable='arm_status', output='screen',
             parameters=[{'model': model}]),
        Node(package='arts_driver', executable='tool_info', output='screen',
             parameters=[{'tool': LaunchConfiguration('tool')}]),
    ]
    real_only = UnlessCondition(LaunchConfiguration('use_fake_hardware'))
    nodes += [
        Node(package='ur_robot_driver', executable='dashboard_client', name='dashboard_client',
             output='screen', parameters=[{'robot_ip': ip}], condition=real_only),
        Node(package='ur_robot_driver', executable='robot_state_helper',
             name='ur_robot_state_helper', output='screen', condition=real_only,
             parameters=[{'headless_mode': LaunchConfiguration('headless_mode')},
                         {'robot_ip': ip}]),
        Node(package='ur_robot_driver', executable='controller_stopper_node',
             name='controller_stopper', output='screen', emulate_tty=True,
             condition=real_only,
             parameters=[{'headless_mode': LaunchConfiguration('headless_mode')},
                         {'joint_controller_active': False},
                         {'consistent_controllers': CONSISTENT}]),
        Node(package='ur_robot_driver', executable='urscript_interface', output='screen',
             parameters=[{'robot_ip': ip}], condition=real_only),
    ]
    return [GroupAction([PushRosNamespace(arm), *nodes, *fake_instrument])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', description='psm_left | psm_right | ecm (ur_arms.yaml keys)'),
        DeclareLaunchArgument('mesh_format', default_value='dae',
                              description='dae (vendor visual meshes) | stl (for SlicerROS2, which '
                                          'cannot read DAE)'),
        DeclareLaunchArgument('tool', default_value='none',
                              description='instrument key in config/instruments.yaml'),
        DeclareLaunchArgument('wrist_instrument', default_value='',
                              description='instrument file (arts_dv_tool_driver/config/'
                                          'instruments/): the wrist joins the controller'),
        DeclareLaunchArgument('tip', default_value='flange',
                              description='tool-tip preset in config/tool_tips.yaml (flange | '
                                          'da_vinci_x_wrist); arts_bringup passes ur_arms.yaml'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='ros2_control mock hardware instead of the arm.'),
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0',
                              description='Leave unset: taken from ur_arms.yaml (a different '
                                          'value is refused).'),
        DeclareLaunchArgument('headless_mode', default_value='false',
                              description='Send the control program directly instead of waiting '
                                          'for External Control on the pendant.'),
        DeclareLaunchArgument(
            'command_interface', default_value='',
            description='velocity (UR speedj, the default since the 2026-10-30 A/B) or position '
                        '(UR servoj, ~30 ms more lag); empty: ur_arms.yaml command_interface, '
                        'else velocity.'),
        DeclareLaunchArgument(
            'servo_config', default_value=_share('arts_controllers', 'config', 'ur5e_servo.yaml')),
        DeclareLaunchArgument(
            'description_file', default_value=_share('arts_driver', 'urdf', 'arts_ur_arm.urdf.xacro'),
            description='The vendor description rooted at <arm>_mount, with a non-singular mock start pose.'),
        OpaqueFunction(function=launch_setup),
    ])
