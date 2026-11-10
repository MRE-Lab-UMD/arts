"""ARTS bringup: one launch file, one switch per module (decision 10 ladder, decision 18 names).

    ros2 launch arts_driver arts_bringup.launch.py \\
        psm_left:=true  psm_left_tool:=large_needle_driver \\
        psm_right:=true psm_right_tool:=monopolar_curved_scissors \\
        ecm:=true ecm_stereo:=true ecm_stereo_sync:=true \\
        mtm_left:=true mtm_right:=true footswitch_teleop:=true head_sensor:=true gamepad:=true \\
        use_fake_hardware:=false rviz:=true

Each switch is named after the namespace path its topics live under (decision 18), with '/'
written '_': ecm_stereo_left -> /ecm/stereo_left/*, ecm_stereo_sync -> /ecm/stereo_sync/pulse,
footswitch_teleop -> /console/footswitch_teleop, head_sensor -> /console/head_sensor,
gamepad -> /console/gamepad. `ecm` is the camera ARM only; the cameras are their own switches.

  psm_left, psm_right, ecm   one UR arm each, own namespace and controller manager
                             (ur_arm.launch.py); tip preset from ur_arms.yaml
  psm_left_tool_actuator,    the adapter's Dynamixel disc servos for that PSM
  psm_right_tool_actuator    (arts_dv_tool_driver disc_drive -> <psm>/tool_actuator_states);
                             independent of the arm, so the discs can be checked alone
  mtm_left, mtm_right        Haply Inverse3 + VerseGrip per side (one bridge process, one
                             WebSocket, publishing only the sides switched on)
  ecm_stereo_left/_right     one FLIR camera each; ecm_stereo = both
  ecm_stereo_sync            the ESP32 sync box's micro-ROS agent; with it on, the cameras
                             run hardware-triggered, without it free-running
  footswitch_teleop          the Savant Elite2 teleop pedals (NOT the ESU's pedals, which the
                             ESU-TAP board reads and publish under /esu with `esu`)
  head_sensor                GOOVIS head presence
  gamepad                    Xbox controller -> /console/gamepad, plus the jog source
                             (arts_teleop gamepad_jog)
  esu                        the ESU-TAP board: its micro-ROS agent + the esu node (arts_esu
                             esu.launch.py; always the real board, like ecm_stereo_sync; the
                             fake board is esu.launch.py fake:=true, on its own ROS domain)
  rviz                       RViz with the cart config
  cart_tf                    the cart-level tf edges (default on; exactly one process may run it)
  collision_guard            capsule distances between and within the arms; below 20 mm every
                             arm holds (arts_safety; default on whenever an arm runs; release:
                             ros2 service call /collision_guard/release std_srvs/srv/Trigger)

Camera-process options (decision 5: the recorder and the console display are composed INTO
the camera process, so they are chosen when the cameras start): display, display_fullscreen,
display_hmd, display_hmd_pos, display_tower_pos, image_bag, image_bag_paused. They are passed
to cameras.launch.py unchanged.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARMS = ('psm_left', 'psm_right', 'ecm')
MTMS = ('mtm_left', 'mtm_right')
CAMERAS = ('ecm_stereo_left', 'ecm_stereo_right')
SWITCHES = (ARMS + MTMS + CAMERAS
            + ('psm_left_tool_actuator', 'psm_right_tool_actuator',
               'ecm_stereo', 'ecm_stereo_sync', 'footswitch_teleop', 'head_sensor', 'gamepad',
               'esu', 'cart_tf', 'collision_guard', 'use_fake_hardware', 'rviz'))
DEFAULT_ON = ('cart_tf', 'collision_guard', 'use_fake_hardware')
CAMERA_OPTIONS = {'display': 'false', 'display_fullscreen': 'false', 'display_hmd': 'true',
                  'display_hmd_pos': '0,0', 'display_tower_pos': '0,0',
                  'image_bag': '', 'image_bag_paused': 'false'}


def _launch(pkg, name, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), 'launch', name])),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def _launch_setup(context, *args, **kwargs):
    lc = context.launch_configurations
    on = {s: lc[s].lower() == 'true' for s in SWITCHES}
    if on['ecm_stereo']:
        on['ecm_stereo_left'] = on['ecm_stereo_right'] = True
    actions = [LogInfo(msg=f'[arts_driver] bringup: '
                           f'{[s for s in SWITCHES if on[s]] or "(none)"}')]
    fake = 'true' if on['use_fake_hardware'] else 'false'
    mapping = PathJoinSubstitution([FindPackageShare('arts_teleop'), 'config',
                                    'input_mapping.yaml'])
    with open(os.path.join(get_package_share_directory('arts_driver'), 'config',
                           'ur_arms.yaml')) as f:
        registry = yaml.safe_load(f)['arms']

    # The one publisher of the cart-level tf edges (world -> mounts, pedestal, console). A
    # supervisor that starts modules as separate bringup processes (arts_gui) runs it once
    # and passes cart_tf:=false to the rest.
    if on['cart_tf']:
        actions.append(Node(package='arts_driver', executable='cart_tf', output='screen'))
    if on['collision_guard'] and any(on[a] for a in ARMS):
        actions.append(Node(
            package='arts_safety', executable='collision_guard.py', name='collision_guard',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('arts_safety'), 'config',
                                     'collision_guard.yaml'),
                        {'arms': [a for a in ARMS if on[a]]}]))
    with open(os.path.join(get_package_share_directory('arts_driver'), 'config',
                           'instruments.yaml')) as f:
        catalog = yaml.safe_load(f)
    for arm in ARMS:
        if on[arm]:
            # wrist:=true: a PSM's instrument wrist joins its controller (decision 17,
            # 2026-11-03 amendment); needs the instrument drive (psm_*_tool_actuator) on real
            # hardware, a fake instrument on mock.
            wrist_file = ''
            if lc['wrist'].lower() == 'true' and arm.startswith('psm'):
                wrist_file = (catalog.get(lc.get(f'{arm}_tool', 'none')) or {}).get(
                    'kinematics_file', '')
                if not wrist_file:
                    raise RuntimeError(f'wrist:=true but {arm}_tool has no kinematics_file')
            actions.append(_launch('arts_driver', 'ur_arm.launch.py', arm=arm,
                                   use_fake_hardware=fake, tool=lc.get(f'{arm}_tool', 'none'),
                                   tip=registry[arm].get('tip', 'flange'),
                                   mesh_format=lc['mesh_format'], wrist_instrument=wrist_file))

    for psm in ('psm_left', 'psm_right'):
        if on[f'{psm}_tool_actuator']:
            # The instrument's kinematics file from the catalog (config/instruments.yaml).
            with open(os.path.join(get_package_share_directory('arts_driver'), 'config',
                                   'instruments.yaml')) as f:
                kin = (yaml.safe_load(f).get(lc.get(f'{psm}_tool', 'none')) or {}).get(
                    'kinematics_file', '')
            actions.append(_launch('arts_dv_tool_driver', 'disc_drive.launch.py', arm=psm,
                                   allow_mode_write=lc['allow_mode_write'],
                                   instrument_file=kin))

    cameras_on = on['ecm_stereo_left'] or on['ecm_stereo_right']
    if cameras_on:
        # The pedal node goes INSIDE the camera launch when it starts, so a recording episode
        # mark and the pedal stream land in the image bag with the frames (cameras.launch.py).
        actions.append(_launch(
            'arts_driver', 'cameras.launch.py',
            enable_left=str(on['ecm_stereo_left']).lower(),
            enable_right=str(on['ecm_stereo_right']).lower(),
            trigger=str(on['ecm_stereo_sync']).lower(),
            footswitch_teleop=str(on['footswitch_teleop']).lower(),
            **{k: lc[k] for k in CAMERA_OPTIONS}))
    elif on['footswitch_teleop']:
        actions.append(Node(
            package='arts_teleop', executable='footswitch.py', name='footswitch_teleop',
            output='screen', parameters=[{'mapping_file': mapping}]))
    if on['ecm_stereo_sync']:
        actions.append(_launch('arts_camera_sync', 'camera_sync.launch.py'))
    if on['esu']:
        actions.append(_launch('arts_esu', 'esu.launch.py'))

    sides = [m.split('_')[1] for m in MTMS if on[m]]
    if sides:
        actions.append(Node(
            package='arts_driver', executable='haply_bridge', name='mtm_bridge', output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('arts_driver'), 'config',
                                              'haply.yaml']),
                        {'sides': sides}]))
    if on['head_sensor']:
        actions.append(Node(package='arts_teleop', executable='head_sensor.py',
                            name='head_sensor', output='screen'))
    if on['gamepad']:
        actions.append(Node(
            package='joy', executable='joy_node', name='gamepad', output='screen',
            parameters=[{'device_id': 0, 'deadzone': 0.1, 'autorepeat_rate': 50.0}],
            remappings=[('joy', '/console/gamepad'), ('joy/set_feedback',
                                                      '/console/gamepad/set_feedback')]))
        actions.append(Node(package='arts_teleop', executable='gamepad_jog.py',
                            name='gamepad_jog', output='screen'))
    if on['rviz']:
        # rviz2 (Jazzy) dies or hangs when it gets SIGINT twice, which is what Ctrl+C does under
        # ros2 launch: the terminal signals the whole process group and launch forwards the
        # signal again. setsid puts rviz2 in its own session, so only launch's SIGINT reaches it.
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log', prefix='setsid',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('arts_description'),
                                                   'rviz', 'arts.rviz'])]))
    return actions


def generate_launch_description():
    declared = [DeclareLaunchArgument(s, default_value='true' if s in DEFAULT_ON else 'false')
                for s in SWITCHES]
    declared += [DeclareLaunchArgument(
        'allow_mode_write', default_value='false',
        description='disc servos: permit the EEPROM operating-mode write (arts_dv_tool_driver)')]
    declared += [DeclareLaunchArgument('wrist', default_value='false',
                                       description='the PSMs\' instrument wrists join their '
                                                   'controllers (9 joints)')]
    declared += [DeclareLaunchArgument('mesh_format', default_value='dae',
                                       description='dae | stl (stl for SlicerROS2)')]
    declared += [DeclareLaunchArgument(f'{p}_tool', default_value='none',
                                       description='instrument key in config/instruments.yaml')
                 for p in ARMS]
    declared += [DeclareLaunchArgument(k, default_value=v,
                                       description='passed to cameras.launch.py')
                 for k, v in CAMERA_OPTIONS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
