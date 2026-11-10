"""ARTS camera bringup — launches FLIR Blackfly S nodes for the stereo pair.

Free-running (default; this is what the committed 40 Hz baselines used):
  ros2 launch arts_driver cameras.launch.py

Hardware triggered from the ESP32 sync box (FLIR Stage 5, rungs 5c-5e):
  ros2 launch arts_driver cameras.launch.py trigger:=true enable_right:=false
  ros2 launch arts_driver cameras.launch.py trigger:=true

Single camera:
  ros2 launch arts_driver cameras.launch.py enable_right:=false

Included by arts_bringup.launch.py when ecm_stereo_left/_right (or ecm_stereo) is on.

TRIGGERED MODE DOES NOT SET THE FRAME RATE. The ESP32 does (AGENTS.md decision 14). The
`rate` argument below only configures the diagnostics window, so a mismatch between it and
the firmware's CONFIG_ARTS_SYNC_TRIGGER_RATE_HZ shows up as a diagnostics complaint rather
than as a rate change.

ONE PROCESS FOR BOTH CAMERAS, AND IT IS A CORRECTNESS REQUIREMENT RATHER THAN AN
OPTIMIZATION. Upstream states that Spinnaker "does not support two programs accessing the
Spinnaker SDK at the same time, even if two different cameras are accessed. Strange things
happen, in particular with USB3 cameras" (spinnaker_camera_driver/doc/index.rst). ARTS ran
two `camera_driver_node` processes through rung 5e and saw exactly that: both drivers
dropping to 12-15 Hz and recovering WITHIN 2 ms OF EACH OTHER despite sitting on separate
USB host controllers, which is the observation that ruled out per-controller bandwidth and
left the shared SDK as the only resource spanning both. So the cameras are composable nodes
in ONE container, per the decision-5 amendment.

Template is upstream's own `multiple_cameras.launch.py`, with two deliberate departures.
`trigger_source` is Line0, not the example's Line3: Line 3 is non-isolated with a 3.6 V
absolute-maximum input, so following the example from a 5 V board destroys a camera
(decision 14). And the container is NOT given the `exposure_control` remapping, because
ARTS fixes exposure rather than closing a loop on it — a wandering exposure would make the
ExposureActive strobe width wander, which is the signal the sync box measures.

TWO THINGS THAT DO NOT CHANGE when composing, both worth stating because it is natural to
assume otherwise. Topic names are unchanged: `Camera` advertises `"~/" + topicPrefix_ +
"meta"` with an EMPTY prefix in this driver, so a node named `camera_left` still publishes
`/camera_left/meta` and `/camera_left/image_raw` exactly as it did standalone, and every
recorded bag, analysis script and topic list keeps working. (The `<container>/<cam>/meta`
layout belongs to the SYNCHRONIZED driver, where one node owns several cameras; ARTS does
not use it.) And a single-threaded container is not a throughput risk, because each `Camera`
spawns its own acquisition thread and calls `doPublish` on it (camera.cpp:193, 728, 799) —
the container's executor carries only the 1 Hz diagnostics timers and parameter callbacks.

WHAT COMPOSING DOES COST, measured here 2026-08-20 with both cameras unplugged: ONE ABSENT
CAMERA NOW TAKES DOWN THE OTHER. `Camera::openDevice` retries a hardcoded 5 times at 1 s
(camera.cpp:279, `retry < 6`, NOT a parameter) and then throws; upstream then dies with
SIGSEGV rather than exiting cleanly, which the standalone node does too — so the crash is
upstream's, not composition's. The difference is the blast radius. `load_composable_nodes`
loads sequentially, so the container died while loading `camera_left` and `camera_right` was
never loaded at all; run the same thing standalone and the two processes fail independently.
The realistic trigger is not an unplugged camera but a slow one: a camera that enumerates
more than ~5 s after the container starts, or the USB-C dongle coming up without its camera
beneath it (seen on this laptop). So CHECK BOTH CAMERAS ARE ENUMERATED (`lsusb`) BEFORE
LAUNCHING, and read a dead container as "one of them was late", not as a driver fault.
Deliberately NOT mitigated with `respawn=True`: a genuinely missing camera would then become
an endless crash loop instead of one clear "giving up" message, and a mid-session respawn
would restart the HEALTHY camera too, re-phasing its exposure so the skew baseline measured
earlier in the recording no longer describes the rest of it. Better fixed upstream, where
the 5-try budget should be a parameter and a failed `configure()` should not segfault.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# The driver's registered component, from its CMakeLists rclcpp_components_register_nodes.
CAMERA_PLUGIN = 'spinnaker_camera_driver::CameraDriver'

# Container node name. Fixed rather than an argument: it appears in `ros2 component list`
# output and in bug reports, and one predictable name is worth more than the flexibility.
CONTAINER_NAME = 'arts_camera_container'


# Driver-level parameters, declared by the driver itself rather than by the parameter
# definition file, so they are valid in both modes.
DRIVER_PARAMS = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': True,
    'dump_node_map': False,
    'buffer_queue_size': 10,
}

# Factory settings loaded into camera RAM at every start, on both paths. Both definition
# files list these two entries first, so every later write lands on the factory set rather
# than on whatever UserSet the camera's flash boots. Nothing is written to flash.
#
# Why: both ARTS cameras boot a UserSet1 saved with the Spinnaker GUI (by collaborator A, to
# make the two cameras and optical channels look alike) that differs from factory in ISP,
# the frame-rate limiter, gamma, white balance, colour matrix and sharpening. A replicating
# lab's cameras will not carry that set, so the published configuration must not depend on
# it. Per-camera image matching, if wanted, belongs in this file, explicit and versioned.
# Node-map diff: docs/modules/pilot_flir_camera.md.
FACTORY_RESET_PARAMS = {
    'user_set_selector': 'Default',
    'user_set_load': True,  # a command: any value executes UserSetLoad
}

# Free-running configuration. Unchanged from the version the 10-minute baselines in
# docs/benchmarks/flir_bfs-u3-63s4c_*.md were recorded with, except FACTORY_RESET_PARAMS,
# which restores the factory boot state those baselines were taken from.
#
# One wart kept visible rather than silently dropped: `frame_rate_auto` is NOT defined in
# upstream's blackfly_s.yaml, and the driver skips parameters it has no definition for, so
# this line has never done anything. It is removed here because a parameter that looks
# effective and is not is worse than no parameter; the rate was always set by
# frame_rate_enable + frame_rate.
FREERUN_PARAMS = {
    **FACTORY_RESET_PARAMS,
    'gain_auto': 'Continuous',
    'exposure_auto': 'Continuous',
    'frame_rate': 40.0,
    'frame_rate_enable': True,
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
    'diagnostic_period': 1.0,
    'diagnostic_min_freq': 39.0,
    'diagnostic_max_freq': 41.0,
}


def _triggered_params(rate, exposure_us, gain_db, trigger_delay_us, activation, enable_3v3,
                      wb_red, wb_blue, gamma, overlap, binning):
    """Hardware-triggered configuration.

    Every non-obvious value here is a measured camera default that must be overridden; the
    evidence is in docs/modules/pilot_flir_camera.md under "Camera defaults measured from
    the node map". The ORDER these reach the camera is set by
    config/blackfly_s_arts.yaml, not by this dict — see that file's header.
    """
    return {
        # A command, before the factory load; see config/blackfly_s_arts.yaml. Triggered path
        # only: upstream's blackfly_s.yaml (free-running) has no definition for it, and the
        # driver silently skips undefined parameters.
        'acquisition_stop': True,
        **FACTORY_RESET_PARAMS,
        'trigger_mode_disarm': 'Off',
        'pixel_format': 'BayerRG8',
        'isp_enable': False,

        # Recording resolution (decision 5 amendment, 2026-10-28): 2x2 averaging binning in
        # the camera gives 1536x1024 raw Bayer over the full field of view, ~47 MB/s per
        # camera at 30 Hz instead of ~189 MB/s at 3072x2048. binning:=1 is the full-
        # resolution upgrade path. Average, not Sum, so exposure and gain keep their meaning.
        'binning_selector': 'All',
        'binning_horizontal_mode': 'Average',
        'binning_vertical_mode': 'Average',
        'binning_horizontal': binning,
        'binning_vertical': binning,

        # Fixed exposure and gain. Auto exposure would make the ExposureActive pulse width
        # wander frame to frame, and on this camera it settles near its 15000 us ceiling.
        'gain_auto': 'Off',
        'gain': gain_db,
        'exposure_auto': 'Off',
        'exposure_time': exposure_us,

        # Colour preset: fixed white balance and gamma, defaults from collaborator A's saved
        # UserSet1 (see the launch arguments). Fixed rather than auto because auto white
        # balance drifts with the scene, and both settings are applied in the camera to the
        # Bayer data that is recorded (measured 2026-10-28), so a drift would be in the data.
        'balance_white_auto': 'Off',
        'balance_ratio_selector_red': 'Red',
        'balance_ratio_red': wb_red,
        'balance_ratio_selector_blue': 'Blue',
        'balance_ratio_blue': wb_blue,
        'gamma_enable': True,
        'gamma': gamma,

        # The sync box owns the rate; disable the camera's own limiter so there is only one
        # thing that can throttle acquisition.
        'frame_rate_enable': False,

        # Line 1 = ExposureActive strobe back to the ESP32.
        'line1_selector': 'Line1',
        'line1_linemode': 'Output',
        'line1_linesource': 'ExposureActive',

        # Line 2 carries a switchable 3.3 V rail, disabled from the factory. Enabling it
        # puts a known DC voltage on a known pigtail wire, which is the only static way to
        # confirm the connector's colour-to-pin mapping without cutting anything open.
        'line2_selector': 'Line2',
        'line2_v33enable': enable_3v3,

        # Trigger. trigger_mode is applied LAST because the definition file orders it last.
        'trigger_selector': 'FrameStart',
        'trigger_source': 'Line0',
        'trigger_activation': activation,
        'trigger_overlap': overlap,
        'trigger_delay': trigger_delay_us,
        'trigger_mode': 'On',

        # chunk_enable_frame_id is what decision 15's image-to-pulse join reads.
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True,

        'stream_buffer_handling_mode': 'OldestFirst',

        # Diagnostics only — see the module docstring. A tight window is wanted here: the
        # whole point of rung 5c is that the camera follows the pulse exactly.
        'diagnostic_period': 1.0,
        'diagnostic_min_freq': rate - 1.0,
        'diagnostic_max_freq': rate + 1.0,
    }


# header.frame_id per camera: the ECM stereo optical frames (decisions 12, 18), so images join
# the TF tree. The driver's default is the node name (stereo_left), which no tree contains. These are
# SENSOR frames: images stay as the sensor reads them, and the head's 180° mounting roll is a
# TF transform plus a display rotation, never ReverseX/ReverseY.
# Decision 18: the cameras are the ECM's stereo pair, so their topics are
# /ecm/stereo_left/{image_raw, camera_info, meta} and /ecm/stereo_right/... The driver publishes
# relative to its node, so the namespace plus the node name gives exactly that.
NAMESPACE = 'ecm'

OPTICAL_FRAME = {
    'stereo_left': 'ecm_stereo_left_optical_frame',
    'stereo_right': 'ecm_stereo_right_optical_frame',
}


def _camera_params(name, serial, parameter_file, mode_params):
    """Parameter list shared by the composed and standalone forms.

    Kept as one function so the two forms cannot be configured differently — the whole point
    of retaining the standalone form is an A/B on process count, which is worthless if the
    parameters also differ.
    """
    return [
        DRIVER_PARAMS,
        mode_params,
        {
            'parameter_file': parameter_file,
            'serial_number': serial,
            'frame_id': OPTICAL_FRAME[name],
        },
    ]


def _make_composable_camera_node(name, serial, parameter_file, mode_params):
    return ComposableNode(
        package='spinnaker_camera_driver',
        plugin=CAMERA_PLUGIN,
        name=name,
        namespace=NAMESPACE,
        parameters=_camera_params(name, serial, parameter_file, mode_params),
        # No in-process subscriber exists yet, so today this changes nothing measurable. It
        # is set now because it is what makes the container worth having later: rectification,
        # the display path and GPU JPEG encoding are all in-process consumers of full-res
        # Bayer frames, and for those the difference is a pointer versus a 6 MB copy per
        # frame per consumer.
        extra_arguments=[{'use_intra_process_comms': True}],
    )


# Topics the in-process image recorder writes. Images, their camera_info and the meta topics
# (camera_time, decision 5) stay together in the image bag; robot state and the sync pulses
# belong to the state bag, recorded elsewhere.
IMAGE_BAG_TOPICS = [
    '/%s/%s/%s' % (NAMESPACE, cam, t) for cam in ('stereo_left', 'stereo_right')
    for t in ('image_raw', 'camera_info', 'meta')
]


def _make_image_recorder(uri, start_paused, extra_topics):
    """rosbag2 recorder as a component in the camera container (docs/modules/console_display.md).

    Measured 2026-10-28: a separate `ros2 bag record` process taking both 1536x1024 image
    topics lost 125 and 111 meta frames in 60 s, against 0 with the images left out. In the
    camera process the frames do not cross a process boundary.
    """
    return ComposableNode(
        package='rosbag2_transport',
        plugin='rosbag2_transport::Recorder',
        name='image_recorder',
        parameters=[{
            'record.topics': IMAGE_BAG_TOPICS + extra_topics,
            'record.start_paused': start_paused,
            'record.disable_keyboard_controls': True,
            'storage.uri': uri,
            'storage.storage_id': 'mcap',
            # Writes go through a cache drained by the writer's own thread, so a slow disk
            # write does not block the subscription callback. 256 MiB is ~2.7 s of both
            # cameras at 1536x1024, 30 Hz.
            'storage.max_cache_size': 256 * 1024 * 1024,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def _make_display(fullscreen, overlay, hmd_pos, tower_pos, hmd=True, size='1920,1080',
                  recorder_present=False, pair_tolerance_ms=5.0):
    """Surgeon display (arts_display) in the camera container, beside the recorder.

    Window positions default to one screen for the bench; on the console they are the HMD's
    and the tower monitor's desktop offsets, "X,Y" as `xrandr --listmonitors` prints them
    (docs/modules/console_display.md).
    """
    hx, hy = (int(v) for v in hmd_pos.split(','))
    tx, ty = (int(v) for v in tower_pos.split(','))
    ww, wh = (int(v) for v in size.split(','))
    return ComposableNode(
        package='arts_display',
        plugin='arts_display::StereoDisplay',
        name='stereo_display',
        parameters=[{'fullscreen': fullscreen, 'overlay': overlay, 'hmd': hmd,
                     'recorder_present': recorder_present,
                     'hmd_x': hx, 'hmd_y': hy, 'tower_x': tx, 'tower_y': ty,
                     'window_width': ww, 'window_height': wh,
                     'pair_tolerance_ms': pair_tolerance_ms}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def _make_standalone_camera_node(name, serial, parameter_file, mode_params):
    return Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name=name,
        namespace=NAMESPACE,
        parameters=_camera_params(name, serial, parameter_file, mode_params),
    )


# The driver reports a refused camera write only as a WARN and keeps running, so a session can
# record on whatever the camera's flash booted (observed 2026-10-28: IspEnable refused on the
# first launch, camera still at 30.00 Hz). These are every phrasing camera.cpp uses for a
# parameter it could not apply (set*/execute, a Spinnaker exception in parameterChanged,
# a malformed definition entry), and none of its runtime warnings, which must not stop a
# recording. Floats only warn beyond a 2.5 % mismatch. The exception form was missed by the
# first version of this list and found by exercising it with an out-of-range gain.
_CONFIG_FAILURE_MARKERS = (
    ' instead of: ', ' failed: ', ']: param ', 'bad non-', 'invalid node type',
    'while driver is not ready', 'ignoring parameter', 'error reading parameter definitions',
    'lists no parameters',
)


def _shutdown_on_config_failure(event):
    for line in event.text.decode(errors='replace').splitlines():
        if any(m in line for m in _CONFIG_FAILURE_MARKERS):
            reason = 'camera configuration not applied: ' + line.strip()
            return [LogInfo(msg='[cameras.launch] ' + reason + ' (strict_config:=false to '
                                'run anyway)'),
                    EmitEvent(event=Shutdown(reason=reason))]
    return None


def _strict(actions):
    """Attach the configuration-failure watcher to each camera process."""
    return actions + [
        RegisterEventHandler(OnProcessIO(target_action=a,
                                         on_stdout=_shutdown_on_config_failure,
                                         on_stderr=_shutdown_on_config_failure))
        for a in actions
    ]


def _launch_setup(context, *args, **kwargs):
    triggered = LaunchConfiguration('trigger').perform(context).lower() == 'true'
    composed = LaunchConfiguration('composed').perform(context).lower() == 'true'
    strict = LaunchConfiguration('strict_config').perform(context).lower() == 'true'
    wrap = _strict if strict else (lambda actions: actions)

    if triggered:
        param_file = PathJoinSubstitution(
            [FindPackageShare('arts_driver'), 'config', 'blackfly_s_arts.yaml']
        ).perform(context)
        rate = float(LaunchConfiguration('rate').perform(context))
        mode_params = _triggered_params(
            rate,
            float(LaunchConfiguration('exposure_time').perform(context)),
            float(LaunchConfiguration('gain').perform(context)),
            float(LaunchConfiguration('trigger_delay').perform(context)),
            LaunchConfiguration('trigger_activation').perform(context),
            LaunchConfiguration('enable_3v3').perform(context).lower() == 'true',
            float(LaunchConfiguration('white_balance_red').perform(context)),
            float(LaunchConfiguration('white_balance_blue').perform(context)),
            float(LaunchConfiguration('gamma').perform(context)),
            LaunchConfiguration('trigger_overlap').perform(context),
            int(LaunchConfiguration('binning').perform(context)),
        )
    else:
        param_file = PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'config', 'blackfly_s.yaml']
        ).perform(context)
        mode_params = FREERUN_PARAMS

    serial_left = LaunchConfiguration('serial_left').perform(context)
    serial_right = LaunchConfiguration('serial_right').perform(context)
    enable_left = LaunchConfiguration('enable_left').perform(context).lower() == 'true'
    enable_right = LaunchConfiguration('enable_right').perform(context).lower() == 'true'

    cameras = []
    if enable_left and serial_left:
        cameras.append(('stereo_left', serial_left))
    if enable_right and serial_right:
        cameras.append(('stereo_right', serial_right))

    image_bag = LaunchConfiguration('image_bag').perform(context)
    footswitch = LaunchConfiguration('footswitch_teleop').perform(context).lower() == 'true'
    recorder = []
    if image_bag:
        extra = LaunchConfiguration('image_bag_extra_topics').perform(context).split()
        if footswitch:
            # Episode marks belong in the state bag; until it exists they go with the images.
            extra += ['/console/episode_mark', '/console/footswitch_teleop']
        paused = LaunchConfiguration('image_bag_paused').perform(context).lower() == 'true'
        recorder = [_make_image_recorder(image_bag, paused, extra)]
    guard = []
    if image_bag:
        # Separate small process: it reads disk usage, never images.
        guard = [Node(
            package='arts_record', executable='disk_guard.py', name='disk_guard',
            output='screen',
            arguments=['--path', image_bag, '--reserve-gb',
                       LaunchConfiguration('image_bag_reserve_gb').perform(context)],
        )]
    if footswitch:
        # Own process: it reads a USB HID device and calls the recorder's services, never
        # touches images. The mapping file is the decision-9 input mapping.
        mapping = (LaunchConfiguration('input_mapping').perform(context)
                   or FindPackageShare('arts_teleop').perform(context)
                   + '/config/input_mapping.yaml')
        guard.append(Node(
            package='arts_teleop', executable='footswitch.py', name='footswitch_teleop',
            output='screen',
            parameters=[{'mapping_file': mapping, 'recorder': '/image_recorder'}],
        ))
    if LaunchConfiguration('display').perform(context).lower() == 'true':
        recorder.append(_make_display(
            LaunchConfiguration('display_fullscreen').perform(context).lower() == 'true',
            LaunchConfiguration('display_overlay').perform(context).lower() == 'true',
            LaunchConfiguration('display_hmd_pos').perform(context),
            LaunchConfiguration('display_tower_pos').perform(context),
            LaunchConfiguration('display_hmd').perform(context).lower() == 'true',
            LaunchConfiguration('display_window_size').perform(context),
            recorder_present=bool(image_bag),
            # Free-running, the two cameras' clocks drift: with the triggered 5 ms tolerance
            # the view froze once their phase passed 5 ms (2026-11-05). Untriggered, pair the
            # nearest frames, up to half a free-run period.
            pair_tolerance_ms=5.0 if triggered else 500.0 / FREERUN_PARAMS['frame_rate']))

    if not composed:
        if recorder:
            raise RuntimeError('image_bag and display need composed:=true: they must '
                               'share the camera process')
        return wrap([
            _make_standalone_camera_node(name, serial, param_file, mode_params)
            for name, serial in cameras
        ])

    # A container holding one node is deliberately not special-cased. Single-camera rungs
    # would otherwise exercise a different process layout than the stereo pair, and then the
    # thing that differs between a working single-camera run and a failing stereo run would
    # be two changes rather than one.
    return guard + wrap([
        ComposableNodeContainer(
            name=CONTAINER_NAME,
            namespace='',
            package='rclcpp_components',
            # Plain (single-threaded) container, matching upstream's stereo example. See the
            # module docstring: publishing happens on each camera's own acquisition thread,
            # so the executor is nearly idle and _mt would buy nothing.
            executable='component_container',
            composable_node_descriptions=[
                _make_composable_camera_node(name, serial, param_file, mode_params)
                for name, serial in cameras
            ] + recorder,
            output='screen',
        )
    ])


def generate_launch_description():
    return LaunchDescription([
        # Roles as displayed after the 180° view rotation, confirmed 2026-10-28 by disparity
        # sign on paired frames (xL - xR = +22.9 px with these roles) and by the operator.
        # 24057132 is on sync-box terminal J1 (capture channel cam0), so cam0 is RIGHT.
        DeclareLaunchArgument('serial_left', default_value='24057131',
                              description='Serial of left stereo camera (DECIMAL, as '
                                          'Spinnaker reports it)'),
        DeclareLaunchArgument('serial_right', default_value='24057132',
                              description='Serial of right stereo camera (sysfs shows it in '
                                          'hex as 016F152C)'),
        DeclareLaunchArgument('enable_left', default_value='true',
                              description='Launch left camera'),
        DeclareLaunchArgument('enable_right', default_value='true',
                              description='Launch right camera'),
        DeclareLaunchArgument('strict_config', default_value='true',
                              description='Shut the launch down if the driver reports any '
                                          'camera write or command it could not apply. '
                                          'false only to diagnose such a failure'),
        DeclareLaunchArgument('trigger', default_value='false',
                              description='Hardware-trigger the cameras from the ESP32 sync '
                                          'box on Line 0 instead of free-running'),
        DeclareLaunchArgument('composed', default_value='true',
                              description='Run both cameras as composable nodes in ONE '
                                          'process (default, and the only supported layout '
                                          '— Spinnaker does not support two processes '
                                          'sharing the SDK). Setting false restores the '
                                          'two-process layout ONLY to reproduce the '
                                          'correlated 12-15 Hz dropouts of rung 5e for an '
                                          'A/B measurement. Delete this argument once that '
                                          'comparison is a committed benchmark'),
        DeclareLaunchArgument('rate', default_value='30.0',
                              description='Sync box pulse rate in Hz. Configures the '
                                          'diagnostics window ONLY; the ESP32 firmware sets '
                                          'the actual rate. Must match '
                                          'CONFIG_ARTS_SYNC_TRIGGER_RATE_HZ'),
        DeclareLaunchArgument('exposure_time', default_value='5000.0',
                              description='Fixed exposure in microseconds, triggered mode. '
                                          'Scene-dependent; what matters for sync is that '
                                          'it is fixed, not its value. Must stay well under '
                                          'the trigger period'),
        DeclareLaunchArgument('gain', default_value='10.0',
                              description='Fixed gain in dB, triggered mode. Raise this '
                                          'rather than the exposure if the image is dark'),
        # Colour preset, triggered mode. Defaults are collaborator A's tuning from the
        # cameras' saved UserSet1 (identical on both), kept because auto white balance drifts
        # and that drift would be recorded. Only the parts that work with the ISP off are
        # taken: the ISP caps these cameras at 17.25 fps, so UserSet1's colour matrix,
        # sharpening and light-source preset are not applied (do colour correction on the
        # host if needed). To re-tune for another light source or camera, point the scope at
        # a grey target and adjust the two ratios until the R, G and B Bayer means match;
        # docs/modules/pilot_flir_camera.md, "Colour preset", has the procedure.
        DeclareLaunchArgument('white_balance_red', default_value='0.927368',
                              description='BalanceRatio Red/Green, triggered mode. Default: '
                                          'collaborator A preset'),
        DeclareLaunchArgument('white_balance_blue', default_value='2.98132',
                              description='BalanceRatio Blue/Green, triggered mode. Default: '
                                          'collaborator A preset'),
        DeclareLaunchArgument('gamma', default_value='0.4',
                              description='Camera gamma, triggered mode, applied in-camera to '
                                          'the recorded Bayer data. Default: collaborator A '
                                          'preset (factory 0.8; about 3x brighter mid-tones)'),
        DeclareLaunchArgument('trigger_delay', default_value='379.0',
                              description='TriggerDelay in microseconds, triggered mode. '
                                          '379 is the FLOOR on BFS-U3-63S4C, not merely the '
                                          'default: the camera rejects 9 with '
                                          'OutOfRangeException, so upstream\'s "value >= 9" '
                                          'comment does not apply to this model. Set it '
                                          'identically on both cameras — it is per-camera, '
                                          'and a mismatch shows up as skew that is real as '
                                          'a measurement and meaningless as physics'),
        DeclareLaunchArgument('binning', default_value='2',
                              description='Recording resolution, triggered mode: 2 = 2x2 '
                                          'averaging binning, 1536x1024 raw Bayer, full field '
                                          'of view (default); 1 = full 3072x2048, four times '
                                          'the data rate'),
        DeclareLaunchArgument('image_bag', default_value='',
                              description='If set, record images, camera_info and meta to '
                                          'this MCAP bag path with a recorder composed into '
                                          'the camera process (empty = no recorder)'),
        DeclareLaunchArgument('display', default_value='false',
                              description='Surgeon display (HMD side-by-side + tower view) '
                                          'composed into the camera process; needs DISPLAY'),
        DeclareLaunchArgument('display_fullscreen', default_value='false',
                              description='Display windows fullscreen'),
        DeclareLaunchArgument('display_window_size', default_value='1920,1080',
                              description='Window size "W,H" when not fullscreen (E7 bench)'),
        DeclareLaunchArgument('display_hmd', default_value='true',
                              description='Open the HMD window (false: tower view only)'),
        DeclareLaunchArgument('display_hmd_pos', default_value='0,0',
                              description='HMD window position "X,Y": the headset output\'s '
                                          'desktop offset (xrandr --listmonitors)'),
        DeclareLaunchArgument('display_tower_pos', default_value='0,0',
                              description='Tower window position "X,Y": the tower '
                                          'monitor\'s desktop offset'),
        DeclareLaunchArgument('display_overlay', default_value='false',
                              description='Print frame age on the tower view (bench use)'),
        DeclareLaunchArgument('image_bag_reserve_gb', default_value='30',
                              description='disk_guard stops the image recorder when free '
                                          'space on the bag disk falls below this (GB)'),
        DeclareLaunchArgument('image_bag_paused', default_value='false',
                              description='Start the image recorder paused; resume with the '
                                          '~/resume service (footswitch_teleop:=true: the record pedal)'),
        DeclareLaunchArgument('footswitch_teleop', default_value='false',
                              description='Run the footswitch node (arts_teleop): pedal '
                                          'state, episode marks, record pedal toggles the '
                                          'image recorder; pair with image_bag_paused:=true'),
        DeclareLaunchArgument('input_mapping', default_value='',
                              description='Input mapping file (default: arts_teleop '
                                          'config/input_mapping.yaml)'),
        DeclareLaunchArgument('image_bag_extra_topics', default_value='',
                              description='Space-separated extra topics for the image bag '
                                          '(bench use, e.g. /ecm/stereo_sync/pulse)'),
        DeclareLaunchArgument('trigger_overlap', default_value='ReadOut',
                              description='TriggerOverlap, triggered mode: ReadOut accepts a '
                                          'trigger during readout, Off does not'),
        DeclareLaunchArgument('trigger_activation', default_value='RisingEdge',
                              description='Edge the camera fires on. Exposed as an argument '
                                          'because the sense of an opto-isolated input is '
                                          'not self-evident, so trying FallingEdge is the '
                                          'cheapest way to rule polarity out when a '
                                          'correctly wired trigger produces no frames'),
        DeclareLaunchArgument('enable_3v3', default_value='false',
                              description='BENCH DIAGNOSTIC. Enables the 3.3 V rail on '
                                          'Line 2 (red wire, referenced to brown/pin 6) so '
                                          'a meter can confirm the pigtail colour-to-pin '
                                          'mapping. Off by default'),
        OpaqueFunction(function=_launch_setup),
    ])
