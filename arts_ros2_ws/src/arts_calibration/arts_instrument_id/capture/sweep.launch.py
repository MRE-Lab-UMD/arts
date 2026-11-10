"""Stage B/C capture: joint sweeps and random poses with fusionTrack recording.

STUB. Blocked on arts_dv_tool_driver and on activating the atracsys middle overlay.

The contract this must honour when it is written:
  - sweep DISTAL TO PROXIMAL in the order given by the instrument config (jaw, yaw, pitch,
    roll), so each circle fit only sees joints already characterised
  - every sweep bidirectional; the gap between branches is the backlash and it is a
    published number, not a nuisance
  - record the tip fiducial as a POSITION stream and the base geometry as a POSE stream.
    A single sphere at the tip is sufficient — circle-point analysis needs points, not
    poses, which is what makes the rig buildable at all
  - repeat the pitch/yaw sweeps at each roll angle in the config, to test whether a constant
    coupling matrix is the right model
  - record commanded disc angles time-aligned with tracker samples; misalignment shows up as
    apparent backlash and would silently inflate a headline number

Output: synchronised (q_actuator, tip_position, base_pose, direction_flag) rows.

Usage once implemented:
    ros2 launch arts_instrument_id sweep.launch.py \
        instrument:=monopolar_curved_scissors mode:=single_disc out:=<capture dir>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    instrument = LaunchConfiguration('instrument')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'instrument',
            default_value='monopolar_curved_scissors',
            description='name matching a file in config/',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='single_disc',
            choices=['single_disc', 'random_poses', 'pivot'],
            description='single_disc initialises the fits; random_poses tests superposition; '
                        'pivot establishes the tool frame origin',
        ),
        DeclareLaunchArgument(
            'out',
            default_value='/tmp/arts_instrument_id',
            description='directory for raw capture data',
        ),
        LogInfo(msg=[
            'arts_instrument_id sweep capture (', mode, ') for ', instrument,
            ' is NOT IMPLEMENTED — blocked on arts_dv_tool_driver and the atracsys overlay. ',
            'See docs/instrument_identification.md, Stages B and C.',
        ]),
    ])
