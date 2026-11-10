"""Stage A capture: repeated hard-stop homing, for repeatability statistics.

STUB. Blocked on two things that do not exist yet:
  - arts_embedded/arts_dv_tool_driver, which owns the Dynamixel bus and therefore the
    present-current feedback the hard-stop search depends on
  - the atracsys middle overlay, not yet activated (not needed for Stage A, but the same
    bringup is shared with sweep.launch.py)

The contract this must honour when it is written:
  - drive each disc at the single fixed approach velocity from the instrument config;
    detection lag grows with approach speed, so it may not be tuned per disc
  - measure free-running current per disc first, and set the trigger as a ratio of it
  - warm up before recording, since Dynamixel current drifts as the winding warms
  - record RAW per-cycle stops, never a pre-averaged zero — the statistics are the result
  - prompt the operator to re-seat the instrument between seating groups, and stamp
    seating_id into the output, because re-seat repeatability is the operationally
    relevant number and it is invisible if the groups are not labelled

Output: one row per cycle, matching arts_instrument_id.homing.HomingCycle.

Usage once implemented:
    ros2 launch arts_instrument_id homing.launch.py \
        instrument:=monopolar_curved_scissors out:=<capture dir>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    instrument = LaunchConfiguration('instrument')

    return LaunchDescription([
        DeclareLaunchArgument(
            'instrument',
            default_value='monopolar_curved_scissors',
            description='name matching a file in config/',
        ),
        DeclareLaunchArgument(
            'out',
            default_value='/tmp/arts_instrument_id',
            description='directory for raw capture data',
        ),
        LogInfo(msg=[
            'arts_instrument_id homing capture for ', instrument,
            ' is NOT IMPLEMENTED — blocked on arts_dv_tool_driver. ',
            'See docs/instrument_identification.md, Stage A.',
        ]),
    ])
