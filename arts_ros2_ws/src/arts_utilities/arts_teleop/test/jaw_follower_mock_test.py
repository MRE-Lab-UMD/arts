#!/usr/bin/env python3
"""jaw_follower against faked inputs: no bridge, no disc_drive, no hardware.

    source arts_ros2_ws/install/setup.bash
    python3 arts_ros2_ws/src/arts_utilities/arts_teleop/test/jaw_follower_mock_test.py

Fakes /mtm_left/cursor (liveness), /mtm_left/stylus_state (grip), /psm_left/tool_joint_states
(the measured jaw) and /psm_left/teleop_state, and reads /psm_left/tool_joint_target. Needle
driver file; A's Hall range 10..29, full grip at 9. Reads /psm_left/tool_jaw_target (the
unitless jaw command). Checks:

  [1] gate always, grip 29 (open): command 90
  [2] grip 10 (just closed): 0
  [2b] grip 9 and grip 7 (released lever): -10, full grip
  [3] grip 19.5 (half): 45
  [4] grip beyond the range (40): clamped to 90
  [5] the bridge goes silent: nothing more
  [6] gate teleop, not engaged: nothing; engaged: follows; disengaged again: holds the command
"""
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from arts_interfaces.msg import StylusState, TeleopState

FAILS = []


class Rig(Node):

    def __init__(self):
        super().__init__('jaw_follower_mock_test')
        self.cursor = self.create_publisher(PointStamped, '/mtm_left/cursor', 10)
        self.stylus = self.create_publisher(StylusState, '/mtm_left/stylus_state', 10)
        self.joints = self.create_publisher(JointState, '/psm_left/tool_joint_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.teleop = self.create_publisher(TeleopState, '/psm_left/teleop_state', latched)
        self.out = []
        self.create_subscription(Float64, '/psm_left/tool_jaw_target',
                                 lambda m: self.out.append(m.data), 10)

    def feed(self, seconds, grip=None, jaw=0.0, cursor=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cursor:
                self.cursor.publish(PointStamped())
            if grip is not None:
                self.stylus.publish(StylusState(grip=float(grip)))
            self.joints.publish(JointState(
                name=['psm_left_tool_roll', 'psm_left_tool_wrist_pitch',
                      'psm_left_tool_wrist_yaw', 'psm_left_tool_jaw'],
                position=[0.0, 0.0, 0.0, jaw]))
            rclpy.spin_once(self, timeout_sec=0.01)


def check(name, ok, detail):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        FAILS.append(name)


def last(rig):
    return rig.out[-1] if rig.out else None


def start(gate):
    # The installed script run directly (not `ros2 run`, which does not pass SIGINT on).
    script = os.path.join(get_package_prefix('arts_teleop'), 'lib', 'arts_teleop',
                          'jaw_follower.py')
    return subprocess.Popen([sys.executable, script, '--ros-args',
                             '-p', f'gate:={gate}',
                             '-p', 'instrument_file:=471006_large_needle_driver.yaml'])


def main():
    rclpy.init()
    rig = Rig()
    proc = start('always')
    try:
        rig.feed(3.0, grip=29, jaw=0.0)  # start-up
        for name, grip, want in [('[1] grip 29 (open)', 29, 90.0), ('[2] grip 10 (just closed)', 10, 0.0),
                                 ('[2b] grip 9 (released lever)', 9, -10.0), ('[2b] grip 7', 7, -10.0),
                                 ('[3] grip 19.5 (half)', 19.5, 45.0), ('[4] grip 40', 40, 90.0)]:
            print(name)
            rig.feed(0.5, grip=grip, jaw=0.0)
            check('command', last(rig) is not None and abs(last(rig) - want) < 1e-9,
                  f'{last(rig)} vs {want}')
        print('[5] bridge silent')
        rig.feed(1.0, jaw=0.5, cursor=False)
        n = len(rig.out)
        rig.feed(1.0, jaw=0.5, cursor=False)
        check('then nothing', len(rig.out) == n, f'{len(rig.out) - n} more messages')
    finally:
        proc.send_signal(2)
        proc.wait(timeout=5)
    proc = start('teleop')
    try:
        print('[6] gate teleop')
        rig.teleop.publish(TeleopState(clutch_engaged=False))
        rig.feed(3.0, grip=29, jaw=0.0)
        rig.out = []
        rig.feed(0.5, grip=29, jaw=0.0)
        check('not engaged: nothing', not rig.out, f'{len(rig.out)} messages')
        rig.teleop.publish(TeleopState(clutch_engaged=True))
        rig.feed(0.5, grip=9, jaw=0.0)
        check('engaged: follows (grip)', last(rig) == -10.0, f'{last(rig)}')
        rig.teleop.publish(TeleopState(clutch_engaged=False))
        rig.feed(0.5, grip=29, jaw=0.0)
        check('disengaged: holds the grip', last(rig) == -10.0, f'{last(rig)}')
    finally:
        proc.send_signal(2)
        proc.wait(timeout=5)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main() or 0)
