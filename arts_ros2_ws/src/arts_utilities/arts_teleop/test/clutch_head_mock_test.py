#!/usr/bin/env python3
"""Teleop follower in the CLINICAL convention on a mock UR5e: clutch mode, head-sensor enable.

    arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/clutch_head_mock_test.py

Starts teleop_follower with mode clutch and enable_source head_sensor (position only, the rig
and mapping of teleop_follower_mock_test.py: handle +x -> arm +y, scale 0.25). Plays the
handle, the footswitch and /console/head_sensor at 20 Hz. Checks, in order:
  1. no head, pedal up: handle +20 mm, the arm does not move
  2. head in, pedal up: handle +40 mm -> arm +10 mm along +y
  3. pedal DOWN (clutch): handle +40 mm, the arm stays (the hand repositions)
  4. pedal up again: no jump; handle -40 mm -> arm -10 mm
  5. head out (present 0): handle moves, the arm stays
  6. head in, then the sensor goes SILENT: after head_stale_s the arm stays (silence is not a
     head)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import rclpy
from sensor_msgs.msg import Joy

from teleop_follower_mock_test import BOX, R_TEST, SCALE, Rig, check, move, spin_for, FAILS


class HeadRig(Rig):

    def __init__(self):
        super().__init__()
        self.head_pub = self.create_publisher(Joy, '/console/head_sensor', 10)
        self.head = 0          # 1 present, 0 absent, None silent
        self.pedal_down = False
        self.create_timer(0.05, self._tick_head)

    def _tick_head(self):
        if self.head is not None:
            j = Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.buttons = [int(self.head)]
            self.head_pub.publish(j)
        self.pedal(self.pedal_down)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mapping = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    mapping.write('footswitch_teleop:\n  device: none\n  pedals: {}\n'
                  '  actions: {left: clutch, middle: camera, right: record_toggle}\n')
    mapping.close()
    follower = subprocess.Popen(
        [sys.executable, os.path.join(here, '..', 'scripts', 'teleop_follower.py'),
         '--ros-args', '-p', 'arm:=psm_left', '-p', 'mtm:=mtm_left', '-p', 'mode:=clutch',
         '-p', 'enable_source:=head_sensor', '-p', f'scale:={SCALE}', '-p', f'max_offset:={BOX}',
         '-p', f'operator_to_base:={R_TEST}', '-p', f'mapping_file:={mapping.name}'])
    rclpy.init()
    rig = HeadRig()
    try:
        spin_for(rig, 4.0)

        print('[1] no head, pedal up, handle +20 mm x')
        a = rig.arm()
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.arm() - a) * 1e3
        check('no head: arm still', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')

        print('[2] head in, pedal up, handle +40 mm x')
        rig.head = 1
        spin_for(rig, 0.5)
        a = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a) * 1e3
        check('head in, pedal up: arm +10 mm y', abs(d[1] - 10.0) < 1.0 and abs(d[0]) < 1.0 and
              abs(d[2]) < 1.0, f'{np.round(d, 2)} mm')

        print('[3] pedal down (clutch), handle +40 mm x')
        rig.pedal_down = True
        spin_for(rig, 0.3)
        a = rig.arm()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 0.5)
        d = (rig.arm() - a) * 1e3
        check('clutch: arm still while the hand repositions', np.linalg.norm(d) < 0.2,
              f'{np.round(d, 2)} mm')

        print('[4] pedal up again, then handle -40 mm x')
        rig.pedal_down = False
        spin_for(rig, 0.5)
        a = rig.arm()
        check('no jump at re-engagement', np.linalg.norm((rig.arm() - a) * 1e3) < 0.2, 'ok')
        move(rig, [-0.04, 0, 0], 1.0)
        spin_for(rig, 1.0)
        d = (rig.arm() - a) * 1e3
        check('re-engaged: arm -10 mm y', abs(d[1] + 10.0) < 1.0, f'{np.round(d, 2)} mm')

        print('[5] head out, handle +20 mm x')
        rig.head = 0
        spin_for(rig, 0.3)
        a = rig.arm()
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.arm() - a) * 1e3
        check('head out: arm still', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')

        print('[6] head in, then the sensor goes silent; handle +20 mm x')
        rig.head = 1
        spin_for(rig, 0.5)
        rig.head = None
        spin_for(rig, 0.5)
        a = rig.arm()
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.arm() - a) * 1e3
        check('silent sensor: arm still', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')
    finally:
        follower.send_signal(2)
        try:
            follower.wait(timeout=5)
        except subprocess.TimeoutExpired:
            follower.kill()
        os.unlink(mapping.name)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
