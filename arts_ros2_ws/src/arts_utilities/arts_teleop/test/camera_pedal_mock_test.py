#!/usr/bin/env python3
"""Camera control from the middle pedal on a mock camera arm (ecm).

    ARTS_ARM=ecm arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/camera_pedal_mock_test.py

Starts teleop_follower for psm_left (not running here; only its teleop_state is read) in
deadman mode, room frame operator_to_base = handle +x -> base +y, camera_arm ecm, scale 0.5.
Publishes psm_left_base_link at identity under ecm_base_link, so the mapped direction is the
same in the camera arm's base. Checks:
  1. middle pedal up: handle +20 mm x, the camera does not move
  2. middle pedal down: handle +40 mm x -> camera tip +20 mm along +y (scale 0.5)
  3. middle pedal released: handle moves, the camera stays
  4. left pedal held (deadman) with the middle pedal held: psm_left NOT engaged; middle
     released: engaged
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from tf2_ros import StaticTransformBroadcaster

from arts_interfaces.msg import TeleopState, ToolTipState
from teleop_follower_mock_test import R_TEST, Rig, check, move, spin_for, FAILS


class CamRig(Rig):

    def __init__(self):
        super().__init__()
        self.pedals = [0, 0, 0]
        self.cam = None
        self.engaged = None
        self.create_subscription(ToolTipState, '/ecm/tool_tip_state', self._on_cam, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TeleopState, '/psm_left/teleop_state',
                                 lambda m: setattr(self, 'engaged', m.clutch_engaged), latched)
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'ecm_base_link', 'psm_left_base_link'
        t.transform.rotation.w = 1.0
        self.static.sendTransform(t)
        self.create_timer(0.02, self._tick_pedals)

    def _on_cam(self, m):
        p = m.pose.position
        self.cam = np.array([p.x, p.y, p.z])

    def _tick_pedals(self):
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.buttons = list(self.pedals)
        self.joy.publish(j)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mapping = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    mapping.write('footswitch_teleop:\n  device: none\n  pedals: {}\n'
                  '  actions: {left: clutch, middle: camera, right: record_toggle}\n')
    mapping.close()
    follower = subprocess.Popen(
        [sys.executable, os.path.join(here, '..', 'scripts', 'teleop_follower.py'),
         '--ros-args', '-p', 'arm:=psm_left', '-p', 'mtm:=mtm_left', '-p', 'mode:=deadman',
         '-p', 'camera_arm:=ecm', '-p', 'camera_scale:=0.5',
         '-p', f'operator_to_base:={R_TEST}', '-p', f'mapping_file:={mapping.name}'])
    rclpy.init()
    rig = CamRig()
    try:
        spin_for(rig, 4.0)

        print('[1] middle pedal up, handle +20 mm x')
        c = rig.cam.copy()
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.cam - c) * 1e3
        check('camera still without the pedal', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')

        print('[2] middle pedal down, handle +40 mm x')
        rig.pedals = [0, 1, 0]
        spin_for(rig, 0.5)
        c = rig.cam.copy()
        move(rig, [0.04, 0, 0], 1.0)
        spin_for(rig, 1.5)
        d = (rig.cam - c) * 1e3
        check('camera +20 mm along +y (scale 0.5)', abs(d[1] - 20.0) < 1.0 and abs(d[0]) < 1.0 and
              abs(d[2]) < 1.0, f'{np.round(d, 2)} mm')

        print('[3] middle pedal released, handle +20 mm x')
        rig.pedals = [0, 0, 0]
        spin_for(rig, 0.5)
        c = rig.cam.copy()
        move(rig, [0.02, 0, 0], 0.5)
        spin_for(rig, 0.5)
        d = (rig.cam - c) * 1e3
        check('camera stays after release', np.linalg.norm(d) < 0.2, f'{np.round(d, 2)} mm')

        print('[4] left (deadman) + middle held, then middle released')
        rig.pedals = [1, 1, 0]
        spin_for(rig, 0.5)
        check('psm_left not engaged during camera control', rig.engaged is False,
              f'clutch_engaged {rig.engaged}')
        rig.pedals = [1, 0, 0]
        spin_for(rig, 0.5)
        check('psm_left engaged after the camera pedal', rig.engaged is True,
              f'clutch_engaged {rig.engaged}')
        rig.pedals = [0, 0, 0]
        spin_for(rig, 0.3)
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
