#!/usr/bin/env python3
"""gamepad_jog against a mock arm: a scripted gamepad, no joy_node, no hardware.

    ARTS_ARM=psm_left arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh \\
        $PWD/arts_ros2_ws/src/arts_utilities/arts_teleop/test/gamepad_jog_mock_test.py

Starts gamepad_jog.py, plays /console/gamepad at 50 Hz, and measures the tip from
/psm_left/tool_tip_state. The arm runs the da Vinci X tip preset (as arts_bringup starts the
PSMs), so the shaft is the tip frame's z. Checks:

  [1] stick deflected, RB NOT held: no motion (the deadman)
  [2] RB held, left stick full across for 1 s: the tip moves ~20 mm (A's 20 mm/s) across the
      shaft and not along it
  [3] RB released: the tip stops within a few mm of where it was
  [4] RB held, LT full for 1 s (RCM off): the tip moves ~20 mm ALONG the shaft
  [4b] RB held, right stick Y full for 1 s, then d-pad X for 1 s (RCM off): the tip turns
      ~0.3 rad (wrist_angular_velocity) about its own x, then about its own y, and stays put
  [5] RB held, then the gamepad goes silent: motion stops after stale_s (0.5 s)
  [6] RCM on, RB held, left stick full for 1 s: the shaft pivots about the RCM point (the
      controller's rcm_state deviation stays small) and the tip moves on the sphere
  [7] Start pressed: psm_right selected; RB on psm_left then moves nothing
  [8] LB + B, left stick full: instrument-joint mode, roll at tool_jog_rad_s (A: pi rad/s) on
      /psm_left/tool_joint_target, RT released = jaw command -10 (full grip) on
      /psm_left/tool_jaw_target, no jaw in the joint target, nothing on
      tool_actuator_target
  [9] LB released: the last joint target is zero
  [10] LB + A, left stick full: back to disc mode, disc 1 at 10 rpm on tool_actuator_target
"""
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Joy

from arts_interfaces.msg import RcmState, ToolTipState
from arts_interfaces.srv import SetRcm, SetToolTipOffset

TIP = [0.0, 0.609, 0.0885, -0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
FAILS = []


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Rig(Node):

    def __init__(self):
        super().__init__('gamepad_jog_mock_test')
        self.pad = self.create_publisher(Joy, '/console/gamepad', 10)
        self.tip = None
        self.rcm = None
        self.create_subscription(ToolTipState, '/psm_left/tool_tip_state', self._tip, 10)
        self.create_subscription(RcmState, '/psm_left/rcm_state', self._rcm, 10)
        self.set_rcm = self.create_client(SetRcm, '/psm_left/set_rcm')
        self.set_tip = self.create_client(SetToolTipOffset, '/psm_left/set_tool_tip_offset')
        self.devs = None
        self.joint_t = []
        self.disc_t = []
        self.jaw_t = []
        self.create_subscription(Float64, '/psm_left/tool_jaw_target',
                                 lambda m: self.jaw_t.append(m.data), 10)
        self.create_subscription(JointState, '/psm_left/tool_joint_target',
                                 lambda m: self.joint_t.append(m), 10)
        self.create_subscription(JointState, '/psm_left/tool_actuator_target',
                                 lambda m: self.disc_t.append(m), 10)
        self.fake_joints = self.create_publisher(JointState, '/psm_left/tool_joint_states', 10)

    def _tip(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.tip = (np.array([p.x, p.y, p.z]), quat_to_matrix(o.x, o.y, o.z, o.w))

    def _rcm(self, m):
        self.rcm = m
        if self.devs is not None:
            self.devs.append(m.deviation)

    def send(self, rb=False, left_x=0.0, lt=0.0, start=False, lb=False, a=False, b=False,
             right_y=0.0, hat_x=0.0):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [left_x, 0.0, 1.0 - 2.0 * lt, 0.0, right_y, 1.0, hat_x, 0.0]
        m.buttons = [0] * 11
        m.buttons[0] = int(a)
        m.buttons[1] = int(b)
        m.buttons[4] = int(lb)
        m.buttons[5] = int(rb)
        m.buttons[7] = int(start)
        self.pad.publish(m)
        # A measured jaw of 0.2 rad, as disc_drive would publish with an instrument file.
        self.fake_joints.publish(JointState(
            name=['psm_left_tool_roll', 'psm_left_tool_wrist_pitch', 'psm_left_tool_wrist_yaw',
                  'psm_left_tool_jaw'], position=[0.0, 0.0, 0.0, 0.2]))

    def call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result()


def spin_for(rig, s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(rig, timeout_sec=0.005)


def play(rig, seconds, **kw):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rig.send(**kw)
        spin_for(rig, 0.02)


def check(name, ok, detail):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        FAILS.append(name)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    jog = subprocess.Popen([sys.executable, os.path.join(here, '..', 'scripts', 'gamepad_jog.py')])
    rclpy.init()
    rig = Rig()
    try:
        req = SetToolTipOffset.Request()
        req.offset = Pose()
        (req.offset.position.x, req.offset.position.y, req.offset.position.z,
         req.offset.orientation.x, req.offset.orientation.y, req.offset.orientation.z,
         req.offset.orientation.w) = TIP
        rig.set_tip.wait_for_service(timeout_sec=10.0)
        check('da Vinci X tip preset set', rig.call(rig.set_tip, req).success, '')
        spin_for(rig, 3.0)
        p0, r0 = rig.tip
        shaft = r0[:, 2]

        print('[1] stick across, RB not held')
        play(rig, 1.0, left_x=1.0)
        d = np.linalg.norm(rig.tip[0] - p0) * 1e3
        check('no motion without RB', d < 0.1, f'{d:.3f} mm')

        print('[2] RB + left stick full for 1 s')
        p1 = rig.tip[0]
        play(rig, 1.0, rb=True, left_x=1.0)
        play(rig, 0.5)  # release, settle
        dv = rig.tip[0] - p1
        along = abs(dv @ shaft) * 1e3
        across = np.linalg.norm(dv - (dv @ shaft) * shaft) * 1e3
        check('~20 mm across the shaft', 15.0 < across < 25.0, f'{across:.1f} mm')
        check('nothing along the shaft', along < 1.0, f'{along:.2f} mm')

        print('[3] released: stays')
        p2 = rig.tip[0]
        play(rig, 1.0, left_x=1.0)
        d = np.linalg.norm(rig.tip[0] - p2) * 1e3
        check('stays after release', d < 0.2, f'{d:.3f} mm')

        print('[4] RB + LT full for 1 s (RCM off)')
        p3 = rig.tip[0]
        play(rig, 1.0, rb=True, lt=1.0)
        play(rig, 0.5)
        dv = rig.tip[0] - p3
        check('~20 mm along the shaft (insert)', 15.0 < dv @ shaft * 1e3 < 25.0,
              f'{dv @ shaft * 1e3:.1f} mm')

        for name, kw, k in (('pitch, right stick Y', dict(right_y=1.0), 0),
                            ('yaw, d-pad X', dict(hat_x=-1.0), 1)):
            print(f'[4b] RB + {name} for 1 s')
            pa, ra = rig.tip
            play(rig, 1.0, rb=True, **kw)
            play(rig, 0.5)
            pb, rb_ = rig.tip
            rv = ra.T @ rb_                       # rotation in the start tip frame
            ang = math.acos(max(-1.0, min(1.0, (np.trace(rv) - 1.0) / 2.0)))
            axis = np.array([rv[2, 1] - rv[1, 2], rv[0, 2] - rv[2, 0], rv[1, 0] - rv[0, 1]])
            axis /= max(np.linalg.norm(axis), 1e-9)
            check(f'{name}: ~0.3 rad', 0.22 < ang < 0.38, f'{ang:.3f} rad')
            check(f'{name}: about the tip {"xy"[k]} axis', abs(axis[k]) > 0.95,
                  f'axis in the tip frame {np.round(axis, 3)}')
            d = np.linalg.norm(pb - pa) * 1e3
            check(f'{name}: tip stays', d < 1.0, f'{d:.2f} mm')

        print('[5] RB held, then the gamepad goes silent')
        play(rig, 0.3, rb=True, left_x=1.0)
        p4 = rig.tip[0]
        spin_for(rig, 1.5)  # silence: stale after 0.5 s
        d = np.linalg.norm(rig.tip[0] - p4) * 1e3
        check('stops on a silent gamepad', d < 15.0, f'{d:.1f} mm after the last message '
              '(<= 0.5 s at 20 mm/s plus settling)')
        p5 = rig.tip[0]
        spin_for(rig, 1.0)
        d = np.linalg.norm(rig.tip[0] - p5) * 1e3
        check('and stays stopped', d < 0.2, f'{d:.3f} mm')

        print('[6] RCM on, RB + left stick full for 1 s')
        r = rig.call(rig.set_rcm, SetRcm.Request(enable=True, from_current_shaft=True,
                                                 insertion_depth=0.10))
        check('RCM on', r.success, r.message)
        spin_for(rig, 0.3)
        c = np.array([r.applied_point.x, r.applied_point.y, r.applied_point.z])
        p6 = rig.tip[0]
        rig.devs = []
        play(rig, 1.0, rb=True, left_x=1.0)
        play(rig, 0.5)
        devs, rig.devs = np.array(rig.devs) * 1e3, None
        p7 = rig.tip[0]
        moved = np.linalg.norm(p7 - p6) * 1e3
        radius = abs(np.linalg.norm(p7 - c) - np.linalg.norm(p6 - c)) * 1e3
        check('tip moved (pivot)', moved > 5.0, f'{moved:.1f} mm')
        check('on the sphere about the RCM', radius < 0.5, f'depth change {radius:.2f} mm')
        check('RCM held', devs.max() < 0.1, f'max deviation {devs.max():.4f} mm')
        rig.call(rig.set_rcm, SetRcm.Request(enable=False))
        spin_for(rig, 0.3)

        print('[8] LB + B, left stick full: instrument-joint mode')
        play(rig, 0.1, lb=True, b=True)
        rig.joint_t, rig.disc_t, rig.jaw_t = [], [], []
        play(rig, 0.5, lb=True, left_x=1.0)
        last = rig.joint_t[-1] if rig.joint_t else None
        v = dict(zip(last.name, last.velocity)) if last else {}
        check('roll at A\'s pi rad/s', abs(v.get('roll', 0) - math.pi) < 1e-6,
              f"roll {v.get('roll')}")
        check('RT released: jaw command -10 (full grip) on tool_jaw_target',
              bool(rig.jaw_t) and rig.jaw_t[-1] == -10.0, f"{rig.jaw_t[-1:] or 'none'}")
        check('no jaw in the joint target', 'jaw' not in v, f'{sorted(v)}')
        check('pitch and yaw still', v.get('wrist_pitch') == 0.0 and v.get('wrist_yaw') == 0.0,
              f"{v.get('wrist_pitch')}, {v.get('wrist_yaw')}")
        check('no disc target in joint mode', not rig.disc_t, f'{len(rig.disc_t)} messages')

        print('[9] LB released')
        play(rig, 0.3)
        last = rig.joint_t[-1]
        check('last joint target zero', not any(last.velocity), f'{list(last.velocity)}')

        print('[10] LB + A, left stick full: disc mode')
        play(rig, 0.1, lb=True, a=True)
        rig.joint_t, rig.disc_t = [], []
        play(rig, 0.5, lb=True, left_x=1.0)
        play(rig, 0.3)
        d1 = [m.velocity[0] for m in rig.disc_t if any(m.velocity)]
        check('disc 1 at 10 rpm', bool(d1) and abs(d1[-1] - 10 * 2 * math.pi / 60) < 1e-6,
              f'{d1[-1] if d1 else None} rad/s')
        check('no joint target in disc mode', not rig.joint_t, f'{len(rig.joint_t)} messages')

        print('[7] Start: psm_right selected')
        play(rig, 0.1, start=True)
        play(rig, 0.1)
        p8 = rig.tip[0]
        play(rig, 1.0, rb=True, left_x=1.0)
        d = np.linalg.norm(rig.tip[0] - p8) * 1e3
        check('psm_left does not move', d < 0.1, f'{d:.3f} mm')
    finally:
        jog.send_signal(signal.SIGINT)  # SIGTERM races rclpy's context shutdown
        jog.wait(timeout=5)
        rclpy.shutdown()
    print('\nPASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
