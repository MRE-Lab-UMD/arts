#!/usr/bin/env python3
"""The controller's tip frame on tf matches the tip it servoes (mock hardware).

    ARTS_ARM=psm_left run_servo_smoke_test.sh tip_frame_mock_test.py

The servo controller publishes <arm>_tool0 -> <arm>_tool_tip on /tf_static with the offset it
is using, so an operator can see the tip in RViz. Checks:

  1. at start-up (tip:=flange), tf <arm>_base_link -> <arm>_tool_tip equals tool_tip_state;
  2. after set_tool_tip_offset to the da Vinci X wrist preset, tool0 -> tool_tip equals the
     requested offset and base_link -> tool_tip again equals tool_tip_state;
  3. the offset is restored to the flange, and the frame follows.

Failure path: TIP_CHECK_FRAME=<arm>_tool0 compares tool_tip_state against the flange instead,
which must fail check 2 by the preset's ~0.6 m.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from arm_io import ARM, BASE_FRAME, TOOL_FRAME, TOOL_TIP_FRAME, TOOL_TIP_STATE
from arts_interfaces.msg import ToolTipState
from arts_interfaces.srv import SetToolTipOffset

CHECK_FRAME = os.environ.get('TIP_CHECK_FRAME', TOOL_TIP_FRAME)
POS_TOL = 1e-4      # m
ANG_TOL = 1e-3      # rad


def preset(key):
    path = get_package_share_directory('arts_driver') + '/config/tool_tips.yaml'
    with open(path) as f:
        return yaml.safe_load(f)[key]['offset']


def quat_angle(a, b):
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))


class Rig(Node):

    def __init__(self):
        super().__init__('tip_frame_mock_test')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.tip = None
        self.create_subscription(ToolTipState, TOOL_TIP_STATE, self._on_tip, 10)
        self.offset_cli = self.create_client(SetToolTipOffset, f'/{ARM}/set_tool_tip_offset')

    def _on_tip(self, msg):
        self.tip = msg

    def lookup(self, parent, child):
        t = self.tf.lookup_transform(parent, child, Time()).transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def set_offset(self, off):
        req = SetToolTipOffset.Request()
        p, o = req.offset.position, req.offset.orientation
        p.x, p.y, p.z, o.x, o.y, o.z, o.w = map(float, off)
        fut = self.offset_cli.call_async(req)
        while not fut.done():
            time.sleep(0.01)
        return fut.result()


def wait_for(cond, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return
        time.sleep(0.05)
    raise TimeoutError(what)


def main():
    rclpy.init()
    rig = Rig()
    ex = SingleThreadedExecutor()
    ex.add_node(rig)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    failures = []

    def compare_to_state(label):
        time.sleep(0.5)   # the offset applies at the next cycle; tf_static arrives latched
        pos, quat = rig.lookup(BASE_FRAME, CHECK_FRAME)
        m = rig.tip.pose
        sp = np.array([m.position.x, m.position.y, m.position.z])
        sq = np.array([m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])
        dp, da = float(np.linalg.norm(pos - sp)), quat_angle(quat, sq)
        ok = dp < POS_TOL and da < ANG_TOL
        print(f'{"PASS" if ok else "FAIL"} {label}: tf {BASE_FRAME}->{CHECK_FRAME} vs '
              f'tool_tip_state: {dp * 1e3:.3f} mm, {math.degrees(da):.3f} deg')
        if not ok:
            failures.append(label)

    try:
        wait_for(lambda: rig.tip is not None, 10, 'tool_tip_state')
        wait_for(lambda: rig.tf.can_transform(BASE_FRAME, CHECK_FRAME, Time()), 10,
                 f'tf {BASE_FRAME}->{CHECK_FRAME}')
        wait_for(lambda: rig.offset_cli.service_is_ready(), 10, 'set_tool_tip_offset')

        compare_to_state('1 start-up (flange)')

        wrist = preset('da_vinci_x_wrist')
        r = rig.set_offset(wrist)
        if not r.success:
            print(f'FAIL 2: set_tool_tip_offset refused: {r.message}')
            return 1
        time.sleep(0.5)
        pos, quat = rig.lookup(TOOL_FRAME, CHECK_FRAME)
        dp = float(np.linalg.norm(pos - np.array(wrist[:3])))
        da = quat_angle(quat, np.array(wrist[3:]))
        ok = dp < POS_TOL and da < ANG_TOL
        print(f'{"PASS" if ok else "FAIL"} 2a {TOOL_FRAME}->{CHECK_FRAME} vs the requested '
              f'da_vinci_x_wrist offset: {dp * 1e3:.3f} mm, {math.degrees(da):.3f} deg')
        if not ok:
            failures.append('2a')
        compare_to_state('2b da_vinci_x_wrist')

        r = rig.set_offset(preset('flange'))
        if not r.success:
            print(f'FAIL 3: restore refused: {r.message}')
            return 1
        compare_to_state('3 restored (flange)')
    except TimeoutError as e:
        print(f'FAIL: timed out waiting for {e}')
        return 1
    finally:
        try:
            ex.shutdown()
            spin.join(timeout=2.0)
            rig.destroy_node()
            rclpy.shutdown()
        except ExternalShutdownException:
            pass

    print('ALL PASS' if not failures else f'FAILED: {failures}')
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
