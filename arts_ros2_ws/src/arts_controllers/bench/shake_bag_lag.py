#!/usr/bin/env python3
"""Lag from command to MEASURED joints for a shake_ab_real_arm.py run recorded in a bag.

    python3 shake_bag_lag.py BAG_DIR

The bag holds the command (/arts_cartesian_servo_controller/target_pose before decision 18,
/<arm>/tool_tip_target after; stamped at send time) and the joint states (/joint_states, or
/<arm>/joint_states: the UR's measured joint positions, stamped on receipt by the driver). Each
shake is found as a run of moving commands lasting > 3 s; over its last 2 s the command's x
(1 ms grid) is correlated with each of the three most-moving joints shifted by 0-499 ms, and
the shift with the highest |r| is that joint's lag. The joints' peak-to-peak (deg) shows the
amplitude reached, comparable across runs of the same script. RUN rule: source
arts_ros2_ws/install only (rosbag2_py).
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    cmd, js = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic.endswith(('target_pose', 'tool_tip_target')) or topic.endswith('/joint_states'):
            m = deserialize_message(raw, types[topic])
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if topic.endswith('/joint_states'):
                js.append((t, *m.position))
            else:
                cmd.append((t, m.pose.position.x))
    cmd, js = np.array(cmd), np.array(js)
    x = cmd[:, 1] - np.median(cmd[:, 1])
    moving = np.where(np.abs(np.gradient(x, cmd[:, 0])) > 1e-4)[0]
    for seg in np.split(moving, np.where(np.diff(cmd[moving, 0]) > 0.5)[0] + 1):
        a, b = cmd[seg[0], 0], cmd[seg[-1], 0]
        if b - a < 3.0:
            continue
        g = np.arange(b - 2.0, b, 0.001)
        c = np.interp(g, cmd[:, 0], x)
        up = g[np.where(np.diff(np.sign(c)) > 0)[0]]
        f = 1 / np.mean(np.diff(up)) if len(up) > 1 else 1 / (2 * np.mean(np.diff(g[np.where(np.diff(np.sign(c)) != 0)[0]])))
        win = js[(js[:, 0] > g[0]) & (js[:, 0] < g[-1] + 0.5)]
        out = []
        for k in np.argsort(-np.ptp(win[:, 1:], axis=0))[:3]:
            cc = [abs(np.corrcoef(c, np.interp(g + L * 1e-3, js[:, 0], js[:, 1 + k]))[0, 1]) for L in range(500)]
            L = int(np.argmax(cc))
            out.append(f'joint {k} {np.degrees(np.ptp(win[:, 1 + k])):.2f} deg lag {L} ms (r {cc[L]:.3f})')
        print(f'{f:3.1f} Hz: ' + '; '.join(out))


if __name__ == '__main__':
    main()
