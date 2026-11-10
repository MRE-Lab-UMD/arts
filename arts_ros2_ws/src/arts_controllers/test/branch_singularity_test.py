#!/usr/bin/env python3
"""Hardware-free branch / gimbal-lock regression test for the Cartesian servo controller.

Run it like servo_smoke_test.py, against a live `ur_arm.launch.py arm:=psm_left
use_fake_hardware:=true` (or via `run_servo_smoke_test.sh branch_singularity_test.py`).
Decision 17 requires it to pass before any real-arm run.

It targets the two failures a seeded resolved-rate / DLS IK can have that the smoke test does
not look for, both seen in collaborator code on real arms:

  1. Gimbal lock at the wrist. With wrist_2 at 0 the wrist_1 and wrist_3 axes are collinear,
     and an IK that only minimises the step can spin both in opposite directions. The test
     drives the tool through that singularity along a path a pure wrist_2 rotation makes
     exactly (rotation about the wrist_2 axis), so any wrist_1 / wrist_3 travel is the
     solver's, not the path's. It must cross with every joint inside its rate limit, end on
     the far side, and come back to the start JOINTS, not just the start pose: coming back
     with wrist_1 and wrist_3 each off by pi is the branch flip that leaves the tool in place
     and the instrument cable wound.
  2. Branch alternation. A sweep of large moves around the workspace and back must never
     change the elbow's side, and the same pose must give the same joints every time.

The test checks that it actually reached the singularity (|wrist_2| < 3 deg at some sample):
a run that steered around it would otherwise pass without testing anything.
"""
import math
import sys
import time

import numpy as np
import rclpy
from arm_io import ARM, servo_param, BASE_FRAME, JOINT_STATES, TARGET_TOPIC, TOOL_FRAME, ToolTipTarget
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

WRIST_2_FRAME = f"{ARM}_wrist_2_link"   # its z axis is the wrist_2 joint axis
JOINTS = [f"{ARM}_{j}" for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
MAX_JOINT_VELOCITY = 1.0     # rad/s, must match config/ur5e_servo.yaml
MAX_LINEAR_VELOCITY = servo_param('cartesian', 'max_linear_velocity')  # m/s, read from config/ur5e_servo.yaml
PUBLISH_PERIOD = 0.02        # s, inside the 0.1 s reference timeout
RATE_WINDOW = 0.05           # s, joint rate is evaluated over this window
CROSS_ANGLE = math.radians(110.0)   # wrist_2 from -90 deg to +20 deg, through 0
SINGULAR_REACHED = math.radians(3.0)
SAME_JOINTS = 2e-3           # rad, "the same pose gives the same joints"
SPECTATOR_TRAVEL = math.radians(10.0)


class Probe(Node):
    def __init__(self):
        super().__init__("branch_singularity_test")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            ToolTipTarget, TARGET_TOPIC, 10)
        self.history = []   # (t, q[6])
        self.create_subscription(JointState, JOINT_STATES, self._on_js, 50)

    def _on_js(self, msg):
        if not all(j in msg.name for j in JOINTS):
            return
        idx = [msg.name.index(j) for j in JOINTS]
        self.history.append((time.time(), np.array([msg.position[i] for i in idx])))

    def q(self):
        return self.history[-1][1].copy()

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def frame(self, child):
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, child, rclpy.time.Time())
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([r.x, r.y, r.z, r.w])

    def send(self, p, rot):
        msg = ToolTipTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = rot.as_quat()
        self.pub.publish(msg)

    def follow(self, path, seconds):
        """Publish path(s) for s in [0, 1] over `seconds`, then hold the end for 3 s.

        The path is paced slower than the speed limits allow, so the controller tracks it
        rather than chasing a far target, and the test measures the solver on that path.
        """
        began = time.time()
        while True:
            s = min(1.0, (time.time() - began) / seconds)
            self.send(*path(s))
            self.spin_for(PUBLISH_PERIOD)
            if s >= 1.0 and time.time() - began > seconds + 3.0:
                return


def peak_joint_rate(history, since):
    samples = [(t, q) for t, q in history if t >= since]
    peak = np.zeros(6)
    j = 0
    for i, (t, q) in enumerate(samples):
        while j < i and t - samples[j][0] > RATE_WINDOW:
            j += 1
        if j < i and t - samples[j][0] >= RATE_WINDOW * 0.5:
            peak = np.maximum(peak, np.abs(q - samples[j][1]) / (t - samples[j][0]))
    return peak


def travel(history, since):
    qs = np.array([q for t, q in history if t >= since])
    return np.abs(np.diff(qs, axis=0)).sum(axis=0)


def main():
    rclpy.init()
    node = Probe()
    failures = []
    node.spin_for(3.0)
    if not node.history:
        print(f"FAIL: no {JOINT_STATES}; is the bringup running?")
        return 1

    q_start = node.q()
    p_start, r_start = node.frame(TOOL_FRAME)
    print("start joints (deg):", np.round(np.degrees(q_start), 1))

    # ── 1. Through the wrist singularity and back ──────────────────────────────────────
    w_origin, w_rot = node.frame(WRIST_2_FRAME)
    axis = w_rot.apply([0.0, 0.0, 1.0])
    # Direction: whichever sign of rotation about the axis drives wrist_2 toward 0. A pure
    # wrist_2 rotation by +theta about this axis changes wrist_2 by +theta.
    sign = 1.0 if q_start[4] < 0 else -1.0

    def about_wrist_2(theta):
        rot = Rotation.from_rotvec(sign * theta * axis)
        return w_origin + rot.apply(p_start - w_origin), rot * r_start

    radius = np.linalg.norm(np.cross(axis, p_start - w_origin))
    seconds = max(CROSS_ANGLE / 0.2, radius * CROSS_ANGLE / (0.5 * MAX_LINEAR_VELOCITY))
    print(f"[1] wrist_2 {math.degrees(q_start[4]):+.0f} -> "
          f"{math.degrees(q_start[4] + sign * CROSS_ANGLE):+.0f} deg over {seconds:.0f} s "
          f"(tool on a {1000 * radius:.0f} mm radius)")
    t0 = time.time()
    node.follow(lambda s: about_wrist_2(s * CROSS_ANGLE), seconds)
    q_far = node.q()
    closest = min(abs(q[4]) for t, q in node.history if t >= t0)
    rate = peak_joint_rate(node.history, t0)
    moved = travel(node.history, t0)
    print(f"    closest |wrist_2| {math.degrees(closest):.2f} deg; far side "
          f"wrist_2 {math.degrees(q_far[4]):+.1f} deg")
    print("    joint travel (deg):", np.round(np.degrees(moved), 1))
    print("    peak joint rate (rad/s):", np.round(rate, 3))
    if closest > SINGULAR_REACHED:
        failures.append(f"never reached the wrist singularity (closest "
                        f"{math.degrees(closest):.1f} deg): the test exercised nothing")
    if abs(q_far[4] - (q_start[4] + sign * CROSS_ANGLE)) > math.radians(2.0):
        failures.append(f"did not cross the singularity: wrist_2 ended at "
                        f"{math.degrees(q_far[4]):+.1f} deg")
    for j in (3, 5):
        if moved[j] > SPECTATOR_TRAVEL:
            failures.append(f"{JOINTS[j]} travelled {math.degrees(moved[j]):.1f} deg on a "
                            f"path that needs only wrist_2 (gimbal-lock spin)")
    if rate.max() > MAX_JOINT_VELOCITY * 1.15:
        failures.append(f"joint rate {rate.max():.2f} rad/s exceeds the "
                        f"{MAX_JOINT_VELOCITY} rad/s limit crossing the singularity")

    print("    ... and back to the start pose")
    t1 = time.time()
    node.follow(lambda s: about_wrist_2((1.0 - s) * CROSS_ANGLE), seconds)
    q_back = node.q()
    dq = np.abs(q_back - q_start)
    print("    start-joint error after the round trip (deg):", np.round(np.degrees(dq), 2))
    if dq.max() > SAME_JOINTS:
        failures.append(f"round trip through the singularity changed the joints by up to "
                        f"{math.degrees(dq.max()):.1f} deg at the same pose (branch flip)")
    rate = peak_joint_rate(node.history, t1)
    if rate.max() > MAX_JOINT_VELOCITY * 1.15:
        failures.append(f"joint rate {rate.max():.2f} rad/s on the way back")

    # ── 2. Branch sweep: large moves out and back, the elbow never changes side ────────
    moves = [(+0.15, 0, 0, 0, 0, 0), (0, +0.15, 0, 0, 0, 0), (0, 0, -0.15, 0, 0, 0),
             (-0.10, -0.10, +0.10, 0.6, 0, 0), (0, 0, 0, 0, 0.6, 0), (0, 0, 0, 0, 0, 1.2),
             (+0.10, -0.15, -0.10, -0.5, 0.5, -1.0)]
    elbow_side = np.sign(q_start[2])
    t2 = time.time()
    print("[2] branch sweep: 7 large moves, each out and back")
    for k, (dx, dy, dz, rx, ry, rz) in enumerate(moves):
        p_goal = p_start + np.array([dx, dy, dz])
        r_goal = Rotation.from_rotvec([rx, ry, rz]) * r_start
        out = lambda s: (p_start + s * (p_goal - p_start),
                         Rotation.from_rotvec(s * Rotation.from_rotvec([rx, ry, rz]).as_rotvec())
                         * r_start)
        back = lambda s, o=out: o(1.0 - s)
        dist = max(np.linalg.norm(p_goal - p_start) / (0.5 * MAX_LINEAR_VELOCITY),
                   np.linalg.norm([rx, ry, rz]) / 0.2, 1.0)
        node.follow(out, dist)
        node.follow(back, dist)
        dq = np.abs(node.q() - q_start)
        print(f"    move {k + 1}: back at start, max joint error {math.degrees(dq.max()):.3f} deg")
        if dq.max() > SAME_JOINTS:
            failures.append(f"move {k + 1}: the start pose came back with different joints "
                            f"(up to {math.degrees(dq.max()):.1f} deg)")
    sides = {np.sign(q[2]) for t, q in node.history if t >= t2}
    if sides != {elbow_side}:
        failures.append("the elbow changed side during the sweep (branch alternation)")
    rate = peak_joint_rate(node.history, t2)
    print("    peak joint rate (rad/s):", np.round(rate, 3))
    if rate.max() > MAX_JOINT_VELOCITY * 1.15:
        failures.append(f"joint rate {rate.max():.2f} rad/s during the sweep")

    print()
    for f in failures:
        print(f"FAIL: {f}")
    print("PASS" if not failures else f"{len(failures)} failure(s)")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
