#!/usr/bin/env python3
"""Hardware-free smoke test for the Cartesian servo controller.

Run it against a live `ur_arm.launch.py arm:=psm_left use_fake_hardware:=true`
(run_servo_smoke_test.sh does that). It is not a unit
test and is not run by colcon test; it is the check that the whole chain moves, which is
the thing unit tests of the individual stages will not tell you.

    ros2 run --prefix "python3" arts_controllers servo_smoke_test.py   # or just python3 it

Each check corresponds to a bug that has actually happened here:

  1. The arm does NOT move before anything is commanded. An empty reference buffer read as
     a valid pose, because a default-constructed quaternion is the identity, and sent the
     tool toward the base origin.
  2. A well-conditioned move is tracked, without wandering.
  3. Nothing exceeds the configured Cartesian speed limit. This is the check that has
     caught the most: the limit used to bound an internal setpoint the arm then chased,
     and every source of lag downstream of it turned into a burst of catch-up speed.
  4. A target well outside the workspace neither exceeds the speed limit nor disables the
     controller. An earlier revision latched a fault on an unreachable target and had to be
     manually reactivated.
  5. The arm can return afterwards, i.e. step 4 left nothing latched.
  6. It stops when the commands stop, rather than holding the last target forever.

The arm's start pose is not incidental to any of this. Mock hardware begins wherever the URDF
says, and the ARTS description deliberately overrides the vendor default because that default is
the elbow singularity; see arts_driver/config/ur5e_initial_positions.yaml. Run this against
anything else and step 2 measures the geometry rather than the controller.
"""
import math
import sys
import time

import rclpy
from arm_io import servo_param, BASE_FRAME, JOINT_STATES, TARGET_TOPIC, TOOL_FRAME, ToolTipTarget
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

MAX_LINEAR_VELOCITY = servo_param('cartesian', 'max_linear_velocity')  # m/s, read from config/ur5e_servo.yaml
STEP = 0.05                 # m, how far to ask the tool to move
OUT_OF_REACH = 1.5          # m past the start pose, comfortably beyond the UR5e's 0.85 m reach


class SmokeTest(Node):
    def __init__(self):
        super().__init__("servo_smoke_test")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            ToolTipTarget, TARGET_TOPIC, 10)
        self.joint_state = None
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def tool_position(self):
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def tool_orientation(self):
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
        r = tf.transform.rotation
        return (r.x, r.y, r.z, r.w)

    def send(self, position, orientation):
        msg = ToolTipTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = orientation
        self.pub.publish(msg)


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


PUBLISH_PERIOD = 0.02   # s, comfortably inside the controller's 0.1 s reference timeout
SPEED_WINDOW = 0.25     # s, window over which speed is evaluated


def drive_to(node, target, orientation, seconds):
    """Publish `target` at a steady rate until it is reached or `seconds` elapse.

    Republishing is required, not optional: the controller deliberately treats a reference
    older than reference_timeout as gone, so a single publish moves the arm for one
    timeout's worth and stops. Publishing on a wall-clock schedule rather than once per
    sampling iteration also keeps the reference continuously fresh, so the run measures
    steady tracking instead of a sawtooth of stalls and restarts.

    Speed is evaluated over a sliding window rather than between consecutive samples,
    because TF arrives at a different rate than this loop samples and per-sample
    differences are mostly jitter.

    Returns (elapsed, net displacement, path length, peak windowed speed). Net
    displacement and path length are reported separately because they disagree exactly
    when the arm wanders, which is what a badly conditioned direction looks like.
    """
    began = time.time()
    origin = node.tool_position()
    history = [(began, origin)]
    next_publish = began

    while time.time() - began < seconds:
        now_time = time.time()
        if now_time >= next_publish:
            node.send(target, orientation)
            next_publish = now_time + PUBLISH_PERIOD
        node.spin_for(0.005)
        history.append((time.time(), node.tool_position()))
        if distance(history[-1][1], target) < 1e-3:
            break

    peak_speed = 0.0
    for i, (t_end, p_end) in enumerate(history):
        for t_start, p_start in reversed(history[:i]):
            if t_end - t_start >= SPEED_WINDOW:
                peak_speed = max(peak_speed, distance(p_end, p_start) / (t_end - t_start))
                break

    path = sum(distance(a[1], b[1]) for a, b in zip(history, history[1:]))
    return (time.time() - began, distance(history[-1][1], origin), path, peak_speed)


def main():
    rclpy.init()
    node = SmokeTest()
    failures = []

    node.spin_for(3.0)
    if node.joint_state is None:
        print(f"FAIL: no {JOINT_STATES}; is the bringup running?")
        rclpy.shutdown()
        return 1

    start = node.tool_position()
    orientation = node.tool_orientation()
    print(f"start {TOOL_FRAME} in {BASE_FRAME}: "
          f"({start[0]:+.4f}, {start[1]:+.4f}, {start[2]:+.4f}) m")

    # 1. Silence must mean stillness.
    node.spin_for(3.0)
    drift = distance(node.tool_position(), start)
    print(f"[1] drift with no command over 3 s: {1000 * drift:.3f} mm")
    if drift > 1e-4:
        failures.append(f"arm moved {1000 * drift:.3f} mm before being commanded")

    # 2. Track a well-conditioned move. The arm starts bent and away from every singularity,
    #    so a 50 mm sideways step is about the easiest thing it can be asked to do and a
    #    failure here is the controller's, not the geometry's. It has to come before the
    #    out-of-reach probe, which leaves the arm somewhere unhelpful for everything after it.
    target = (start[0] + STEP, start[1], start[2])
    print(f"[2] commanding +{1000 * STEP:.0f} mm in x ...")
    elapsed, _, path, peak_speed = drive_to(node, target, orientation, seconds=15.0)
    error = distance(node.tool_position(), target)
    reached = error < 1e-3
    print(f"    reached={reached} in {elapsed:.2f} s along a {1000 * path:.1f} mm path, "
          f"residual {1000 * error:.3f} mm")
    if not reached:
        failures.append(f"did not reach a well-conditioned target, "
                        f"{1000 * error:.3f} mm short")
    if path > STEP * 1.5:
        failures.append(f"wandered {1000 * path:.0f} mm to make a {1000 * STEP:.0f} mm move")

    # 3. Speed limit. Two independent checks, because they fail differently: the windowed
    #    peak catches a transient burst, and the total time catches a sustained overspeed
    #    that a window might average away.
    print(f"[3] peak windowed tool speed: {1000 * peak_speed:.1f} mm/s "
          f"(limit {1000 * MAX_LINEAR_VELOCITY:.0f} mm/s)")
    if peak_speed > MAX_LINEAR_VELOCITY * 1.15:
        failures.append(f"exceeded the Cartesian speed limit at "
                        f"{1000 * peak_speed:.1f} mm/s")

    lower_bound = STEP / MAX_LINEAR_VELOCITY
    print(f"    {1000 * STEP:.0f} mm took {elapsed:.2f} s; the speed limit allows no less "
          f"than {lower_bound:.2f} s")
    if reached and elapsed < lower_bound * 0.95:
        failures.append(f"travelled {1000 * STEP:.0f} mm in {elapsed:.2f} s, which beats "
                        f"the {lower_bound:.2f} s the speed limit allows")

    # 4. Ask for a point far outside the workspace. The arm will run to the boundary and stall
    #    there, which is correct; what it must not do is exceed the speed limit chasing the
    #    target or disable itself when it cannot get there.
    before_probe = node.tool_position()
    unreachable = (before_probe[0] + OUT_OF_REACH, before_probe[1], before_probe[2])
    print(f"[4] commanding a target {1000 * OUT_OF_REACH:.0f} mm away, well out of reach ...")
    _, moved, path_far, peak_far = drive_to(node, unreachable, orientation, seconds=5.0)
    print(f"    net {1000 * moved:.1f} mm along a {1000 * path_far:.1f} mm path, "
          f"peak {1000 * peak_far:.1f} mm/s")
    if peak_far > MAX_LINEAR_VELOCITY * 1.15:
        failures.append(f"exceeded the Cartesian speed limit at {1000 * peak_far:.1f} mm/s "
                        f"chasing an unreachable target")

    # 5. Going back proves the probe latched nothing.
    print(f"[5] commanding a return to the pose from step 2 ...")
    _, _, _, _ = drive_to(node, target, orientation, seconds=15.0)
    error = distance(node.tool_position(), target)
    print(f"    residual {1000 * error:.3f} mm")
    if error > 1e-3:
        failures.append(f"could not return to a pose it had already reached, "
                        f"{1000 * error:.3f} mm short; the probe latched something")

    # 6. Stop publishing. A jerk-limited stop covers some distance, which is correct, so
    #    settle first and then check that it is genuinely stopped rather than still
    #    tracking a target nobody is sending any more. The bar is "stopped", not "frozen":
    #    at the speed limit it would cover 180 mm in this window.
    node.spin_for(1.0)
    held = node.tool_position()
    node.spin_for(3.0)
    creep = distance(node.tool_position(), held)
    print(f"[6] creep in the 3 s after settling: {1000 * creep:.3f} mm")
    if creep > 1e-4:
        failures.append(f"kept moving {1000 * creep:.3f} mm after commands stopped")

    print()
    for failure in failures:
        print(f"FAIL: {failure}")
    print("PASS" if not failures else f"{len(failures)} failure(s)")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
