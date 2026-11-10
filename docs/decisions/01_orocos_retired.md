# Decision 1 — OROCOS RTT retired; two rate-grouped ros2_control managers

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

1. **OROCOS RTT is retired.** ARTS is a ROS 2 / 3D Slicer / NVIDIA stack. The 100 Hz RTT
   mid-level controller (the v1.4 OROCOS controller) is replaced by ros2_control chainable
   controllers in TWO rate-grouped controller_manager processes: a 500 Hz manager for the two
   UR5e tool arms (+ instrument wrists) and a 125 Hz manager for the UR5 CB3 camera arm.
   The rates cannot mix in one manager: `ur_robot_driver`'s `read()` blocks on the RTDE
   stream, so a shared loop would be dragged to the slowest arm, and ros2_control has no
   native per-controller rates. Pipeline per manager: teleop follower → RCM + collision QP
   (pinocchio + proxsuite) → Ruckig trajectory smoothing → per-arm forward_position.
   Cross-manager joint states are mirrored via RT-safe buffers for the collision QP (≤ 8 ms
   staleness is fine for collision margins; the camera arm moves slowly). Reflexxes → Ruckig;
   KDL → pinocchio (+ ur_ikfast). Note: rtt_ros2_integration is open source — the objection
   is maintenance/community/build burden only.

## Amendment 2026-10-28 — the pipeline is a position-commanded hierarchy

The pipeline sentence above is superseded by [decision 17](17_control_hierarchy.md): reference
source (teleop follower | policy follower | goal mover) → task layer (RCM-constrained
resolved-rate QP, integrated to joint targets) → joint layer (Ruckig + limits) → per-arm
forward position. The two rate-grouped managers and the cross-manager buffers stand.

## Amendment 2026-10-29 — the stated reason for two managers is wrong for driver 3.x

"`ur_robot_driver`'s `read()` blocks on the RTDE stream" is not the default. In 3.8.0 and
3.10.0 the hardware interface runs `non_blocking_read:=true`; blocking is opt-in
(`blocking_read`, added in 3.10.0), and the driver documents non-blocking as REQUIRED when one
controller_manager holds several hardware components — which the 500 Hz manager does (two
UR5e + the instrument drives). So E5 and production run non-blocking, with the manager on the
host's steady clock and each UR controller on its own. The two-manager layout is not changed
by this note; whether the UR5 could share the 500 Hz manager (the driver's `rw_rate` lets a
component run slower than its manager) is an open question for Phase 2, not a decision.

## Amendment 2026-11-01 — one controller manager per arm (decision 18)

The two rate-grouped managers are replaced by **one manager per arm**: `/psm_left` and
`/psm_right` at 500 Hz, `/ecm` at 125 Hz, each in its own process on an isolated core. A PSM's
manager holds the UR5e and its instrument drive, so one `joint_states` carries both. Reasons:
the UR driver's launch is built for one robot per manager; one arm's fault cannot stop
another; decision 13 already requires one description per manager; and this decision's
grouping reason was found wrong on 2026-10-29. Cost: the collision QP reads the other arm
over a topic, a few milliseconds old, which its 100–300 ms rollout absorbs. The "cross-manager
joint states via RT-safe buffers" sentence above now means topics between three processes.
