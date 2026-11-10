# Phase 1 — Control core — status narrative

> **Role: status narrative archive.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Checkboxes here are a SNAPSHOT and are NOT authoritative — status lives only in
> `AGENTS.md`. Narrative that duplicates a module doc is still to be de-duplicated
> into that module doc (see `docs/repo_structure_triage.md`). Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

- [x] ros2_control chainable servo controller (Ruckig + pinocchio) on one UR5e.
 WRITTEN, RUNNING AND PASSING ON MOCK HARDWARE 2026-08-12. Real-arm numbers are still
 unmeasured; everything below is from `use_fake_hardware:=true`.
 `arts_controllers/CartesianServoController`, a chainable controller whose cycle is
 reference pose → damped-least-squares IK (pinocchio) → Ruckig joint smoothing →
 Cartesian speed governor → `{joint}/position`. Chainable so Phase 2 can insert the
 RCM/collision QP between a teleop follower and this controller without either changing.
 Five things worth knowing before touching it:
 - RUCKIG IS IN JOINT SPACE, LAST, per decision 1 — not Cartesian as the v1.4 reference
 had it. The limits that protect the hardware are joint limits, so the stage enforcing
 them is the one right before the write, with nothing downstream able to violate them.
 - THE TOOL-SPACE SPEED LIMIT (the surgically meaningful one, mm/s) IS A BOUND ON
 RUCKIG'S VELOCITY LIMITS, NOT A CLAMP ON ITS TARGET. This is the single hardest thing
 that was learned building it and it cost most of a day, so it is worth the paragraph.
 The obvious design — clamp the pose handed to IK to one cycle's worth of Cartesian
 travel — does not work, because Ruckig's Position interface plans to ARRIVE at its
 target: a target permanently one cycle ahead asks it to accelerate and stop inside a
 tenth of a millimetre, and the arm crawled at 3 mm/s against a 60 mm/s limit. Handing
 it a `target_velocity` to compensate, which is what MoveIt's `ruckig_filter.cpp` does,
 is WORSE: Ruckig reads the pair as "be at that point travelling at that speed", and
 arriving 0.0005 rad away at 0.5 rad/s from rest is infeasible under the acceleration
 limit, so its minimum-time answer is to REVERSE first to gain runway. The next cycle's
 target is anchored to the command that just moved backwards, so it reverses again.
 Commanded +50 mm in x, the arm travelled 800 mm in −x at nearly the speed limit — with
 an IK residual of exactly zero throughout, which is what made it look like a
 kinematics bug for hours. What works: IK targets the FULL reference, Ruckig gets a
 stationary goal (which is what it is for), and the Cartesian limit is converted each
 cycle into a joint-rate bound via the Jacobian — allowed tip speed divided by the tip
 speed a unit joint rate buys along the current direction of travel — then split across
 the joints in proportion to how far each must move. Near a singularity that bound goes
 to infinity, correctly, since the joint limits are binding there; a Cartesian speed
 governor after `update()` scales Ruckig's output back when the start-of-cycle Jacobian
 was too optimistic, which is the only stage that can actually guarantee the limit.
 Result on mock hardware: a 50 mm move in 1.06 s along a 49.9 mm path, peak 57.7 mm/s
 against a 60 mm/s limit.
 - THE TOOL-TIP OFFSET lives inside the pinocchio model as the tip frame's placement, so
 the Jacobian goes to the real tip — which is what Phase 2's RCM needs for the true
 shaft line. Changing it is twelve doubles into an existing frame placement, read from
 an RT-safe buffer; the v1.4 version rebuilt four solvers per call, leaked them, and
 forgot the velocity solver. Exposed as the `~/set_tool_tip_offset` service
 (`arts_interfaces/SetToolTipOffset`) rather than v1.4's deployer-only operation, so it
 can be scripted and logged. UNITS ARE METRES; v1.4 took millimetres and negated them
 internally, so do not port numbers across without converting.
 - Three v1.4 behaviours were deliberately NOT reproduced: the undamped pseudoinverse
 (now damped least squares, because undamped is fine until the arm nears a singularity),
 the unbounded 250-iteration IK (now capped at 8, since an occasional 200-iteration
 cycle is a deadline miss waiting to happen), and the absent command timeout (a dead
 upstream publisher used to leave the last target latched forever).
 - UR PUBLISHES NO ACCELERATION OR JERK LIMIT for any UR5e joint — `ur_description`'s
 own `config/ur5e/joint_limits.yaml` says so in a comment on all six joints and sets
 `has_acceleration_limits: false`. Ruckig needs both, so those numbers are ARTS's own
 design parameters, not a lookup. Defaults and the full reasoning are in
 `arts_controllers/config/ur5e_servo.yaml`; they are provisional until validated on
 hardware, and changing them is a benchmarked act like any other pin.
- [x] Bringup: controller_manager at 500 Hz + controller yaml + launch in `arts_driver`;
 verify with `use_fake_hardware` BEFORE the real arm. DONE 2026-08-12.
 `arts_driver/launch/ur5e_tool_arm.launch.py` wraps `ur_control.launch.py` rather than
 replacing it (the dashboard client, robot state helper, controller stopper and URScript
 interface are all needed on real hardware and are not ARTS's to reimplement), supplying
 only the two things that are ARTS's: `arts_controllers/config/ur5e_update_rate.yaml`
 and the servo controller's own parameters. It passes `activate_joint_controller:=false`
 because the default joint trajectory controller claims the same position command
 interfaces; it stays loaded but inactive and can be switched to for homing.
 Verified by `arts_controllers/test/run_servo_smoke_test.sh`, which brings up a mock
 arm on its own `ROS_DOMAIN_ID` and checks six things, each one a bug that happened:
 no motion before a command, tracking without wandering, the Cartesian speed limit
 (windowed peak and total elapsed time, which fail differently), an out-of-reach target
 neither overspeeding nor latching a fault, recovery afterwards, and stopping when the
 commands stop. Run it with the wrapper, not bare — isolation and process-group teardown
 are both load-bearing; three orphaned bringups on one domain once produced a full run of
 confidently wrong numbers.
 THE MOCK START POSE IS PART OF THE TEST, NOT SCENERY. `ur_description`'s default is
 `elbow_joint = 0`, the elbow singularity, where the arm cannot move along its own axis
 at all and IK-based tracking fails in a way that looks exactly like a controller bug.
 `arts_driver/config/ur5e_initial_positions.yaml` overrides it to a pose a quarter turn
 from all three UR singularities. Injecting it needs `arts_driver/urdf/arts_ur.urdf.xacro`
 passed as `description_file`, because neither UR launch file forwards
 `initial_positions_file`; the wrapper works by declaring the arg before including the
 vendor xacro, since `<xacro:arg>` only sets a default when one is not already defined.
- [ ] Phase 1 exit is the mock servo **regression**, not a paper accuracy number.
      DECIDED 2026-08-28: do not run a parity test against the v1.4 OROCOS
      controller. That baseline is closed, unreproducible, and identifying, and
      the new controller deliberately differs (decision 1; undamped pinv,
      unbounded IK, and no command timeout were not reproduced).
      `reference/v1.4_ref/` stays a private porting aid. Do not write "matched
      the old traces" into any tracked file.
      WHAT `run_servo_smoke_test.sh` IS: one mock UR5e, vendor description only
      (`arts_ur.urdf.xacro`), TF `base_link` → `tool0` (the flange, not an
      instrument tip). It checks six bugs that already happened (no motion
      before a command, tracking, 60 mm/s cap, unreachable target, recovery,
      stop-on-silence). Keep it green. A `docs/benchmarks/` note from it, if
      written, must say **mock flange / controller regression**. It is not a
      SOTA row and must not be compared to MOPS.
      WHAT THE PAPER MAY QUOTE FROM A BARE ARM, and only once a real UR5e is
      attached: command-path loop rate + jitter + deadline misses (the Control
      row in the Evaluation metrics table), against dVRK-Si's published loop
      statistics, with the caveat that UR joint servos are internal. Still
      `tool0`. Still no instrument.
      WHAT IS NOT PHASE 1, and must not be promised here: MOPS vision-in-loop
      (1.2 mm / 0.3° is a stereo-tracked **marker on the instrument**, taught
      trajectory, real arm — Evaluation / paper row, after Phase 4 tool drive);
      kinematic or physical RCM (Phase 2, QP + cannula); SPARC/LDLJ and
      master–slave tracking (reference run). Flange-to-tip is not a fixed
      mesh (decision 12): those numbers need the wrist joints in ros2_control
      and the tip frame in pinocchio. First instrumented bring-up uses dVRK's
      published DH + coupling from `$ARTS_LOCAL` (not shipped); the measured
      instrument-id protocol stays deferred.
