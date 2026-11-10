# Decision 11 — No MoveIt

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

11. **No MoveIt anywhere in `arts_ros2_ws` or the middle overlays.** MoveIt overlaps only
    with the parts decision 1 deliberately replaces (KDL kinematics, Reflexxes-style
    smoothing) and supplies none of what ARTS needs: RCM as a QP constraint, instrument
    wrist joints, a chainable 500 Hz pipeline across two rate-grouped managers. MoveIt Servo
    is the nearest thing and has no RCM, so adopting it means forking it into what Phase 2
    builds anyway. The decisive objection is the prior system's: MoveIt's `RobotModel` is
    built from URDF+SRDF at load and treated as immutable, so a LIVE tool-tip frame offset —
    which the v1.4 OROCOS controller added `add_tool_tip_offset` for, and which is used
    constantly during calibration and experiments — is not cleanly supported. Servo's
    end-effector frame can follow TF, but the offset is then outside the kinematic model, so
    the Jacobian is computed to a model link rather than the real tip. ARTS needs the tip
    offset IN the model because the RCM constraint needs the true shaft line. With pinocchio
    ARTS owns the chain and the offset is a parameter updated into an RT-safe buffer.
    Concretely: `ur_moveit_config` is excluded in `middle_overlay_ros2_wss/ur_ros2_ws/build.sh`
    via COLCON_IGNORE + AMENT_IGNORE + `--skip-keys` (all three are required, and the
    reason each one is needed is documented in that file's header). Building it pulled 45
    apt packages including `ros-jazzy-ruckig` 0.9.2, which shadowed the pinned Ruckig — see
    the Ruckig note under "Dependency pinning policy". If MoveIt is ever genuinely wanted it
    belongs in an APPLICATION OVERLAY, where its dependencies cannot reach the control path;
    re-enabling it is a one-line change plus a rebuild, which is why the option was not
    bought upfront.
