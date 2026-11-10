# RCM under gamepad jog, left UR5e, 2026-11-01

**Layer 4 (measured numbers): one session, immutable once committed.** Interpretation lives in
decision 17 and the AGENTS.md headline.

## Setup

- Arm: left UR5e (`psm_left`, PolyScope 5.25.1), bare metal, `ur_robot_driver` 3.10.0,
  CartesianServoController at 500 Hz, `speedj` (command_interface velocity), RCM via the task
  QP (proxsuite 0.7.3) → Ruckig → RCM projection; `rcm.orientation_task: none`.
- Tool: the da Vinci X adapter, NO instrument fitted. Tip = the `da_vinci_x_wrist` preset
  (collaborator A's model: shaft along tool0 +y, 88.5 mm out; wrist 609 mm along it). The shaft
  is therefore virtual; the deviation below is KINEMATIC.
- RCM: `set_rcm.py on` (current shaft, default depth 100 mm behind the wrist), engaged at
  deviation 0.000 mm; point (0.4624, 0.6258, 0.1259) m in `psm_left_base_link`.
- Source: `gamepad_jog` (collaborator A's layout, RB deadman): left stick = pivot about the RCM
  point (≤ 0.1 rad/s), LT/RT = insertion (≤ 20 mm/s); operator at the cart.
- Code: `74a0485` plus the first-motion tip fix of the same day (tests/scripts only).
- Metric: `/psm_left/rcm_state.deviation`, the open laparoscopic platform's definition — the
  distance from the RCM point to the shaft line, by forward kinematics of the MEASURED joints,
  every controller cycle (500 Hz).
- Bag (machine-local): `$ARTS_LOCAL/bench_20261101/rcm_gamepad_psm_left` (rcm_state,
  joint_states, tool_tip_state/target, joint_command, status, gamepad).

## Results

296 s with the RCM on, 148 210 samples; 90 s of it jogging (a jog target within 0.2 s).
Virtual-wrist travel 3.7 m; insertion 97.7 – 152.8 mm; tip speed ≤ 22 mm/s (50 ms windows).

| Samples | n | median | p95 | max |
|---|---|---|---|---|
| all | 148 210 | 0.0135 mm | 0.0488 mm | 0.1891 mm |
| jogging | 45 075 | 0.0252 mm | 0.0731 mm | 0.1891 mm |
| holding | 103 135 | 0.0113 mm | 0.0248 mm | 0.0560 mm |

Least-squares slope of deviation over time: −3.9 µm/min. Per-minute medians: 0.0238, 0.0143,
0.0125, 0.0114, 0.0112 mm.

Same session, before the RCM: `first_motion_real_arm.py --step 0.03 --speed 0.005` with the
tip preset, x / y / z each 30 mm out and back: residuals ≤ 0.05 mm, return ≤ 0.04 mm, hold
drift ≤ 0.022 mm; one controller overrun at start-up, none after.

Operator remarks: motion smooth except at full stick; the stick axes follow the tool (shaft
frame), not the operator's view.
