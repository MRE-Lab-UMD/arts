# Bench session 2026-11-07 — fusionTrack benchmarks (repeatability, RCM drift) + ECM RCM

**Layer 3 (bench doc): do this at the bench, in order.** Status lives in `AGENTS.md`;
results go to `docs/benchmarks/` and a one-line headline beside the checkbox.

Two shells, never mixed:
- **ARTS shell**: `source /etc/profile.d/arts-dds.sh; source ~/arts/arts_ros2_ws/install/setup.bash`
- **Tracker shell**: `source /etc/profile.d/arts-dds.sh; source ~/atracsys_ws/install/setup.bash`
  (the DDS line only sets the RMW, so the tracker's topics reach the ARTS shell; nothing in
  `~/atracsys_ws` is sourced by ARTS, see `~/atracsys_ws/NOTES.md`).

`B=~/arts/arts_ros2_ws/src/arts_controllers/bench`. Real arm: the operator holds the pendant and
the e-stop for every motion, gives the go per motion, and stops the pendant program BEFORE the
driver. Stop each module once its item has passed.

## Gamepad (Xbox, `gamepad_jog.py`; to move into the README checklist)

- **Back / Start / Guide** select psm_left / psm_right / ecm. Press once before jogging.
- **Hold RB** = arm jog (deadman; release stops). RCM off: left stick = tip across the shaft,
  LT = in along the shaft, RT = out, right stick X = roll about the shaft. RCM on: left stick =
  pivot about the RCM, LT insert, RT retract. RCM off only (added 2026-11-07): right stick Y =
  pitch about the tip's x, d-pad X = yaw about the tip's y, 0.3 rad/s, about the tip point (the
  controller shares them between the instrument wrist and the arm).
- **Hold LB** = instrument (disc) jog; with LB held, A = disc mode, B = joint mode, X = SET ZERO.
  Joint mode: left stick X roll, right stick X pitch, d-pad X yaw, **RT = jaw (opens)**.
  Refused while the arm's servo controller streams `tool_joint_target` (any arm bringup with
  `wrist:=true`), so with the arm up LB does nothing but log a warning. With the ProGrasp marker
  clipped on: never touch LB.

## S0 — instrument start (EVERY session, before any arm controller runs)

The instrument is re-engaged, realigned and zeroed every session; never re-zeroed alone (PI,
2026-11-06: both needle drivers sat tilted after a plain re-zero).
1. Pendants: power on, brakes released; do NOT press Play yet.
2. **Instrument OFF the adapter.** `ros2 launch arts_dv_tool_driver tool_homing.launch.py
   arm:=psm_left tool:=forceps` (the ProGrasp uses the bipolar's `forceps` profile). Wait for
   "Start positions reached" (a warning that a motor did not reach its start = the instrument
   was on: stop, take it off, restart).
3. Instrument ON, press **A**. Wait for "All homing complete" (~30 s). Ctrl+C.
4. Visual check + touch-up: `ros2 launch arts_driver arts_bringup.launch.py psm_left_tool:=prograsp_forceps
   psm_left_tool_actuator:=true gamepad:=true allow_mode_write:=true use_fake_hardware:=false`.
   Back = psm_left; hold LB, B = joint mode: roll left stick X, pitch right stick X, yaw d-pad X;
   wrist straight, jaws just closed; LB + X = set zero. Ctrl+C. Do not touch the instrument
   until the arm bringup (its disc_drive zeroes at the discs' positions at start).
   (2026-11-06 ProGrasp: homing left yaw ~30 deg off; pitch hit its second limit by push.)

## S1 — tracker network and node (10 min)

1. USB-Ethernet adapter in P10, tracker powered. `ip -br link` → the new interface `<if>`.
2. **sudo, run by the PI** (docker0 holds 172.17.0.0/16; a /24 on `<if>` is more specific and
   wins; not persistent across reboots):
   `sudo ip addr add 172.17.1.100/24 dev <if> && sudo ip link set <if> up`;
   check `ping -c 3 172.17.1.7`.
3. Tracker shell: `cd ~/atracsys_ws/config && L=~/atracsys_ws/install/cisst/lib &&
   LD_PRELOAD="$L/libdevice64.so $L/libfusionTrack64.so" ros2 run atracsys atracsys -j
   config-prograsp-20261107.json` (S2; geometry re-measured 2026-11-07 after the spheres were remounted, `$B/tracker_geometry_maker.py`) or `-j config-vacuum.json` (S3). Without the preload the node
   segfaults at start (the SDK's regex binds to a newer copy; `~/atracsys_ws/NOTES.md`).
   "Stray markers" errors = reflective surfaces in view, not a fault.
4. ARTS shell: `ros2 topic hz /atracsys/Prograsp/measured_cp` (the topic name follows the
   tool name in the config). If nothing arrives here but `ros2 topic list` in the tracker
   shell shows it, the RMW differs: check `echo $RMW_IMPLEMENTATION` in both shells.

## S2 — pose repeatability, ProGrasp + its marker (~40 min)

Collaborator A's procedure; the dVRK Classic gets the same from A on Monday.
1. S0 done (jaws closed at the zero). Clip the marker on only AFTER step 2, with the disc torque
   on: a disc back-driven while the torque is off shifts the zero, since disc_drive zeroes at
   the discs' positions when it starts. The jaws stay closed for the whole test (the runner
   never commands the jaw).
2. ARTS shell: `ros2 launch arts_driver arts_bringup.launch.py psm_left:=true
   psm_left_tool:=prograsp_forceps psm_left_tool_actuator:=true wrist:=true gamepad:=true
   use_fake_hardware:=false`; Play on the psm_left pendant. RCM stays OFF (no cannula).
3. Teach: `python3 $B/repeatability_real_arm.py teach ~/arts_local/repeat_20261107/poses.yaml
   --tracker-topic /atracsys/Prograsp/measured_cp`. Jog with the gamepad (Back, hold RB);
   5 poses with large position AND orientation variation, the marker visible at each
   (SPACE refuses when it is not), no contact between the marker and the shaft; q writes.
4. Dry run (watch it, hand on the e-stop): `ARTS_ARM=psm_left $B/run_repeatability.sh
   ~/arts_local/repeat_20261107/dry run ~/arts_local/repeat_20261107/poses.yaml
   --tracker-topic /atracsys/Prograsp/measured_cp --cycles 1`.
5. Full run: same with OUT `.../run1` and no `--cycles` (30 cycles, 150 stops; ~6-8 s per stop:
   move + 3 s hold + 1 s static window, ~20 min). The tracker table in `analysis.txt` is the
   number (ISO RP and A's mean/std/RMS/max per pose and pooled); misses are listed.
6. Pass: 0-2 misses, every pose present. Result to `docs/benchmarks/repeatability_psm_left_20261107.md`.

## S3 — RCM drift, shaft marker (~45 min)

1. Remove the ProGrasp marker. Tracker node with `config-vacuum.json` (marker-centred origin;
   the `-pivot` geometry's origin was calibrated on A's vacuum tool, 93.9 mm from the marker,
   and need not lie on this shaft's axis).
2. ARTS: `python3 $B/set_rcm.py on --arm psm_left` (100 mm behind the wrist, in free air).
3. Alignment: ARTS `python3 $B/rcm_motion_real_arm.py wobble` (15 deg cone, 6 s/rev, Ctrl+C
   ends it, back to the start in 3 s). Tracker shell: `python3 $B/pivot_monitor.py --topic
   /atracsys/Vacuum/measured_cp`. Slide the marker along the shaft until `along` ~ 0 (ALIGNED,
   0.5 mm); `across` is the marker's distance from the shaft axis and does not change by
   sliding. Tighten the set screw; confirm `along` again; Ctrl+C both.
4. Dry run (1 min, watch the marker stays visible over pivot +-20 deg and roll +-30 deg):
   `$B/run_rcm_drift.sh ~/arts_local/rcm_drift_20261107/dry --minutes 1 --seed 1`.
   Occluded: lower `--max-roll`, and use the same value for every trial.
5. Trials: `.../trial1 --minutes 5 --seed 1`, `trial2 --seed 2`, `trial3 --seed 3`. Each
   `analysis.txt`: pivot error mean +- std (DASH's form), RMS, p95, max; drift slope; the
   kinematic deviation beside it.
6. Result to `docs/benchmarks/rcm_drift_psm_left_20261107.md`. A 30 min trial only if a slope
   is clearly above zero.

## S4 — ECM RCM, first real check (20 min)

The ECM now servos the scope tip (`tool_tips.yaml stereo_laparoscope`, roll-only orientation
under the RCM); mock: `ecm_rcm_mock_test.py`, `camera_teleop_mock_test.py`.
1. Bringup as for clinical teleop (ecm, cameras, display positions, head sensor), Play on all.
2. `python3 $B/set_rcm.py on --arm ecm` (scope tip 100 mm beyond the point).
3. `run_teleop_real.sh` with `ARTS_CAMERA=1 ARTS_CAMERA_ARGS="-p camera_max_offset:=0.02"`
   (the box limits the scope TIP; under the RCM the flange swings ~4.7x as far).
4. Middle pedal, both hands: sideways = the view pans (scope pivots about the port), toward /
   away = zoom (insertion), steering = roll about the optical axis. `set_rcm.py status --arm
   ecm`: deviation stays small.

## S5 — checks from 2026-11-06 (15 min)

1. **VerseGrip orientation at rest** (the 2 Hz shaking): clamp the left VerseGrip still, record
   `ros2 bag record /mtm_left/stylus_state /mtm_left/cursor` 3 min; a 1.5-2 Hz swing in the
   orientation with the grip clamped = a sensor/fusion fault, not the hand.
2. **Disc jog with the arm running**: arms up with `wrist:=true`, hold LB: disc_drive now logs
   "disc jog IGNORED: tool_joint_target is streaming" (instead of failing silently).
