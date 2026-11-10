# servoj / speedj A/B — command to measured joints, real left UR5e, 2026-10-30

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Decision: `docs/decisions/17_control_hierarchy.md` (amendment 2026-10-30). Module doc:
> `docs/modules/ur_arms.md`; tools `arts_controllers/test/shake_ab_real_arm.py` (motion) and
> `arts_controllers/bench/shake_bag_lag.py` (analysis).

**Result.** The same scripted shake (10 mm peak to peak along base x, 1 / 2 / 4 Hz, analytic
velocity as feedforward) reached the measured joints **34 / 35 / 40 ms** after the command
with `servoj` and **4–5 / 5 / 9 ms** with `speedj`, at the same amplitude (largest joint
1.00 vs 1.01–1.03°). With `speedj`, a hard-killed driver left the arm still and the pendant
program stopped by itself.

## Setup

| | |
|---|---|
| Host | production workstation, kernel 6.8.1-1062-realtime |
| Arm | left UR5e, 192.168.2.2, PolyScope 5.25.1, own factory calibration, adapter mounted (no instrument), payload 1 kg; External Control |
| Driver | `ur_robot_driver` 3.10.0; 500 Hz loop SCHED_FIFO 50 |
| Controller | `CartesianServoController`, `tracking.mode: rate` (K = 5, feasibility-weighted feedforward), limits 250 mm/s, 20 rad/s², 500 rad/s³ (`config/ur5e_servo.yaml`) |
| Modes | `command_interface:=position` → `servoj` (look-ahead 0.03 s, gain 2000); `command_interface:=velocity` → `speedj(qd, 40 rad/s², 2 ms)` |
| Motion | `shake_ab_real_arm.py` defaults: 5 s hold, then 4 s per frequency, 1.5 s rest between |
| Recording | `ros2 bag record` (MCAP): `/joint_states`, `target_pose`, `target_velocity` |
| Code | commit `9e4387b` (run); analysis tool added with this report |
| Data | machine-local `$ARTS_LOCAL/ab_speedj_20261030/`: `bag_servoj`, `bag_speedj`, launch and script logs, `analysis.txt` |

## Method

`target_pose` is stamped when the script sends it; `/joint_states` carries the UR's measured
joint positions, stamped on receipt by the driver. Over the last 2 s of each shake the
command's x is correlated with each of the three most-moving joints shifted by 0–499 ms; the
shift with the highest |r| is the lag. Both modes ran back to back, the arm untouched between.

## Results

| Shake | servoj lag (wrist_3 / shoulder_pan / wrist_1) | speedj lag | wrist_3 p-p servoj / speedj |
|---|---|---|---|
| 1 Hz | 34 / 34 / 34 ms | 4 / 5 / 4 ms | 1.00° / 1.01° |
| 2 Hz | 35 / 34 / 35 ms | 5 / 4 / 5 ms | 1.01° / 1.02° |
| 4 Hz | 40 / 39 / 41 ms | 9 / 8 / 10 ms | 0.99° / 1.03° |

All correlations r = 1.000. Hold before the shakes: tool moved 0.018 mm (servoj) and
0.035 mm (speedj) in 5 s; back at start within 0.01 mm in both.

**Stream loss, speedj.** With the arm holding still under External Control, the driver's
process group was killed with SIGKILL (07:16:03). The arm did not move (operator), and the
pendant program went to stopped on its own.

## Invalid numbers from the same session

The script's own printed lags (257–299 ms, both modes) are an artefact: it spun its ROS
callbacks once per 2 ms send cycle, so `/tf` queued ~270 ms behind. The script was fixed in
the commit that adds this report (callbacks in their own thread; lag from `/joint_states`
stamps) and on mock reads 1–10 ms, and 31–33 ms with the command clock shifted 30 ms.

## Not measured / limitations

- One arm, one axis, 10 mm; free space, no instrument.
- From the measured joint report, not from the physical arm (E6 measures that).
- The UR's internal servo and its report path are inside both numbers.
