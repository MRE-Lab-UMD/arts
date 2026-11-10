# E5 — servo loop health, real UR5e, 10 min, 2026-10-29

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E5 (`publication_plan.md` in the manuscript repo). Module doc:
> `docs/modules/ur_arms.md`; tools `arts_controllers/bench/run_e5.sh` and `analyze_e5.py`.

**Result.** The 500 Hz command-path loop of the right UR5e ran **499.994 Hz** over 628 s
(314 142 cycles), period **2000.00 ± 76.56 µs**, i.e. jitter **3.83 % of period** (target
< 10 %), every period between 1577.8 and 2425.6 µs. **0 missed cycles** in the loop after
start-up. The bag is 4 `/joint_states` messages short (4 of 314 146, 0.0013 %); they are lost
messages, not loop misses (below).

## Setup

| | |
|---|---|
| Host | production workstation, Intel Core Ultra 9 285K, kernel 6.8.1-1062-realtime, `isolcpus=2,4,6-7` |
| Arm | right UR5e, 192.168.2.1, PolyScope 5.25.1, its own factory calibration (`arts_calibration/config/tool_arm_right_kinematics.yaml`; no mismatch error); External Control started on the pendant; speed slider not in effect for `servoj` (`docs/modules/ur_arms.md`) |
| Driver | `ur_robot_driver` 3.10.0, `ur_client_library` 2.15.2, **non-blocking read** (default; required for a multi-component manager), ros2_control `controller_manager` 4.48.1 |
| Loop | 500 Hz, SCHED_FIFO priority 50, pinned to isolated core 4, memory locked; UR client RTDE thread SCHED_FIFO 99 (not pinned) |
| Controller | `arts_controllers/CartesianServoController` (DLS IK → Ruckig → speed governor), free space, no RCM |
| Motion | `first_motion_real_arm.py --step 0.03 --speed 0.005`, 10 cycles back to back: 10 s hold, then 30 mm out and back along x, y, z at 5 mm/s; all 10 completed, no abort |
| Recording | separate `ros2 bag record` process (MCAP): `/joint_states`, `/diagnostics`, `/arts_cartesian_servo_controller/target_pose` |
| Code | commit `446406d` (run), analyzer wording corrected in the commit that adds this report |
| Bag | 116 MB, machine-local `$ARTS_LOCAL/benchmark_bags/e5_servo_loop_20261029/` with launch log, motion log, `run_info.txt`, `analysis.txt`, `e5_result.json` |

## Method

The joint_state_broadcaster runs inside the controller_manager loop and stamps each message
with that cycle's time, so consecutive `/joint_states` stamps are the loop's periods.
Missed cycles are decided by two sources that do not pass through the recorder: the
controller_manager's "Overrun detected" warnings, and its own periodicity statistics
(cumulative since start-up) in `/diagnostics`.

## Results

| | Value |
|---|---|
| Cycles in the bag | 314 142 over 628.3 s → 499.994 Hz |
| Period, all stamps | 2000.03 ± 76.89 µs; min 1577.8, max 4031.7, p99.9 2285.9 |
| Period, excluding the 4 gaps | **2000.00 ± 76.56 µs**; min 1577.8, max 2425.6; 6216 periods deviate > 200 µs, 0 deviate > 500 µs |
| Overrun warnings | 1, at start-up, before recording began ("loop took 2.40 ms, missed cycles: 2", read 2.26 ms on the first RTDE packet); 0 during the run |
| controller_manager periodicity (cumulative) | min 249.974 Hz throughout the recording (the start-up event); max 604.3 Hz until 99 s into the recording, then 629.1 Hz (a 1.59 ms period, the bag's shortest); mean 500.66 Hz |
| Doubled periods in the bag | 4: 3975, 4032, 3985, 3999 µs at 29.4, 61.7, 142.1, 442.0 s; each neighbour normal (1907–2057 µs) |

**The 4 doubled periods are lost messages.** The 4032 µs period would have lowered the
controller_manager's cumulative minimum from 249.974 Hz to 248.0 Hz, and it did not, so at
least that one never occurred in the loop. For the other three (251–252 Hz, above the
start-up minimum) the statistics cannot decide; none raised an overrun warning, and each is
exactly two periods with normal periods either side, the signature of one message missing
rather than of a late cycle.

## Not measured / limitations

- One arm in the manager. Production holds two UR5e and the instrument drives.
- The UR controller's own servo loop (internal to the arm); ARTS measures the command path.
- Loop execution time per cycle (only its period); the controller's update time was 5.8 µs
  at the start-up overrun.
- The RTDE thread was not pinned to an isolated core.
- Free space, slow motion: no RCM, no teleop input.
- The 4 lost `/joint_states` messages bear on the data metric "zero dropped robot samples":
  the state bag must not rely on a separate default-QoS recorder process.
