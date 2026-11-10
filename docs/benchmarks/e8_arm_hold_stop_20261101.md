# E8 — ARM HOLD stopping time, distance, hold drift and resume, left UR5e, 2026-11-01

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E8 (`publication_plan.md` in the manuscript repo). Module doc:
> `docs/modules/safeguard_stop.md`; tools `arts_controllers/bench/run_e8.sh`,
> `e8_hold_real_arm.py`, `e8_analyze.py`.

**Result.** ARM HOLD pressed 10 times while the left UR5e moved under the servo controller.
From the UR's safeguard-stop report, the tool came within 1 mm of its final pose in
**96.7 ± 1.0 ms** at 100 mm/s and **101.4 ± 3.8 ms** at 250 mm/s (max 105.6 ms). A settle
under 0.3 mm followed and ended (within 0.1 mm) by 288–348 ms. Stopping distance was
**7.76 ± 0.06 mm** at 100 mm/s and **18.41 ± 0.65 mm** at 250 mm/s (max 19.04 mm). Holds of
14–51 s drifted at most **0.014 mm**, and the reset (blue out, green) moved the tool at most
**0.022 mm**. Both are at the level of the pose noise, about 10 µm.

## Setup

| | |
|---|---|
| Host | production workstation, kernel 6.8.1-1062-realtime |
| Arm | left UR5e, 192.168.2.2, PolyScope 5.25.1, own factory calibration, adapter mounted (no instrument), payload 1 kg, External Control |
| Stop | ARM HOLD box (IDEM Viper `SCR73-280005`, S22 monitored manual start) into the SI pair of all three controllers (`safeguard_stop.md`); the other two arms idle |
| Driver | `ur_robot_driver` 3.10.0, `command_interface` velocity (`speedj`), controller as in `servoj_speedj_ab_20261030.md` |
| Motion | `e8_hold_real_arm.py` defaults: back and forth along base x, stroke 120 mm, constant speed 100 mm/s then 250 mm/s (reversals at 2 m/s²), analytic feedforward; 5 presses per speed by the operator at moments of their choosing; publishing stops at the UR's SAFEGUARD_STOP; 20 s requested hold; 3 s resume window with no command |
| Recording | `run_e8.sh`: `/tcp_pose_broadcaster/pose` (the UR's measured TCP, 500 Hz), `io_and_status_controller/{safety_mode, robot_program_running, robot_mode}`, `/joint_states`, the controller commands |
| Code | commit `cd84d60` (run); analyzer metrics revised in the commit that adds this report (below) |
| Data | machine-local `$ARTS_LOCAL/e8_20261101/`: `run/` (bag, `motion.log`, `analysis.txt`, `e8_result.json`, `settle.png`), `arm.log`, `teleop_hold/` (follower check) |

## Method

The stop instant t_s is the bag receive time of the SAFEGUARD_STOP message, which the UR
driver's GPIO controller publishes on change from the 500 Hz loop. The tool's held pose is
its mean position 1–2 s after t_s. Positions are averaged over 10 ms. Speed is not used to
detect rest: differentiating the UR's pose reads 2–15 mm/s of noise while frozen, because
its stamps jitter, whereas the position holds to ~10 µm.

- **t_1mm / t_0.1mm:** the last time within 2 s that the tool is farther than 1 mm / 0.1 mm
  from the held pose.
- **Distance:** held pose minus the tool at t_s.
- **Drift:** mean over the last 1 s before reset minus the held pose, and the largest
  deviation in between.
- **Resume:** the largest deviation over the 1.5 s after NORMAL.

**Analyzer check:** on a synthetic bag (250 mm/s, 10 ms delay, 5 m/s² to rest, true stop
60 ms, 8.75 mm), the analyzer gives t_1mm 40 ms and t_0.1mm 54 ms, which is what the
deceleration predicts, and a distance of 8.75 mm.

## Results

| Stop | Speed at t_s | t_1mm | t_0.1mm | Distance | Hold | Drift (max dev.) | Resume |
|---|---|---|---|---|---|---|---|
| 1 | 101.8 mm/s | 97.9 ms | 301.9 ms | 7.74 mm | 25.0 s | 0.008 (0.027) mm | 0.016 mm |
| 2 | 100.8 | 96.0 | 341.9 | 7.69 | 23.0 | 0.007 (0.028) | 0.022 |
| 3 | 103.0 | 97.9 | 348.0 | 7.86 | 20.5 | 0.005 (0.025) | 0.018 |
| 4 | 101.1 | 96.0 | 345.9 | 7.80 | 16.2 | 0.010 (0.027) | 0.017 |
| 5 | 100.6 | 95.8 | 297.8 | 7.72 | 17.0 | 0.008 (0.027) | 0.016 |
| 6 | 248.0 | 95.9 | 320.0 | 17.53 | 30.5 | 0.012 (0.031) | 0.018 |
| 7 | 239.3 | 97.8 | 310.0 | 17.70 | 14.2 | 0.014 (0.030) | 0.022 |
| 8 | 244.4 | 103.9 | 288.0 | 19.04 | 40.1 | 0.004 (0.026) | 0.014 |
| 9 | 242.9 | 103.9 | 293.8 | 18.81 | 23.2 | 0.011 (0.030) | 0.014 |
| 10 | 234.9 | 105.6 | 347.6 | 18.96 | 51.0 | 0.003 (0.023) | 0.019 |

| Speed | t_1mm | t_0.1mm | Distance |
|---|---|---|---|
| 100 mm/s (n = 5) | 96.7 ± 1.0 ms (max 97.9) | 327.1 ± 22.4 ms (max 348.0) | 7.76 ± 0.06 mm (max 7.86) |
| 250 mm/s (n = 5) | 101.4 ± 3.8 ms (max 105.6) | 311.9 ± 21.2 ms (max 347.6) | 18.41 ± 0.65 mm (max 19.04) |

Every stop has the same shape (`settle.png`): a deceleration of about 100 ms whatever the
speed, then a 0.1–0.3 mm plateau that settles by ~350 ms.

**Follower check (same session, teleop, deadman pedal).** The pedal was held and HOLD
pressed. During the HOLD and until a valid re-press, the handle moved over a 142 mm range
and the arm's largest joint moved 0.09°, which is the stop's own settle. Three pedal presses
made while HOLD was still in did not re-engage. The first press after the stop cleared
re-engaged the follower, and the arm followed with 2.0° over the next 1.3 s. The follower
logged "UR safety mode 5" at the stop and "safety hold cleared by the pedal" at the
re-press.

## Not measured / limitations

- The delay from the button to the UR's report (relay response plus safety-input
  filtering): the relay's spare contact is not wired to the PC. Every time is from the UR's
  report.
- One arm, one payload (1 kg, no instrument), one direction (base x), two speeds. The other
  two arms froze, but were not moving and were not measured.
- The UR's own safety configuration (stopping-time and stopping-distance limits) was not
  read out for this report.
- Distances are from the UR's measured TCP pose, not an external tracker.
