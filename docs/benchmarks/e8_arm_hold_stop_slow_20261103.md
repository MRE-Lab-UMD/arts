# E8 (slow) — ARM HOLD stopping distance at surgical speeds, left UR5e, 2026-11-03

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Complements `e8_arm_hold_stop_20261101.md` (100 and 250 mm/s) with the same protocol at
> 10 and 20 mm/s; tools `arts_controllers/bench/run_e8.sh`, `e8_hold_real_arm.py`,
> `e8_analyze.py`.

**Result.** ARM HOLD pressed 10 times while the left UR5e moved under the servo controller.
From the UR's safeguard-stop report, the tool stopped in **0.72 ± 0.01 mm** at 10 mm/s (max
0.74 mm) and **1.46 ± 0.01 mm** at 20 mm/s (max 1.47 mm), within 0.1 mm of its final pose in
88.0 ± 4.9 ms and 112.0 ± 20.1 ms. Holds of 16–31 s drifted at most **0.006 mm** (largest
deviation 0.023 mm), and the reset moved the tool at most **0.019 mm**. With the 2026-11-01
session the stopping distance is nearly proportional to speed (about 73–78 ms of travel at
10, 20, 100 and 250 mm/s).

## Setup

As `e8_arm_hold_stop_20261101.md` (host, arm, stop wiring, driver, recording), except:

| | |
|---|---|
| Arm | left UR5e, adapter mounted (no instrument), payload 1 kg, External Control, driver default `command_interface` velocity (`speedj`); arm launched by `arts_bringup psm_left:=true psm_left_tool:=large_needle_driver` (da Vinci X tip preset) |
| Motion | `e8_hold_real_arm.py --speeds 0.01,0.02 --trials 5`: back and forth along base x, stroke 120 mm, constant speed 10 mm/s then 20 mm/s, analytic feedforward; presses by the operator at moments of their choosing; 20 s hold |
| Code | commit `10c011a` |
| Data | machine-local `$ARTS_LOCAL/e8_slow_20261103/`: `bag`, `motion.log`, `analysis.txt`, `e8_result.json` |

## Method

Same analyzer and definitions as the 2026-11-01 report. At these speeds the whole stop is
shorter than 1 mm, so `t_1mm` is 0 at 10 mm/s and not meaningful; `t_0.1mm` and the distance
carry the result. The "speed at t_s" column is the analyzer's estimate from the UR pose,
which reads a few mm/s of noise (2026-11-01 report).

## Results

| Stop | Speed at t_s | t_1mm | t_0.1mm | Distance | Hold | Drift (max dev.) | Resume |
|---|---|---|---|---|---|---|---|
| 1 | 8.8 | 0.0 | 87.6 | 0.74 | 23.3 | 0.003 (0.017) | 0.014 |
| 2 | 10.7 | 0.0 | 91.6 | 0.72 | 20.9 | 0.003 (0.019) | 0.016 |
| 3 | 9.1 | 0.0 | 89.6 | 0.73 | 26.0 | 0.003 (0.023) | 0.019 |
| 4 | 10.2 | 0.0 | 91.6 | 0.72 | 22.3 | 0.004 (0.020) | 0.017 |
| 5 | 11.0 | 0.0 | 79.7 | 0.73 | 21.6 | 0.002 (0.017) | 0.013 |
| 6 | 20.8 | 25.4 | 101.4 | 1.46 | 27.1 | 0.003 (0.017) | 0.013 |
| 7 | 19.4 | 25.4 | 103.4 | 1.47 | 31.2 | 0.002 (0.018) | 0.014 |
| 8 | 20.1 | 23.7 | 101.7 | 1.45 | 16.0 | 0.003 (0.018) | 0.014 |
| 9 | 19.5 | 23.7 | 147.7 | 1.47 | 22.4 | 0.002 (0.019) | 0.014 |
| 10 | 20.8 | 23.7 | 105.6 | 1.44 | 29.4 | 0.006 (0.022) | 0.015 |

| Speed | t_1mm | t_0.1mm | Distance |
|---|---|---|---|
| 10 mm/s (n = 5) | 0.0 (stop shorter than 1 mm) | 88.0 ± 4.9 ms (max 91.6) | 0.72 ± 0.01 mm (max 0.74) |
| 20 mm/s (n = 5) | 24.4 ± 0.9 ms (max 25.4) | 112.0 ± 20.1 ms (max 147.7) | 1.46 ± 0.01 mm (max 1.47) |

Stopping distance against speed, both sessions: 0.72 mm at 10, 1.46 mm at 20, 7.76 mm at 100,
18.41 mm at 250 mm/s; distance / speed = 72, 73, 78, 74 ms.

## Not measured / limitations

As the 2026-11-01 report: the button-to-report delay is not measured; one arm, one payload
(1 kg, no instrument), one direction (base x); distances from the UR's measured TCP pose (the
flange side of the adapter, not an instrument tip), not an external tracker.
