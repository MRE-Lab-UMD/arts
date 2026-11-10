# Automatic instrument homing — cautery hook on psm_left, 2026-11-04

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Method and status: `docs/modules/dv_tool_driver.md`, "Automatic homing". Tools:
> `arts_dv_tool_driver` `tool_homing` (launch `tool_homing.launch.py arm:=psm_left tool:=hook`),
> `test/photo_pitch_angle.py`.

**Result.** Ten homings of a Permanent Cautery Hook (470183), each after removing and refitting
the instrument, all COMPLETE, 41.2 ± 2.7 s each (38.2–45.0 s). Home position in motor counts
(4096 per motor turn): roll repeats within **3 ticks** (0.25° of the roll joint, disc = 1.07 ×
joint) and the hook disc M5 within **21 ticks** (std 8.8). Pitch motor counts do NOT repeat:
the pitch center rose from 3826 to 4292 over the ten fits (both pitch limits move with it,
range 1735–1761, twice 1886/1898). Measured physically on runs 6–10, the homed pitch angle
repeats to **0.52° std, 1.22° max − min** (n = 5 photos, includes clicking error).

## Setup
- Adapter FTBEQBRA (FT232H serial) on psm_left (the adapters were swapped between the arms
  2026-11-04 ~10:50; checked this session by hand: torque held only the left adapter's discs).
- XC330-M288 ×5, protocol 2.0, 2 Mbps, Return Delay Time 0. Homing I/O at 333 Hz, logic 50 Hz.
- `tool_homing` from arts commit 719acef (the adapter designer's evening-2026-11-04 node, values
  unchanged): `config/homing/hook.txt`, `adapter_FTBEQBRA.txt` (start M1..M5 = 100, 9096, 2100,
  3236, 2824). Hook homing = roll (M1) and pitch (M4) to the midpoint of their stops, then M5
  between its stops; M2 stays at its start position.
- Per run: instrument off → launch (discs to start positions) → fit hook → gamepad A →
  COMPLETE. Runs 6–10: photo while the node held the home position (torque on), then stop.
- Raw data (machine-local): `$ARTS_LOCAL/tool_homing_logs/` — `lim_homing_summary.csv`,
  `rollpitch_*.csv` (every 50 Hz sample), `run_hook_psm_left_<n>.log`, `photos/` (iPhone,
  4032×3024, fixed position, view perpendicular to the pitch plane) and `photos/angles.csv`
  (every click).

## Per run (motor counts)

| Run | Time | Roll L1 / L2 | Roll center | Pitch L1 / L2 | Pitch range | Pitch center | M5 L1 / L2 | M5 center | Total s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 22:13:40 | 3413 / −4114 | −350 | 4698 / 2955 | 1743 | 3826 | 4484 / 2531 | 3507 | 38.2 |
| 2 | 22:18:48 | 3412 / −4115 | −351 | 4730 / 2995 | 1735 | 3862 | 4482 / 2534 | 3508 | 38.4 |
| 3 | 22:26:05 | 7509 / −19 | 3745 | 4754 / 3019 | 1735 | 3886 | 4483 / 2539 | 3511 | 42.9 |
| 4 | 22:28:16 | 7510 / −19 | 3745 | 4856 / 2970 | 1886 | 3913 | 4483 / 2538 | 3510 | 43.9 |
| 5 | 22:31:38 | 7510 / −19 | 3745 | 4827 / 3090 | 1737 | 3958 | 4483 / 2556 | 3519 | 42.4 |
| 6 | 22:46:40 | 3412 / −4115 | −351 | 5073 / 3330 | 1743 | 4201 | 4500 / 2557 | 3528 | 39.0 |
| 7 | 22:49:20 | 7510 / −19 | 3745 | 5104 / 3358 | 1746 | 4231 | 4489 / 2567 | 3528 | 43.7 |
| 8 | 22:51:04 | 3412 / −4116 | −352 | 5132 / 3381 | 1751 | 4256 | 4498 / 2555 | 3526 | 39.6 |
| 9 | 22:52:46 | 7510 / −20 | 3745 | 5168 / 3270 | 1898 | 4219 | 4483 / 2550 | 3516 | 45.0 |
| 10 | 22:54:20 | 3413 / −4112 | −349 | 5173 / 3412 | 1761 | 4292 | 4497 / 2560 | 3528 | 38.5 |

Roll centers modulo 4096: 3744–3747 (std 0.79 tick). Pitch limit 2 was a "push stop" (motor
at its current cap while still moving) in all ten runs; limit 1 also in runs 2, 4 and 10; M5
limit 2 in run 2. Runs 1–5 and 6–10
were separated by ~15 min with the hook off. One `There is no status packet!` on run 1's first
packet after opening the port; none in runs 2–10.

## Physical pitch angle (runs 6–10)

Angle in the image from the shaft to the hook's distal body (mean of two clicked edges each,
8 clicks per photo, one pass by the operator; tool math checked on synthetic edges at 0, ±10
and 30°).

| Run | Pitch center (counts) | Angle |
|---|---|---|
| 6 | 4201 | −4.604° |
| 7 | 4231 | −4.709° |
| 8 | 4256 | −5.350° |
| 9 | 4219 | −4.131° |
| 10 | 4292 | −5.338° |

Mean −4.83°, std 0.52°, max − min 1.22°. Correlation of angle with pitch center counts:
r = −0.81 (n = 5). The 91-tick count spread over these runs would be 8.5° of pitch if it were
physical (disc 4 = 0.94 × pitch, `config/instruments/470183_permanent_cautery_hook.yaml`).
Not measured: clicking error (no second pass), out-of-plane tilt of the pitch plane.

`disc_drive` after run 2 (`zero_from_homing:=true`, 470183): discs within 3 ticks of the homed
counts; joints roll 0.000, pitch 0.0016, yaw −0.0012, jaw −0.0042 rad. Operator's visual check
at home: shaft straight, roll natural.
