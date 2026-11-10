# RCM drift by optical tracker, left UR5e + ProGrasp, 2026-11-07

**Layer 4 (measured numbers): one session, immutable once committed.** Interpretation lives in
the AGENTS.md headline.

## Setup

- Arm: left UR5e (`psm_left`), bare metal, `ur_robot_driver` 3.10.0, CartesianServoController
  at 500 Hz with the instrument wrist (`wrist:=true`), `speedj`. RCM ON: task QP (hard RCM rows
  + collaborator A's roll row) -> Ruckig -> RCM projection. Point placed by `set_rcm.py on`
  on the current shaft, 100 mm behind frame E (the wrist pitch axis), in free air (no cannula),
  wrist straight. Code: `1775097` plus the uncommitted changes of this day, including the
  `set_rcm` fix (the point is now placed on the SHAFT axis, frame E, with the wrist; before, it
  was placed along the jaw axis and the per-cycle check latched a fault at any wrist bend).
- Instrument: da Vinci ProGrasp 470093 on adapter FTBEQBRA, visual zero; wrist held straight by
  the orientation task (pitch and yaw within 0.5 deg during the motion).
- Tracker: Atracsys fusionTrack (see `repeatability_psm_left_20261107.md`), ~100 Hz.
  Marker: the collaborator's shaft (vacuum) marker clamped on the instrument shaft by a set
  screw; geometry `geometry-vacuum.json` (marker-centred origin, registration error 0.18 mm).
  No pre-calibrated pivot geometry is used: the pivot point is fitted from the tracked motion.
- Alignment: under a 15 deg wobble about the RCM (`rcm_motion_real_arm.py wobble`) the live
  pivot fit (`pivot_monitor.py`) gives the marker origin's offset from the pivot, split along
  and across the shaft; the marker was slid until along was within 0.5 mm (final -0.33 to
  -0.46 mm; across 29.08-29.10 mm, the marker's own stand-off from the shaft axis; fit 0.23 mm
  RMS) and the set screw tightened.
- Motion (`rcm_motion_real_arm.py drift`): the start tip pose rigidly rotated about the RCM
  point, pivot (two axes normal to the shaft) and roll each a seeded sum of three sinusoids
  (periods 7-23 s), bounded to pivot <= 15 deg and roll <= 10 deg, rate <= 15 deg/s, 3 s ramps.
  The shaft was first rolled to +30 deg, where the marker faces the tracker (dry runs: at the
  earlier start roll the marker was tracked only for relative roll >= +10 deg; with pivot
  20 deg an elbow excursion of 45.2 deg tripped the script's 45 deg joint guard).
- Metric: per trial, the pivot point and the marker-frame offset d are fitted by linear least
  squares over all tracked samples (R_i d + p_i = t); pivot error = |R_i d + p_i - t| per
  sample. Drift: the pivot fitted per 60 s window, displacement from the first window and its
  slope. Tracker poses with a zero header stamp (marker not seen) are excluded. Kinematic
  deviation: `/psm_left/rcm_state`, the open laparoscopic platform's definition.
- Data (machine-local): `$ARTS_LOCAL/rcm_drift_20261107/{dry,dry2,dry3,trial1,trial2,trial3}`.

## Results

Three 5 min trials, RCM [589.5, 586.0, -13.0] mm in `psm_left_base_link`, insertion 110.3 mm
(tip beyond the point):

| Trial | seed | tracker samples | rotation range | pivot error mean +- std (mm) | RMS | p95 | max | drift: max displacement, slope | kinematic median / p95 / max (mm) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 29 773 | 19 deg | 0.335 +- 0.165 | 0.373 | 0.649 | 0.933 | 0.053 mm, 0.0054 mm/min | 0.0260 / 0.0636 / 0.1503 |
| 2 | 2 | 28 564 | 18 deg | 0.351 +- 0.178 | 0.394 | 0.688 | 1.066 | 0.086 mm, 0.0088 mm/min | 0.0260 / 0.0635 / 0.1475 |
| 3 | 3 | 28 881 | 20 deg | 0.366 +- 0.183 | 0.409 | 0.719 | 1.090 | 0.049 mm, 0.0078 mm/min | 0.0273 / 0.0663 / 0.1644 |

Marker origin from the fitted pivot |d| 27.40 / 27.40 / 27.66 mm. Dry run 3 (1 min, same
limits): pivot error 0.335 +- 0.169 mm, max 0.882 mm.
