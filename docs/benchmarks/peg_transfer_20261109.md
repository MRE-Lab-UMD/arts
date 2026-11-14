# FLS peg transfer, RCM on, 3 experts + 3 novices, 2026-11-09

**Layer 4 (measured numbers): one session, immutable once committed.** Procedure:
`docs/modules/bench_session_20261109.md`. Interpretation lives in the AGENTS.md headline.

## Setup

- Arms: both UR5e PSMs with 471006 large needle drivers (`wrist:=true`, tool actuators on), the
  ECM holding the stereo endoscope, all three with the RCM ON at the trainer-box ports for every
  trial. Head sensor, clinical-mode pedals (clutch / camera / record), VerseGrip handles on
  Inverse3. Code `2f2d337` (`run_info.txt` per trial).
- Task: FLS board in the trainer box, 3 green triangles left to right with a mid-air handoff,
  then 3 orange right to left; 300 s cap. One recorded trial per operator after 5 min of
  unrecorded practice. Expert = 2+ years of surgical-robot teleoperation; novice = none.
- Counts: transferred and drops out of view from the end frame (checked against the endoscope
  video); time from the record-pedal episode marks; behaviour from the state bag
  (`peg_transfer_analyze.py`). Data (machine-local): `$ARTS_LOCAL/peg_20261109_results/`
  (analysis, annotation, run_info), bags on the external SSD.

## Results

| Trial | Group | Time (s) | Transferred | Drops out of view | Clutch L / R | Tip path L / R (mm) | RCM median L / R (mm) | RCM p95 L / R | RCM max L / R |
|---|---|---|---|---|---|---|---|---|---|
| E1 | expert | 144.8 | 6/6 | 0 | 0 / 0 | 1071 / 968 | 0.034 / 0.027 | 0.085 / 0.063 | 0.195 / 0.151 |
| E2 | expert | 126.4 | 6/6 | 0 | 1 / 1 | 694 / 717 | 0.030 / 0.027 | 0.084 / 0.065 | 0.180 / 0.140 |
| E3 | expert | 93.2 | 6/6 | 0 | 1 / 1 | 914 / 894 | 0.029 / 0.026 | 0.082 / 0.065 | 0.181 / 0.147 |
| N1 | novice | 162.1 | 6/6 | 0 | 8 / 4 | 1421 / 1290 | 0.029 / 0.023 | 0.088 / 0.066 | 0.252 / 0.152 |
| N3 | novice | 300 (cap) | 5/6 | 0 | 8 / 8 | 1092 / 877 | 0.025 / 0.022 | 0.096 / 0.058 | 0.184 / 0.138 |
| N4 | novice | 219.1 | 6/6 | 0 | 5 / 5 | 1577 / 1302 | 0.034 / 0.026 | 0.089 / 0.063 | 0.215 / 0.147 |

- Experts: 3/3 completed, 93.2-144.8 s (mean 121.5 s), clutch 0-1 per arm, tip path 694-1071 mm
  per arm.
- Novices: 2/3 completed (162.1 and 219.1 s); N3 reached the 300 s cap with 5/6 (the sixth in the
  left tool, in view). Clutch 4-8 per arm, tip path 877-1577 mm per arm.
- Kinematic RCM deviation (distance from the RCM point to the shaft line by forward kinematics),
  all 12 arm-trials: median 0.022-0.034 mm, p95 at most 0.096 mm, maximum 0.252 mm.
- 0 camera moves (the endoscope stayed where it was set), 0 collision-guard holds, 0 triangles
  lost out of view, in all six trials.
- N3 has no END press (record window 635 s); its behaviour columns are recomputed over the first
  300 s (`N3/analysis/capped_300s.json`).

## Exclusions and deviations

- **N2 excluded, replaced by N4** (equipment failure): the left VerseGrip stopped sending at
  t = 209.4 s, freezing the left instrument's orientation with the jaw open, so the remaining
  handoffs were impossible; N2 ran to the cap. All N2 data checks passed otherwise.
- E2: data check FAIL on the ECM joint-state gap (21.6 ms against a 20 ms limit; the ECM was
  stationary throughout). No PSM check failed; kept, cause unknown.
- E3 and N3 moved the orange triangles first (protocol: green first).
- E1: the right instrument's orientation was found drifted after E1 and re-zeroed before E2.
  The VerseGrip roll map changed from E3 on (operator preference).
- The test run (system lead, 94.0 s, 5/6) is not an E/N trial.

## Drops in view (PI, from the endoscope videos, 2026-11-10)

E1, E2, E3: 0. N1: 0. N3: 1. N4: 0. Failed handoffs are not reported.
