# Pose repeatability by optical tracker, left UR5e + ProGrasp, 2026-11-07

**Layer 4 (measured numbers): one session, immutable once committed.** Interpretation lives in
the AGENTS.md headline; the dVRK Classic numbers come from the collaborator separately.

## Setup

- Arm: left UR5e (`psm_left`), bare metal, `ur_robot_driver` 3.10.0, CartesianServoController
  at 500 Hz with the instrument wrist (`wrist:=true`, 9 joints, full orientation task), `speedj`.
  RCM OFF (no cannula; the adapter is built so that the free shaft length matches the dVRK's).
- Instrument: da Vinci ProGrasp forceps 470093 on adapter FTBEQBRA, kinematics file
  `470093_prograsp_forceps.yaml` (a copy of the bipolar forceps' values: same disc-to-joint
  directions, collaborator A; transmission ratios not measured). Session start: instrument off
  for the homing start, on, homed (29.4 s), visual re-zero after a bringup restart.
- Tracker: Atracsys fusionTrack over USB-Ethernet (172.17.1.7), SDK 4.7.1,
  sawAtracsysFusionTrack d5c508a (`~/atracsys_ws`, run with the SDK preloaded: see its NOTES),
  ~100 Hz. Marker: the collaborator's ProGrasp clip marker, screwed onto the closed jaws (the
  jaw is never commanded during the test). Geometry `geometry-prograsp-20261107.json`,
  re-measured this day after the spheres were remounted (`tracker_geometry_maker.py`: 933
  frames, per-sphere scatter 0.004 mm, FRE 0.0015 mm; the earlier geometry no longer matched,
  2.1 mm RMS). Measured point: the marker frame origin (sphere centroid), not the tool tip.
  Registration error at the taught poses 0.13-0.27 mm.
- Protocol (`repeatability_protocol.yaml`, collaborator A's procedure, ISO 9283 RP statistic):
  5 poses taught by gamepad jog, each refused unless the tracker saw the marker; every pair
  >= 26 mm and >= 34 deg apart (26-114 mm, 34-113 deg); all six UR joints move between poses
  (ranges 2.7-11.0 deg). P1 -> P5 x 30 cycles in POSE mode through `<arm>/tool_tip_target`
  (linear + slerp, one trapezoid, 20 mm/s and 15 deg/s cruise, 0.1 m/s^2). Static rule: hold
  >= 3 s, then a 1 s window with every axis std <= 0.03 mm and half-window drift <= 0.02 mm with
  the marker tracked throughout, then the NEXT 1 s window must pass the same test and is the
  sample; no static window in 30 s = a miss. Tracker poses with a zero header stamp (the node
  repeats the last pose that way when the marker is lost) are not measurements and are excluded.
- Code: `1775097` plus the uncommitted bench scripts listed in `run_info.txt` (machine-local).
- Data (machine-local): `$ARTS_LOCAL/repeat_20261107/run1` (bag, analysis.txt, results.json,
  run_info.txt), poses `$ARTS_LOCAL/repeat_20261107/poses.yaml`; dry run (1 cycle) in `dry/`.

## Results

150 stops in 30 cycles, 1234 s; 0 misses; 11 measured windows discarded and re-taken (the
second window failed the static test); settling (arrival to measured window) median 3.00 s,
max 5.60 s.

Tracker (`/atracsys/Prograsp/measured_cp`; l = distance of each attained position from its
pose's barycentre; RP = mean(l) + 3 std(l); RPa/b/c = 3 std of the orientation angles):

| Pose | n | RP mm | mean l | std l | RMS l | max l (mm) | RPa / RPb / RPc deg |
|---|---|---|---|---|---|---|---|
| P1 | 30 | 0.139 | 0.061 | 0.026 | 0.066 | 0.124 | 0.120 / 0.219 / 0.123 |
| P2 | 30 | 0.203 | 0.101 | 0.034 | 0.107 | 0.172 | 0.411 / 0.453 / 0.096 |
| P3 | 30 | 0.142 | 0.052 | 0.030 | 0.060 | 0.150 | 0.230 / 0.229 / 0.201 |
| P4 | 30 | 0.252 | 0.096 | 0.052 | 0.109 | 0.208 | 0.280 / 0.293 / 0.413 |
| P5 | 30 | 0.215 | 0.080 | 0.045 | 0.091 | 0.192 | 0.233 / 0.380 / 0.187 |
| pooled | 150 | | 0.0781 | 0.0426 | 0.0889 | 0.2078 | |

Worst pose P4: RP 0.252 mm.

Kinematic, same stops (`/psm_left/tool_tip_state`, forward kinematics of the measured joints):
RP 0.012-0.020 mm per pose (worst P4 0.020), orientation 0.017-0.114 deg; pooled mean 0.0055,
std 0.0036, RMS 0.0066, max 0.0154 mm.
