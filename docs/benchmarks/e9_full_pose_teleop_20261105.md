# E9 — full-pose teleop with the instrument wrist, left UR5e, 2026-11-05

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E9 (`publication_plan.md` in the manuscript repo). Method and the fixes that
> preceded it: decision 17, amendment 2026-11-05. Tools `arts_teleop/test/run_teleop_real.sh`,
> `arts_driver` `cameras.launch.py image_bag:=`.

**Result.** The Haply Inverse3 position and the VerseGrip orientation drove the needle driver's
tip through the UR5e (6 joints) and the instrument (roll, wrist pitch, wrist yaw) plus the jaw,
recorded with all 6 + 4 joints and both endoscope cameras. Run 2 (the reported run): **228 s
engaged in one engagement**; instrument roll spanned **202 deg** (8 to 210), wrist pitch
**95 deg** (-69.5 to 25.4), wrist yaw **115 deg** (-54.1 to 61.2), jaw command the full
-10..90 range with full grip (-10) for 3150 messages; UR joint spans at most 16.2 deg.
11 835 stereo pairs (1536 x 1024 raw Bayer, triggered 30 Hz), equal counts left and right.

## Setup

- Arm psm_left (UR5e), `ur_arm.launch.py arm:=psm_left wrist_instrument:=471006_large_needle_driver.yaml`:
  9-joint servo controller, QP solver, joint weights UR 1.0 / instrument 0.01, tip caps
  250 mm/s and 90 deg/s, RCM off. External Control, `speedj`.
- Instrument: Large Needle Driver 471006 on adapter FTBEQBRA, `disc_drive` at 2 Mbps,
  `zero_from_homing:=true` (home file 2026-11-04T23:34:10, counts [-353, 8792, 2100, 3187,
  3082]), then `set_zero` at counts [-356, 8758, 2100, 3184, 3097] with no touch-up (operator
  judged it straight). Jaw: unitless command, grip cap 1000 at -10.
- MTM: left Inverse3 (05E4) + left VerseGrip (1245), Haply Hub 1.5.0, service 3.5.7, VerseGrip
  v3.1; gravity compensation 0.030 kg; pen calibrated by its button before the runs.
- Follower (`config/teleop.yaml` at arts 6d62ddb): deadman mode (left pedal held), scale
  0.25, position box +-50 mm, room frame (`operator_to_base` from gravity and the ECM's
  heading), `stylus_calibration` pen +y = tip, alignment gate 10 deg, angular feedforward,
  bend limit 60 deg from the shaft.
- Cameras: BFS-U3-63S4C L 24057131 / R 24057132, sync box v2 PCB, `trigger:=true`, 2x2
  binning, written to the system NVMe; view on the tower monitor only (GOOVIS not used).
- Code: arts 6d62ddb (both runs, from `run_info.txt`).

## Runs

| | Run 1 (13:46:53) | Run 2 (13:56:41) |
|---|---|---|
| Teleop window | 5 min | 8 min (stopped at 6.4 min, operator done) |
| State bag length | 308 s | 386 s |
| Engaged | 173 s, 3 engagements | 228 s, 1 engagement |
| Roll | 20.8 .. 205.4 deg | 8.0 .. 209.8 deg |
| Wrist pitch | -68.8 .. 41.6 deg | -69.5 .. 25.4 deg |
| Wrist yaw | -43.9 .. 59.7 deg | -54.1 .. 61.2 deg |
| Jaw joint | -11.2 .. 85.3 deg | -10.8 .. 90.7 deg |
| Jaw command | -10.0 .. 80.5; 1094 msgs at -10 | -10.0 .. 90.0; 3150 msgs at -10 |
| UR joint spans (deg) | 5.1 5.1 4.8 1.6 5.9 5.5 | 16.2 7.5 4.1 10.0 6.1 8.0 |
| Bend-limit warnings (throttled 2 s) | 34 | 81 |
| Image bag | 390.3 s, 11 570 + 11 570 frames | 399.3 s, 11 835 + 11 835 frames |

Joint ranges are the instrument's measured joints (`/psm_left/tool_joint_states`); UR spans
from `/psm_left/joint_states`. Pitch reached -68.8 / -69.5 deg with the bend limit set to
60 deg (the limit acts on the target, measured from the shaft as the UR holds it).

Raw data (machine-local): `$ARTS_LOCAL/e9_20261105/e9_run{1,2}/` — `teleop/bag` (state:
handle, pen, pedal, targets, 6 + 4 joints, jaw command, tf), `teleop/*.log`, `images/`
(both cameras, camera_info, meta), `cameras.log`.
