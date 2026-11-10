# E6 — teleop motion-to-motion latency, left UR5e, speedj, 2026-10-30

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E6 (`publication_plan.md` in the manuscript repo). Tools
> `arts_teleop/bench/e6_track_markers.py`, `e6_latency.py`, `e6_bag_latency.py`.

**Result.** From the Haply handle moving to the arm's adapter moving, both filmed in one
120 fps frame: **58 ms and 54 ms** (lag of best fit over each of the two usable takes);
over 3 s windows, median **45 ms**, range 14–61 ms (n = 64). The bag, on software stamps
(handle frame received → measured joints), gives the same: 58 ms over the session, windows
median 45 ms, range 33–62 ms (n = 105). Target ≤ 100 ms.

## Setup

| | |
|---|---|
| Master | Haply Inverse3 05E4 (left), firmware 7.6, Hub 1.5.0 / service 3.5.7; `arts_driver/haply_bridge` at 500 Hz |
| Follower | `arts_teleop/teleop_follower.py`, deadman mode (pedal held = following), scale 0.25, box ±50 mm, drop guard 0.8 m/s; `operator_to_base` = `[0.7071068,-0.7071068,0, 0,0,1, -0.7071068,-0.7071068,0]` (axis map rated "roughly correct" by the operator) |
| Arm | left UR5e, adapter mounted, payload 1 kg, `command_interface:=velocity` (`speedj`), controller as in `servoj_speedj_ab_20261030.md` |
| Camera | iPhone 18 Pro, iOS 27.0.1, 120 fps, 1920×1080 (portrait), tripod; both markers in frame |
| Markers | ArUco DICT_4X4_50, 40 mm: ID 0 on the handle, ID 1 on the adapter |
| Motion | operator, 3 takes of ~40 s: hold, then back and forth slow → ~2 Hz → a few sharp shakes, pedal held |
| Session | `run_teleop_real.sh`, 6 min, bag of handle, pedal, commands, `/joint_states`, `/tf` |
| Code | commit `9e4387b` (run); analysis tools added with this report |
| Data | machine-local `$ARTS_LOCAL/e6_20261030/`: `IMG_8593/8594/8595.MOV`, `track_*.csv`, `teleop/bag` and logs, `video_latency.txt`, `bag_latency.txt`, `e6_result.json` |

## Method

**Video (main number).** Each frame's marker centres are converted to mm with the marker's
own side length; frame time is index / 120 (the iPhone's packet times step 8.337 ms on
average). Velocities are taken on a 1 ms grid after 30 ms smoothing. Because handle and arm
move in different image directions and at different scales, the arm's image velocity at
t + L is fitted as a linear 2×2 map of the handle's at t, over samples where the arm moves
(> 5 mm/s); the lag is the L (0–299 ms) with the best R². Windows: the same fit per 3 s
window (1 s step), kept when R² > 0.8. Check: motion onsets, the handle leaving rest
(> 20 mm/s after 0.5 s below) and the arm crossing the matched threshold (20 mm/s × the
fitted speed gain).

**Bag (software path).** The same fit, handle Cartesian velocity (`/haply/left/cursor`,
stamped on receipt by the bridge) to joint velocity (`/joint_states`, stamped on receipt by
the driver).

**Estimator check.** The arm track of IMG_8593 and IMG_8595 delayed by 30 ms in software:
take lags 58 → 88 and 54 → 84 ms, window median 45 → 75 ms.

## Results

| Take | Arm moving | Lag (take) | R² | Windows: n, median, IQR, range | Onsets |
|---|---|---|---|---|---|
| IMG_8593 | 34.8 of 43.0 s | **58 ms** | 0.987 | 36, 46, 35–56, 14–61 ms | none detected |
| IMG_8594 | 27.2 of 40.3 s | (101 ms) | 0.334 | 0 | 51 ms |
| IMG_8595 | 30.6 of 44.0 s | **54 ms** | 0.851 | 28, 44, 40–49, 35–54 ms | 33, 59 ms |
| Bag, whole session | 107.1 s | **58 ms** | 0.948 | 105, 45, —, 33–62 ms (mean 46.7 ± 8.2) | — |

**IMG_8594 is not usable for the image method:** the handle moved mostly along the camera's
line of sight (marker 0 apparent size varied 4.7 % std, against 0.9 % in IMG_8593, while its
in-image motion was 7 mm std against 31 mm). Its lag is shown in parentheses and excluded.

Onsets: only 3 events qualified; the operator rarely came to rest between motions, so the
onset check is weak (33–59 ms, median 51 ms).

During the session the drop guard fired twice (handle at 0.80–0.81 m/s) and was cleared by
the pedal.

**Same estimator on the earlier `servoj` session** (left arm, same limits, 2026-10-30
session 3, `$ARTS_LOCAL/teleop_left_20261030/teleop3_fast`): 84 ms over the session, windows
median 75 ms, range 52–94 ms (n = 110). Software stamps only; no video.

## Not measured / limitations

- One operator, one arm, no instrument, free space; the GOOVIS / camera view was not in the
  loop (the operator watched the arm directly).
- Two usable takes. The main number comes from fits dominated by the faster motion; the
  window spread (14–61 ms) shows the lag is not a constant.
- The Haply is on firmware 7.6 / VerseGrip v3.1 (the Hub 1.5 firmware did not flash).
- The axis map was approximate; the estimator does not depend on it.
