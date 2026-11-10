# Camera-sync PCB v2 — Stage D re-run after the edge-classification fix, 2026-10-28

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Procedure: `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/design_package/04_test_procedure.md`, Stage D.
> Raw analyzer output: `camera_sync_skew_v2pcb_fix_20261028.json` (same directory).
> Earlier run the same day, before the fix: `camera_sync_skew_v2pcb_20261028.md`.

This is the committed stereo skew baseline (paper experiment E3). It supersedes the earlier
run of the same day, which failed the missed-strobe criterion.

**Result: Stage D PASS on every criterion.** Hardware skew, left − right, signed:
**−2.793 ± 3.209 µs** (n = 8991, every pulse), worst |skew| **10.825 µs**; **0 missed strobes**
on either camera.

## Setup

Identical to `camera_sync_skew_v2pcb_20261028.md` (same board and measured resistors, same
cameras on the same terminals and host ports, same camera configuration, same agent and
component pins, cameras composed, driver session not restarted in between), except:

| | |
|---|---|
| Firmware | MICROROS, app version `7859c33` (clean tree): strobe start/end decided from the pin level read in the capture ISR instead of the driver's `cap_edge`; first start edge per record wins |
| Bag | 299.7 s MCAP: `/arts_camera_sync/pulse` 8991, `/camera_left/meta` 8990, `/camera_right/meta` 8990. Machine-local at `$ARTS_LOCAL/benchmark_bags/camera_sync_skew_v2pcb_fix_20261028/` (mcap sha256 prefix `6996fb8f6ea511b5`) |

Sign convention: cam0 − cam1 = LEFT − RIGHT = 24057132 − 24057131.

## Diagnosis (SELFTEST, `CONFIG_ARTS_SYNC_EDGE_DIAG=y`, ~9 min each)

| Firmware | Pulses | Start edges reported NEG with pin high | Pulses with no strobe (cam0 / cam1) | Other anomalies |
|---|---|---|---|---|
| before fix (`cap_edge`) | 16 157 | 18 | 3 / 15 | 1 record, both cameras: extra rising edge at 793.39 / 793.40 µs (real starts 712.26 / 712.84 µs), recorded as the start |
| after fix (pin level) | 16 159 | 18 | **0 / 0** | strobe-glitches counter 0 |

Every mislabelled edge, in both runs: capture time 707.7–716.2 µs after the trigger (an
exposure start), driver-reported edge NEG, pin level 1 when read in the ISR, followed by the
true end edge (NEG, level 0) at ~5687 µs. No edge in either run had a reported type that
disagreed with the pin level other than these. The first-start-wins rule did not fire in the
post-fix run; the extra edge it guards against occurred once in 16 157 pulses before the fix.

## D3 — micro-ROS bag

| | |
|---|---|
| Pulses / index span / gaps | 8991 / 8991 / **0** |
| `time_synced` | true on every message |
| Trigger period | 33 333.000 µs, std 0.000, spread **0.000 µs** |

### (a) Hardware

| | cam0 / LEFT (24057132) | cam1 / RIGHT (24057131) |
|---|---|---|
| Trigger → exposure | 709.312 ± 2.262 µs | 712.104 ± 2.264 µs |
| Range | [704.962, 713.763] µs | [707.513, 716.538] µs |
| Exposure width | 4976.477 ± 0.043 µs | 4977.192 ± 0.043 µs |
| Pulses with no capture | **0** / 8991 | **0** / 8991 |

**Skew, LEFT − RIGHT, signed, n = 8991:**

| | |
|---|---|
| Mean (systematic) | **−2.793 µs** |
| Std | **3.209 µs** |
| Range | [−10.825, +5.425] µs |
| Worst \|skew\| | **10.825 µs** |

### (b) Software cross-check — `camera_time`

| | |
|---|---|
| Paired frames | 8990 |
| Drift | +2.485 µs/s |
| **Detrended residual std** | **3.235 µs** (hardware std 3.209 µs) |
| Per-pulse join (frame k−1 ↔ pulse k) | r = **0.9904**, n = 8990 |
| Per-pulse residual | std **0.447 µs**, range [−1.401, +1.276] µs |
| Frame gaps | left 0, right 0 |
| Implied strobe rise delay | L 27.523 µs, R 26.808 µs |

### Capture tick → ROS time fit

Crystal ratio error 7.415 ppm; residual std 181.5 µs (includes micro-ROS transport).

## Acceptance (Stage D)

| | Target | Result | |
|---|---|---|---|
| Skew, hardware | < 1 ms (expect < 100 µs) | mean −2.793 µs, worst \|10.825\| µs | PASS |
| Trigger period spread | ≤ 1 tick | 0.000 µs | PASS |
| Missed strobes | 0, excluding pulses where the camera did not expose | 0 of 8991 on both cameras | PASS |
| Software cross-check | `camera_time` residual << 1 ms | 3.235 µs detrended; 0.447 µs per pulse | PASS |
| Systematic | report; expect ≤ 2.81 µs | −2.793 µs | PASS |

## Not measured in this run

The strobe-glitch counter exists only in SELFTEST; `CameraSyncPulse` does not carry it, so
this bag cannot show whether the first-start-wins rule fired during it.
