# Camera-sync PCB v2 — Stage D, committed stereo skew baseline, 2026-10-28

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Procedure: `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/design_package/04_test_procedure.md`, Stage D.
> Raw analyzer output: `camera_sync_skew_v2pcb_20261028.json` (same directory).

This is the first authoritative skew baseline (paper experiment E3). The breadboard figures
(`camera_sync_stereo_skew_20260819.md`) are bring-up results and are superseded.

**Result:** hardware skew, left − right, signed: **−2.707 ± 3.208 µs** (mean ± std, n = 8975),
worst |skew| **11.075 µs**. Software cross-check from `camera_time`: detrended std **3.230 µs**;
per-pulse residual against hardware **0.430 µs** std. Every skew criterion passes. The
missed-strobe criterion **fails**: 5 (left) + 11 (right) of 8991 pulses had no capture
although the camera exposed (see "Missed strobes").

## Setup

| | |
|---|---|
| Board | `arts-camera-sync-v2`, first fabricated revision, the unit of Stages A–C (not serialized) |
| Measured resistors (Stage A) | trigger series J1 559.3 Ω, J2 560.0 Ω, J3 560.7 Ω; strobe pull-ups J1 991 Ω, J2 990 Ω, J3 994 Ω |
| Module | ESP32-DevKitC-32E, ESP32-D0WD-V3 rev v3.1 |
| Host | production workstation; cameras composed in one `ComposableNodeContainer` |
| J1 / cam0 | BFS-U3-63S4C 24057132, LEFT, host port P3 (chipset xHCI) |
| J2 / cam1 | BFS-U3-63S4C 24057131, RIGHT, host port P6 (Thunderbolt xHCI) |
| Camera firmware / SDK | 1801.0.9.0 / Spinnaker 4.2.0.88 |
| Camera config (both, identical) | `cameras.launch.py trigger:=true` at `4ee0c39`: BayerRG8 3072×2048, `ExposureAuto` Off, `ExposureTime` 5000 µs commanded (chunk reports 5004.0 µs, std 0, both cameras), gain 10 dB, `TriggerSource` Line0 `RisingEdge`, `TriggerOverlap` ReadOut, `TriggerDelay` 379 µs, `IspEnable` 0 |
| Trigger | 30 Hz, 100 µs pulse, MCPWM, loopback on capture channel 3 |
| Capture clock | 80 MHz, 12.500 ns/tick (read back from the hardware) |
| ESP-IDF | v5.5.5 |
| D1–D2 firmware | SELFTEST, app version `69cd649` (as flashed for Stages B–C) |
| D3 firmware | MICROROS, app version `4ee0c39` (clean tree), built 2026-10-28; `micro_ros_espidf_component` @ `16cf0d534e618c2752e5546710cd2c81c0a586cc` |
| Agent | `micro_ros_agent` @ `52abdf5a9897637b2539cb1c239e1ec4fa92e01c` (`oem_ros2_ws`), serial 115200 baud, via `camera_sync.launch.py` |
| Host middleware | CycloneDDS |
| Bag | 299.7 s MCAP, 2.8 MiB: `/arts_camera_sync/pulse` 8991, `/camera_left/meta` 8991, `/camera_right/meta` 8991 messages. Kept machine-local at `$ARTS_LOCAL/benchmark_bags/camera_sync_skew_v2pcb_20261028/` (mcap sha256 prefix `9278d70a2c44890b`) |

Skew sign convention: cam0 − cam1 = LEFT − RIGHT = 24057132 − 24057131. Negative means the
left camera begins exposing first.

## D1–D2 — SELFTEST console (box reset with both cameras at 30.00 Hz, cumulative since boot)

| Quantity | n | mean | min | max | spread |
|---|---|---|---|---|---|
| Trigger period | 1321 | 33 333.000 µs | 33 333.000 | 33 333.000 | 0.000 |
| cam0 (L) trigger→exposure | 1322 | 709.331 µs | 705.000 | 713.575 | 8.575 |
| cam0 (L) exposure width | 1322 | 4976.478 µs | 4976.312 | 4976.588 | 0.275 |
| cam1 (R) trigger→exposure | 1320 | 711.879 µs | 707.438 | 716.175 | 8.738 |
| cam1 (R) exposure width | 1320 | 4977.331 µs | 4977.175 | 4977.450 | 0.275 |
| \|cam0 − cam1\| | 1320 | 3.326 µs | 0.013 | 10.438 | 10.425 |

Pulses 1322, dropped records 0. Pulses with no strobe: cam0 0, cam1 2.
Systematic (difference of means, L − R): **−2.548 µs**.

## D3 — micro-ROS bag

### Trigger

| | |
|---|---|
| Pulses / index span / gaps | 8991 / 8991 / **0** |
| `time_synced` | true on every message |
| Period | 33 333.000 µs, std 0.000, spread **0.000 µs** |

### (a) Hardware — `ExposureActive` captures on one 80 MHz counter

| | cam0 / LEFT (24057132) | cam1 / RIGHT (24057131) |
|---|---|---|
| Trigger → exposure | 709.270 ± 2.260 µs | 711.978 ± 2.265 µs |
| Range | [704.900, 713.663] µs | [707.425, 716.362] µs |
| Exposure width | 4976.493 ± 0.044 µs | 4977.255 ± 0.046 µs |
| Pulses with no capture | 5 / 8991 | 11 / 8991 |

**Skew, LEFT − RIGHT, signed (ticks cast to signed before differencing), n = 8975:**

| | |
|---|---|
| Mean (systematic) | **−2.707 µs** |
| Std | **3.208 µs** |
| Range | [−11.075, +5.188] µs |
| Percentiles 0.5 / 5 / 50 / 95 / 99.5 | −9.850 / −8.079 / −2.713 / +2.650 / +4.475 µs |
| Worst \|skew\| | **11.075 µs** |

Autocorrelation of the skew at lags 1, 2, 4, 8, 16 pulses: 0.64, 0.34, −0.12, −0.46, +0.53.

### (b) Software cross-check — `camera_time` device clocks

| | |
|---|---|
| Paired frames | 8991 |
| Raw delta L − R | arbitrary epoch offset; spread 739.880 µs |
| Drift | +2.453 µs/s (the two camera crystals) |
| **Detrended residual std** | **3.230 µs** (hardware std 3.208 µs) |
| Per-pulse join (frame k−1 ↔ pulse k) | r = **0.9911**, n = 8974 |
| Per-pulse residual, detrended software − centred hardware | std **0.430 µs**, range [−1.210, +1.249] µs |
| Implied strobe rise delay (chunk `ExposureTime` − width) | L 27.507 µs, R 26.745 µs |

The software path cannot recover the systematic term (unrelated clock epochs); the signed
systematic comes from (a) only.

### Capture tick → ROS time fit

Crystal ratio error 7.584 ppm; residual std 139.2 µs (includes micro-ROS transport).

## Missed strobes

Frame gaps in each camera's `camera_time` sequence (interval > 1.5 periods): **left 0,
right 0**. Frame intervals were 33 324.6–33 334.3 µs on both cameras. So the cameras exposed on
every pulse of the recording, and all 16 missing captures are start edges that the box did not
register (delay and width both `NO_CAPTURE`, by the firmware's rule that an end edge without a
start is dropped). They are isolated single pulses, never on both cameras at once:

- left, `pulse_index` 2667, 4841, 7499, 9100, 9112
- right, `pulse_index` 1993, 4407, 4452, 5573, 5721, 5756, 6178, 6996, 7655, 8311, 8637

Effect on the baseline: estimating the skew at each missed pulse as the mean of its two
neighbours gives −2.856 µs on average (range −8.037 to +2.025 µs); including those estimates
moves the mean from −2.707 to −2.708 µs.

## Acceptance (Stage D)

| | Target | Result | |
|---|---|---|---|
| Skew, hardware | < 1 ms (expect < 100 µs) | mean −2.707 µs, worst \|11.075\| µs | PASS |
| Trigger period spread | ≤ 1 tick | 0.000 µs | PASS |
| Missed strobes | 0, excluding pulses where the camera did not expose | 16 of 8991, all with the frame present | **FAIL** |
| Software cross-check | `camera_time` residual << 1 ms | 3.230 µs detrended; 0.430 µs per pulse | PASS |
| Systematic | report; expect ≤ 2.81 µs | −2.707 µs (bag), −2.548 µs (console) | PASS |

## Flash, install and launch record

1. `vcs import components < components.repos` (README recipe, step 1) failed:
   `vcs import: error: argument path: Path 'components' does not exist.` The directory is
   gitignored and absent from a fresh checkout. `mkdir -p components` and re-running fetched
   the pin.
2. The README's step 2, `ln -s ../../../../../arts_interfaces
   components/micro_ros_espidf_component/extra_packages/`, made a dangling link (it resolves to
   `arts_embedded/arts_interfaces`). Six `../` resolve to `arts_ros2_ws/src/arts_interfaces`.
3. `rm -f sdkconfig && idf.py -DIDF_TARGET=esp32
   -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.microros" reconfigure build`,
   in an `env -i` shell with only `esp_idf_env.sh` sourced: succeeded on the FIRST invocation
   (1011 steps); no second pass was needed. `sdkconfig` checked before flashing:
   `CONFIG_ARTS_SYNC_MODE_MICROROS=y`, `CONFIG_ESP_CONSOLE_NONE=y`, UART pins 1/3.
4. `idf.py -p /dev/arts_camera_sync flash`: three regions written, all hashes verified.
5. Agent: session established on first start; publisher created.
6. Camera driver, first launch of the session: `camera_left` (24057132) logged
   `ImageFormatControl/PixelFormat failed: node ... not writable` and the same for
   `IspEnable` (`set to: 1 instead of: 0`), immediately after the `TriggerMode Off` disarm
   was written; it nevertheless ran at 30.00 Hz. `camera_right` configured without warnings.
   The driver was stopped with SIGINT (clean shutdown logged) and relaunched; on the second
   launch both cameras configured with no write failures. All numbers above are from the second
   launch. The cameras had stayed powered since the previous session.
