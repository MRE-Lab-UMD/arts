# Camera sync skew — breadboard, micro-ROS bag (preliminary)

> **Doc layer 4 of 4 — one measurement session.** Figures plus the exact rig, pins and
> firmware they were taken on. Immutable once committed: a later session gets a new file
> beside this one, it does not overwrite it. Decisions live in AGENTS.md, procedure in
> [`../modules/camera_sync_wiring.md`](../modules/camera_sync_wiring.md), module context in
> [`../modules/pilot_flir_camera.md`](../modules/pilot_flir_camera.md).

**Date:** 2026-08-19
**Status:** PRELIMINARY. Breadboard rig, development laptop, single `spinnaker_camera_driver`
per camera. Decision 14 forbids treating this as the committed baseline; retake on the PCB
and the production workstation.

**Hardware:** 2× BFS-U3-63S4C (24057131 left / cam0, 24057132 right / cam1), ESP32-D0WD-V3
rev 3.1, MICROROS firmware 266 KB, `tick_period_ns` = 12.5 (80 MHz capture clock, measured).
Trigger 30 Hz, `ExposureTime` 5004 µs, `trigger_delay` 379 µs, strobe pull-up 1 kΩ.

**Bag:** 34.54 s MCAP, pulse + both meta, no images (350 KiB). 1004 pulses, 1037 meta/side.

## Hardware (ESP32 MCPWM capture)

| Metric | Value |
|---|---|
| \|cam0 − cam1\| mean | 3.52 µs |
| \|cam0 − cam1\| max | 10.44 µs |
| \|cam0 − cam1\| min | 0.013 µs |
| signed (cam0 − cam1) mean (systematic) | 2.81 µs |
| signed spread | 15.34 µs (−4.90 to +10.44) |
| cam0 trigger→exposure | 712.15 µs (spread 8.66 µs) |
| cam1 trigger→exposure | 709.33 µs (spread 8.64 µs) |
| cam0 exposure width | 4977.00 µs (spread 0.24 µs) |
| cam1 exposure width | 4976.32 µs (spread 0.23 µs) |
| trigger period | 33333.000 µs, spread 0.000 µs |
| missed strobes | cam0 3 / 1004; cam1 0 |

Agrees with the 2026-08-18 SELFTEST console (mean 3.4 µs, max 10.6 µs) on a different
sample. Width difference 0.68 µs accounts for part of the 2.81 µs systematic; the rest is
upstream of the strobe (560 Ω tolerance, cable, camera-internal). The ±8.6 µs per-camera
spread matches the IMX178 line period.

Cast `exposure_delay_ticks` to a signed integer before subtracting. A `uint32` wrap reports
53.687 s (`2^32` ticks at 12.5 ns) and looks like a destroyed bag.

## Software (`camera_time` from `~/<cam>/meta`)

`ImageMetaData` has no `FrameID`, so frames were paired by nearest `header.stamp` (all 1037
pairs fell inside 5 ms). The two device clocks differ by a 1.56 s constant; that offset is
not skew.

| Metric | Value |
|---|---|
| left `camera_time` period | 33332.78 µs (spread 9.58 µs) |
| right `camera_time` period | 33332.85 µs (spread 9.46 µs) |
| (L − R) residual std after removing clock offset | 22.8 µs |
| (L − R) residual peak-to-peak | 86.9 µs |
| paired \|header.stamp L − R\| mean | 40.7 µs (max 117.5 µs) |

Larger than the hardware figure, as it must be: independent crystals, no `FrameID` join,
and `header.stamp` pairing. Both remain far below the 1 ms sync target. The hardware
number is the one the paper quotes.

## Runtime notes

- `/arts_camera_sync/pulse` is BEST_EFFORT. `ros2 topic echo` is reliable by default and
  prints nothing; pass `--qos-reliability best_effort`. `ros2 bag record` adapts.
- `arts_ros2_ws` must be built against `flir_ros2_ws` or rosbag2 records pulse and silently
  drops the meta topics (type shows as invalid).
- Agent transitive pins this run: Micro-XRCE-DDS-Agent `7362281` (v2.4.3), spdlog
  `eb322062` (1.9.2).
