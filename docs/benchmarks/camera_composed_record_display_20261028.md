# Cameras + image recorder + surgeon display in one process, 10 min, 2026-10-28

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Design and interpretation: `docs/modules/console_display.md`. Not paper experiment E4
> (that is 30 min with the USB controller topology recorded); this is the 10-minute run
> that decides whether the composed layout can pass it.

**Result.** With both cameras, the rosbag2 recorder and the display composed into one process,
the recorder lost nothing: 18 529 pulses, 18 529 left and 18 528 right images, 0 image stamp
gaps, 0 meta frame gaps. The display showed 29.99 pairs/s, frame age at hand-off 17.8 ms mean.

## Setup

| | |
|---|---|
| Host | production workstation, Intel Core Ultra 9 285K (20 CPUs), kernel 6.8.1-1062-realtime |
| Display | one 1920×1080 monitor (DP-0); HMD and tower outputs as two overlapping, non-fullscreen windows on it. No GOOVIS headset connected |
| Cameras | 2× BFS-U3-63S4C, firmware 1801.0.9.0, Spinnaker 4.2.0.88; left 24057131, right 24057132 (right on sync-box J1 = cam0) |
| Camera config | triggered path of `cameras.launch.py`, defaults: 30 Hz from the sync box, 2×2 averaging binning → 1536×1024 BayerRG8, exposure 5000 µs, gain 10 dB, colour preset, factory set loaded into RAM after `AcquisitionStop` |
| Sync box | PCB v2, MICROROS firmware `7859c33`, Jazzy micro-ROS agent (separate process) |
| Process layout | one `component_container` (single-threaded): `camera_left`, `camera_right`, `image_recorder` (`rosbag2_transport::Recorder`, MCAP, cache 256 MiB), `stereo_display` (`arts_display::StereoDisplay`, OpenCV 4.6.0 on the CPU), all with `use_intra_process_comms: true` |
| Code | the commit that adds this report |
| Command | `ros2 launch arts_driver cameras.launch.py trigger:=true image_bag:=<path> image_bag_extra_topics:=/arts_camera_sync/pulse display:=true display_overlay:=true`, ended by SIGINT after 620 s |
| Bag | 617.6 s, 58.3 GB MCAP. Machine-local at `$ARTS_LOCAL/benchmark_bags/camera_composed_record_display_20261028/` (mcap sha256 prefix `87368aedab925554`), with the launch log, display statistics and the image scan script |

`/arts_camera_sync/pulse` was recorded in the image bag for this analysis only; in the
designed layout it belongs to the state bag.

## Recorder

| Topic | Messages |
|---|---|
| `/arts_camera_sync/pulse` | 18 529 |
| `/camera_left/image_raw`, `camera_info`, `meta` | 18 529 each |
| `/camera_right/image_raw`, `camera_info`, `meta` | 18 528 each |

- Image stamp gaps (> 1.5 × median interval): left 0, right 0; median rate 30.000 Hz both.
- Meta frame gaps from `camera_time`: left 0, right 0.
- Missing strobe captures: cam0 (right) 1, cam1 (left) 0. The one is the first pulse in the
  bag (pulse index 490 591 in the 60 s check of the same layout); the right camera has one
  fewer image, so it is the pulse that arrived while that camera was being armed.

## Skew (analyzer `analyze_sync.py --cam0-meta /camera_right/meta`)

| | n | mean | std | min | max | µs |
|---|---|---|---|---|---|---|
| cam0 (24057132) trigger→exposure | 18 528 | 709.153 | 2.261 | 704.725 | 713.575 | |
| cam1 (24057131) trigger→exposure | 18 529 | 711.936 | 2.263 | 707.413 | 716.375 | |
| skew cam0 − cam1 (24057132 − 24057131) | 18 528 | −2.782 | 3.196 | −11.113 | 5.575 | |

Worst |skew| 11.113 µs. Per-pulse `camera_time` join: lag −1 frame, r = 0.9785.

## Display (`stereo_display` statistics, 123 periods of 5 s)

| | |
|---|---|
| Pairs shown | 29.99 /s mean; lowest period 28.93 /s |
| Pairs skipped | 8 in total, all in the first 15 s (7 in the first period, 1 in the third) |
| Frame age at hand-off to the window system | 17.8 ms mean over all periods; worst 280.1 ms (first period), 85.2 ms (third); every later period ≤ 30.4 ms |
| Compose + show per pair | 16.6 ms mean; worst 279.8 ms (first period) |

Frame age is wall clock after `imshow` minus the image `header.stamp`. It excludes the window
system, the display link and the panel.

## Images

Scan of every image (`shift_stream.py`): frames whose two green Bayer sites disagree by more
than 6 % of the median ratio (the one-row readout shift): left 175 of 18 529 (0.94 %), right
193 of 18 528 (1.04 %).

## Not measured

CPU load, RSS, disk throughput and the USB controller topology were not recorded.
