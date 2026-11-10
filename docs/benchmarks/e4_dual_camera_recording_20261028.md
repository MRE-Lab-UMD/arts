# E4 — dual-camera recording, 30 min, 2026-10-28

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E4 (`publication_plan.md` in the manuscript repo). Layout and reasoning:
> `docs/modules/console_display.md`; the 10-minute run of the same layout:
> `camera_composed_record_display_20261028.md`.

**Result.** 30.5 min, both cameras at 1536×1024 raw Bayer, 30 Hz, recorded with the surgeon
display running: **54 827 frames per camera for 54 827 sync pulses**, 0 pulse-index gaps,
0 image stamp gaps, 0 meta frame gaps, i.e. 100.000 % of nominal on every topic. Stereo
skew (24057131 − 24057132, left − right) **+2.687 ± 3.204 µs**, worst |skew| 11.050 µs.

## Setup

| | |
|---|---|
| Host | production workstation, Intel Core Ultra 9 285K (20 CPUs), kernel 6.8.1-1062-realtime, one 1 TB NVMe (system disk; 377.6 GB free at start) |
| Image disk | **the system disk**: this host has no dedicated image NVMe (decision 5) and no free M.2 slot |
| Display | `stereo_display` on one 1920×1080 monitor, HMD and tower views as two non-fullscreen windows. No GOOVIS headset connected |
| Cameras | 2× BFS-U3-63S4C, firmware 1801.0.9.0, Spinnaker 4.2.0.88. Left 24057131 on bus-port 2-1 (CPU Thunderbolt xHCI `0000:00:0d.0`, port P6), right 24057132 on 4-4 (chipset xHCI `0000:80:14.0`, P3), both 5000 Mb/s. Full `lsusb -t` in the bag directory (`host_usb_topology.txt`) |
| Camera config | triggered path defaults: 30 Hz from the sync box, 2×2 averaging binning → 1536×1024 BayerRG8, exposure 5000 µs, gain 10 dB, colour preset, factory set loaded into RAM after `AcquisitionStop` |
| Sync box | PCB v2, MICROROS firmware `7859c33` (the reconnect fix `434879c` was flashed after this run), Jazzy micro-ROS agent in its own process |
| Process layout | one `component_container`: `camera_left`, `camera_right`, `image_recorder` (`rosbag2_transport::Recorder`, MCAP, cache 256 MiB), `stereo_display`, intra-process comms on; `disk_guard` (reserve 30 GB) in its own process |
| Code | ARTS `bcde307` for the run; analyzer as in the commit that adds this report (local detrend, below) |
| Command | `run_e4.sh` in the bag directory: `ros2 launch arts_driver cameras.launch.py trigger:=true image_bag:=<dir>/bag image_bag_extra_topics:=/arts_camera_sync/pulse display:=true`, SIGINT after 1830 s |
| Bag | 1827.5 s, 172.6 GB MCAP, start 16:38:02, end 17:08:33 (local). Machine-local at `$ARTS_LOCAL/benchmark_bags/e4_dual_camera_20261028/` (mcap sha256 prefix `2a401459b7d0c165`), with the launch log, analyzer output and JSON, image scan, display statistics and the join check |

`/arts_camera_sync/pulse` was recorded in the image bag for this analysis; in the designed
layout it belongs to the state bag.

## Recording

| Topic | Messages |
|---|---|
| `/arts_camera_sync/pulse` | 54 827 (index span 54 827, 0 gaps) |
| `/camera_left/image_raw`, `camera_info`, `meta` | 54 827 each |
| `/camera_right/image_raw`, `camera_info`, `meta` | 54 827 each |

- Image stamp gaps (> 1.5 × median interval): 0 and 0; median rate 30.000 Hz both.
- Meta frame gaps from `camera_time`: 0 and 0.
- Data rate 94.4 MB/s. Free space at the end 227.1 GB (last `disk_guard` report); the guard
  never acted.

## Synchronization (`analyze_sync.py --cam0-meta /camera_right/meta`)

cam0 = 24057132 (right, J1), cam1 = 24057131 (left, J2).

| | n | mean | std | min | max | µs |
|---|---|---|---|---|---|---|
| trigger period | 54 827 | 33 333.000 | 0.000 | 33 333.000 | 33 333.000 | |
| cam0 trigger→exposure | 54 826 | 708.976 | 2.265 | 704.500 | 713.550 | |
| cam1 trigger→exposure | 54 827 | 711.663 | 2.267 | 706.987 | 716.225 | |
| skew cam0 − cam1 (right − left) | 54 826 | −2.687 | 3.204 | −11.050 | 5.625 | |

- Missing strobe captures: cam0 1 (pulse 0 of the bag, index 1144, the recorder's first
  pulse), cam1 0.
- Exposure width: cam0 4976.689 ± 0.071 µs. cam1 4977.940 ± 114.654 µs, the std from ONE
  event: pulses 15 996 and 15 997 of the bag measured 31 767.2 µs and 6721.4 µs (neighbours
  4977.4 µs). The start edges of those pulses were normal (skew within the range above), so
  only exposure-end captures were affected. Cause not established.
- Tick → ROS time fit: crystal ratio error 7.579 ppm, residual std 227.7 µs.
- `camera_time` cross-check: 54 827 paired frames; delta drift 2.552 µs/s. Detrended by a
  single line, residual std 4.418 µs and per-pulse join r = 0.7253: the camera crystals'
  ratio wandered over 30 min (linear-fit residual, per-minute means −3.85 to +5.29 µs).
  Detrended by a 301-frame (~10 s) moving average: residual std **3.223 µs** (hardware
  3.204), per-pulse join lag −1 frame, **r = 0.9879**, residual (software − hardware)
  0.500 µs std. The analyzer now uses the local detrend for the join.

## Display (`stereo_display`, 364 periods of 5 s)

| | |
|---|---|
| Pairs shown | 29.998 /s mean; lowest period 29.22 /s |
| Pairs skipped | 4, all in one period (16:59:05–16:59:10, ~21 min in) |
| Frame age at hand-off | 19.9 ms mean; worst 191.5 ms (that period), 62.8 ms (first period), 40.6 ms (one more period); all others ≤ 40 ms |
| Compose + show per pair | 18.8 ms mean; worst 190.4 ms (same period) |

The 191 ms stall coincided with no operator-side command from the analysis session; its
cause is not established. The recorder lost nothing during it.

## Images

Frames whose two green Bayer sites disagree by more than 6 % of the median ratio (the open
one-row readout shift): left 554 of 54 827 (1.01 %), right 564 of 54 827 (1.03 %).

## Not measured

CPU load, RSS and disk write latency were not recorded.
