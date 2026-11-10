# E7 — glass-to-glass video latency, 5 min, 2026-10-28

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Paper experiment E7 (`publication_plan.md` in the manuscript repo). Method and display
> pipeline: `docs/modules/console_display.md`; tools `arts_display/bench/latency_clock.py`
> and `analyze_latency_clock.py`.

**Result.** Glass-to-glass latency of the surgeon display path, camera to tower view:
**67.8 ms** (from the 60/80 ms split of the 91.7 % of frames that read 3 or 4 units), against
the ≤ 100 ms target. Mean over every decoded frame 70.8 ± 0.3 ms (sem), which includes 2.1 %
physically impossible misreads; mean over the 97.9 % of frames that read 40–160 ms, 69.1 ms.
Measured on a 60 Hz monitor in a 320×180 window; the GOOVIS headset was not measured.

## Method (closed loop, no external camera)

The laparoscope looks at screen 2. On it, side by side: a clock window and the tower view of
the surgeon display. The clock shows k = floor(t / 20 ms) mod 16 as a 4-bit Gray code in four
squares; the tower view shows the left camera's image of the clock. Each camera frame
therefore contains two readings: the clock on the glass now, and the clock as it was on the
glass one trip earlier (camera → USB → driver → display node → window system → panel).
Latency per frame = (k_live − k_copy mod 16) × 20 ms. The camera's own exposure is common to
both readings and cancels.

- **Quantization.** One frame reads in whole 20 ms units. The clock and the cameras are not
  synchronized, so for a latency L between 60 and 80 ms the frames read 3 or 4 units with
  P(4) = (L − 60) / 20; L = 60 + 20 × 2842 / (4465 + 2842) = 67.8 ms.
- **Decoding** (`analyze_latency_clock.py`, nothing placed by hand): the live squares are
  the most-changing pixels; their bit order comes from toggle counts and, for the top two
  bits (equal toggle rates), from agreement with the frame stamps. Each copy square is found
  as the pixels best matching one live bit at a delay of 1–8 frames, then read from its own
  series.
- **Exclusions.** A frame is excluded when the live code is more than one unit from the
  stamp prediction, or the copy code steps from either neighbour by more than 4 units. They
  were defined before this run, on the 45 s trial.
- **Rolling shutter.** Live and copy squares 17 image rows apart. Readout cannot exceed the
  33.3 ms frame period over 1024 rows, so the bias is < 0.6 ms.

## Setup

| | |
|---|---|
| Host | production workstation, Intel Core Ultra 9 285K, RTX 5080, kernel 6.8.1-1062-realtime, GNOME on X11 |
| Screen 2 | basic 24.5-inch 1920×1080 monitor on `DP-4`, **60 Hz**; the clock and the tower view both on it |
| Camera | left, BFS-U3-63S4C 24057131, triggered at 30 Hz by the sync box (firmware `434879c`), 1536×1024 binned BayerRG8, exposure 5000 µs, **gain 40 dB** (normal 10 dB; the scope sees the screen dimly, and the copy is an image of an image). Gain does not change timing |
| Display | `stereo_display` composed with both cameras and the image recorder (recording) in one process; tower view only (`display_hmd:=false`), windowed **320×180** at 2790,443 (not fullscreen) |
| Clock | `latency_clock.py --bits 4 --square 80 --gap 10 --unit-ms 20`, window 190×190 at 2565,535 |
| Command | `ros2 launch arts_driver cameras.launch.py trigger:=true display:=true display_hmd:=false display_tower_pos:=2790,443 display_window_size:=320,180 gain:=40.0 image_bag:=<dir>/bag image_bag_extra_topics:=/arts_camera_sync/pulse`, SIGINT after 305 s |
| Bag | 302.6 s, 26.6 GiB MCAP; 9079 pulses, 9078 left / 9079 right images. Machine-local at `$ARTS_LOCAL/benchmark_bags/e7_glass_to_glass_20261028/` with the launch log, both run scripts, the analyzer output and JSON |
| Code | the commit that adds this report |

## Results

| Reading (units of 20 ms) | 0 | 1 | 2 | **3** | **4** | 5 | 6 | 7 | 8 | 9–15 |
|---|---|---|---|---|---|---|---|---|---|---|
| Frames | 32 | 40 | 145 | **4465** | **2842** | 250 | 59 | 25 | 18 | 96 |

- Decoded 7972 of 9078 frames (1106 excluded). Live code equal to the stamp prediction in
  93.6 % of frames (bit errors 0.8–3.2 %); copy 74.1 % (bit errors 3.4–13.7 %).
- 0–1 and 9–15 units (≤ 20 ms or ≥ 180 ms, 168 frames, 2.1 %) are misreads: no path is
  faster than one exposure plus readout, and 9–15 are negative latencies modulo 16.
- 100 ms: 3.1 % of frames; 120 ms 0.7 %; 140–160 ms 0.5 %. Whether these are slow frames or
  single-bit misreads of 80 ms is not separable with this code (a misread copy bit 0 moves a
  reading by one unit).
- Display during the run: 30.00 pairs/s, 0 pairs skipped in 60 periods of 5 s.

## Not measured / limitations

- The GOOVIS headset (HDMI, 1920×1080 @ 60 Hz, fullscreen). Same process and frames; it
  differs in output port, panel and fullscreen compositing.
- Fullscreen. The run used windows under the GNOME compositor; fullscreen may be unredirected
  and so differ by up to one refresh (16.7 ms) either way.
- The real tower monitor (27-inch, 2560×1440).
- A per-frame latency distribution finer than 20 ms.
