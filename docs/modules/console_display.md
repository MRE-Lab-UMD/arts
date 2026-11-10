# Console display (GOOVIS headset + support-tower monitor)

> **Doc layer 2 of 4 — module overview.** What the surgeon sees, at what resolution, and how
> the display shares the camera stream with the recorder without disturbing it. Status
> belongs in AGENTS.md (Phase 4, "GOOVIS stereo display path"). This file does not own an
> independent status list.
>
> | | Where | For |
> |---|---|---|
> | Decisions + status | AGENTS.md Phase 4; decisions 5, 12, 15 | why, and what is done |
> | **You are here** | this file | output formats, pipeline, the recorder rule |
> | Cameras | [`pilot_flir_camera.md`](pilot_flir_camera.md) | sensor, roles, colour preset, view rotation |

## Outputs (specified 2026-10-28)

The source is each camera's recorded image, 1536×1024 (3:2) BayerRG8, made in the camera by
2×2 binning of the 3072×2048 sensor (decision 5 amendment, 2026-10-28). The display is for
the surgeon only and carries no data (decision 15), so it is free to scale: 1536→1620 is a
5.5 % upscale.

| Output | Signal | Content |
|---|---|---|
| GOOVIS headset | 1920×1080, half-width side-by-side 3D mode | left camera in the left 960×1080 half, right camera in the right half; each image 810×1080 centred in its half (75 px black each side) |
| Support-tower monitor | 1920×1080 | left camera only, 1620×1080, centred (150 px black each side) |
| Recording | — | NOT the display stream: both cameras, 1536×1024 binned raw Bayer, unrotated (decisions 5 and 12) |

Why 810×1080. A 3:2 image at the panel's full 1080 height is 1620×1080. In half-width
side-by-side mode the headset stretches each 960-wide half to the full 1920 width, i.e. ×2
horizontally, so each eye's image is stored squeezed to half its width, 810×1080 (3:4 as
stored), and appears 1620×1080 (3:2) in the eye. Verify the ×2 stretch on the actual headset
once: display a circle test pattern through this path; it must look round, not oval.

Processing order per frame: debayer (with the camera's white balance and gamma already
applied in-camera) → 180° rotation (display only; see "Roles and view rotation" in the camera
doc, never ReverseX/ReverseY) → scale to 1620×1080 → for the headset, squeeze to 810×1080 and
place both eyes into one 1920×1080 frame. Left/right placement follows the camera ROLES
(24057131 left, 24057132 right), not topic order; a swapped placement inverts depth.

## Overlays (2026-11-06)

All drawn on the full-size views before the HMD squeeze, at the same place in both eyes (zero
disparity, read at screen depth), and on the tower (the left view):
- **Recording state**, top left (`status_overlay`).
- **Per PSM** (`align_overlay`): `psm_left  OPEN` (or `MIS` with the RCM on) at 1/4 of the
  width, `psm_right` at 3/4; the teleop status (`ENGAGED`, `align N deg`,
  `NO STYLUS (VerseGrip off?)`) on the line below. Each line is centred, kept between the
  screen edge and the pedal panel, and shrunk if it would not fit (a single long line used to
  run off the screen or into the middle). The two triads while a PSM waits at the gate.
- **Pedal panel** (`pedal_overlay`), bottom centre, laid out as the pedals sit at the console
  (PI, 2026-11-06): upper row, step layer 2 (platform ~10 cm up): the monopolar pedal's CUT
  (yellow) and COAG (blue), then the bipolar pedal (blue) (`energy_pedals` order, confirmed by
  the PI); lower row, step layer 1 (floor): the Savant's CLUTCH, CAMERA, REC
  (`pedal_labels`). A pad fills while its pedal is pressed: the Savant's from
  `/console/footswitch_teleop`, the energy pads from `/esu/state`, i.e. the generator-side
  contacts the ESU-TAP board drives (a software activation lights them too). The energy row
  is grey with "ESU off" when no `/esu/state` arrived for 3 s. All six pads checked by the PI
  at the console with the ESU generator off. The energy row models the ASG-300 as measured
  2026-11-06 ([`esu.md`](esu.md) Results): only the FIRST pedal pressed is filled (delivered);
  a second closed meanwhile is drawn dark with a thick outline (pressed, ignored); releasing
  the first while the second is held shows "F6: release all energy pedals" in red until all
  are up. Confirmed on screen by the PI with the generator on (10 W, no instrument).

## Pipeline: one process, the recorder first

A full-resolution image is 6.3 MB (1.6 MB binned). The 2026-10-28 test viewer — a separate Python process
subscribing to it with reliable QoS — cost the recorder ~120 meta frames per camera per
minute (0 without it). The same holds for the recorder itself: a separate `ros2 bag record`
process taking both binned image topics plus meta and pulses lost 125 and 111 meta frames in
60 s, against 0 when the images were left out (2026-10-28). So neither the display nor the
image recorder is a separate subscriber across processes:

- the two camera nodes, the image recorder (`rosbag2_transport::Recorder`, a component in
  Jazzy) and the display node are composed into ONE container with
  `use_intra_process_comms: true`. The FLIR driver publishes `std::move(img)`
  (`camera.cpp:846`), so each frame reaches both consumers without a copy;
- the display node (`arts_display::StereoDisplay`) debayers, rotates, scales and packs on
  the CPU with OpenCV. The GPU was planned for this, but at 1536×1024 the CPU takes 16.6 ms
  per pair (measured below), within the 33 ms frame period. Subscription callbacks only
  store the newest frame pointer; one render thread does all drawing, so the executor the
  recorder shares is never held up;
- the display runs from bringup. The recorder starts paused and the record pedal resumes and
  pauses it (rosbag2 pause/resume services), so the surgeon watches first and records when
  ready, without the display restarting;
- any extra viewer (laptop, debugging) is a separate low-priority process taking a reduced
  image with best-effort QoS, depth 1: it may drop frames, it must never slow the cameras.

Unverified until measured: that the composed recorder plus display loses nothing (0 meta
gaps, 0 image drops over 10 min), and the frame age at the screen. Glass-to-glass latency
through this path is paper experiment E7 (target ≤ 100 ms).

## Launch

```
ros2 launch arts_driver cameras.launch.py trigger:=true \
  image_bag:=<path> [image_bag_paused:=true] display:=true [display_fullscreen:=true] \
  [display_hmd_pos:=X,Y display_tower_pos:=X,Y]
```

The full console session on the workstation (cameras, both outputs fullscreen, recorder
paused until the record pedal, disk guard):

```
ros2 launch arts_driver cameras.launch.py trigger:=true display:=true display_fullscreen:=true \
  display_hmd_pos:=1920,0 display_tower_pos:=0,0 \
  image_bag:=<path> image_bag_paused:=true footswitch_teleop:=true
```

Before it: the micro-ROS agent for the sync box is running, the GOOVIS is at 1920×1080, and
`disk_guard.py --check-only --duration-s <planned>` passes. After it: export the bag
(`arts_record/scripts/export_bag.sh`) before the next long session.

On the console, each window goes fullscreen on its own output at the desktop offset that
`xrandr --listmonitors` prints.

**Workstation layout since 2026-10-31** (recabled; all four outputs at 1920×1080 @ 60 Hz).
If `xrandr --query | grep " connected"` differs from this, the window positions are wrong:

| Output | Desktop offset | Screen |
|---|---|---|
| `HDMI-0` | 0,0 | tower monitor (top of the support tower) → `display_tower_pos:=0,0` |
| `DP-5` | 1920,0 | GOOVIS G3 (EDID name `GOOVIS_G3`, reports 192×108 mm) → `display_hmd_pos:=1920,0` |
| `DP-0` | 3840,0 | supervisor monitor 1, primary |
| `DP-2` | 5760,0 | supervisor monitor 2 |

The tower is now 1920×1080 (it was a 2560×1440 output, `DP-4`, when E7 and the console check
below were measured), so the tower view fills it without scaling.

The GOOVIS G3 (named in its EDID) prefers 2560×1440 and must
be set to **1920×1080 @ 60 Hz** for half-width side-by-side; set it in Settings → Displays so
it persists (an `xrandr --mode` change does not survive a re-login). Previous layout,
2026-10-28: original monitor `DP-0` at 0,0; tower `DP-4` (2560×1440) at 1920,0; GOOVIS
`HDMI-0` at 4480,0, i.e. `display_hmd_pos:=4480,0 display_tower_pos:=1920,0`. On a
2560×1440 tower the 1920×1080 view is scaled uniformly (image 2160×1440, 3:2 kept).

Recording by pedal: add `image_bag_paused:=true footswitch_teleop:=true`. The recorder starts
paused; the record pedal (right, `arts_teleop/config/input_mapping.yaml`) resumes it and
publishes an `/episode_mark` START stamped at the press, and on the next press publishes END
and pauses it 0.2 s later so the mark is written. Marks and pedal state go into the image bag
until the state bag exists. The footswitch node is its own process: it touches no images.

`image_bag` and `display` need `composed:=true` (the default). The display needs `DISPLAY` in
the environment; `env -i` launches must pass it through. Window positions are parameters
(`hmd_x`/`hmd_y`, `tower_x`/`tower_y`).

## Results (2026-10-28, production workstation, one monitor)

10 minutes, all four in one process
([`camera_composed_record_display_20261028.md`](../benchmarks/camera_composed_record_display_20261028.md)):
recorder lost nothing (18 529 pulses; 18 529 / 18 528 images; 0 image or meta gaps); display
29.99 pairs/s, frame age at hand-off 17.8 ms mean, ≤ 30.4 ms after the first 15 s of window
start-up. Skew unchanged (24057132 − 24057131 −2.782 ± 3.196 µs).

30 minutes, the same layout (E4,
[`e4_dual_camera_recording_20261028.md`](../benchmarks/e4_dual_camera_recording_20261028.md),
the result to quote): 54 827 frames per camera for 54 827 pulses, 0 gaps; display 29.998
pairs/s, frame age 19.9 ms mean; one unexplained 191 ms display stall ~21 min in.

Record pedal
([`footswitch_record_toggle_20261028.md`](../benchmarks/footswitch_record_toggle_20261028.md)):
two pedal-started episodes, each contiguous (492 and 428 pulses, frames equal), nothing
recorded while paused, START/END marks for both; press → first recorded frame 15–21 ms; a
4.8 s hold toggled once.

Console outputs (2026-10-28, GOOVIS G3 on `HDMI-0` at 1920×1080 @ 60 Hz, tower on `DP-4`,
both fullscreen): 30.00 pairs/s, frame age 19.7 ms mean. Operator check: the 3D view fuses,
a round object looks round in the headset (the ×2 stretch is right), the eyes are not swapped
(left eye matches the tower's left-camera view; an approaching hand comes nearer), and the
tower view is upright with side bars. GNOME ignored the placement of a window created before
it had content and drops fullscreen when a fullscreen window is moved; the node therefore
places both windows again, leaving fullscreen first, after the first frame.

Glass-to-glass latency, E7
([`e7_glass_to_glass_20261028.md`](../benchmarks/e7_glass_to_glass_20261028.md)): **67.8 ms**
on a 60 Hz monitor (tower view windowed, recorder recording), 5 min, 0 display skips. Method:
a closed loop on one screen, no external camera. A 4-bit Gray-code clock
(`arts_display/bench/latency_clock.py`) and the tower view sit side by side under the scope;
each camera frame reads the clock now and the clock one trip earlier, and
`analyze_latency_clock.py` decodes every frame of the bag. Practical notes from setting it up:
the scope sees about 640×430 px of the screen at ~10 cm, so both windows must be small; the
copy is an image of an image, so the camera needs gain (40 dB) and the code needs few, large
squares (8 bits in 2×4 were misread 20–32 % per copy bit, 4 bits in 2×2 work); the clock must
sit at the rows where its own copy appears (the view is rotated 180°, so moving the clock down
moves the copy up).

Still open before the checkbox: latency through the GOOVIS itself (HDMI, fullscreen), which
this method cannot see inside the headset. A 280 ms frame at window start-up
is expected and harmless; one mid-session would not be.
