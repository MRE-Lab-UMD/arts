# Footswitch record toggle, functional test, 2026-10-28

> **Role: measured numbers (layer 4).** One test session. Immutable once committed.
> Design: `docs/modules/console_display.md` ("Recording by pedal"); node
> `arts_teleop/scripts/footswitch.py`, mapping `arts_teleop/config/input_mapping.yaml`.

**Result.** The right pedal paused and resumed the composed image recorder twice: two
contiguous segments, 492 and 428 sync pulses with as many frames per camera and no gap inside
either, nothing recorded while paused, and four episode marks (START/END × 2). A 4.8 s hold
of the pedal toggled once.

## Setup

| | |
|---|---|
| Host | production workstation, kernel 6.8.1-1062-realtime |
| Pedal | Kinesis Savant Elite2 (`29ea:0100`), full speed, directly on rear port P2 (bus-port 3-5); factory keys A/B/C = left/middle/right; udev rule from `deploy/modules/footswitch.sh` |
| Mapping | left `clutch`, middle `camera` (both not implemented: logged only), right `record_toggle` |
| Cameras | triggered path defaults (30 Hz, 1536×1024 binned), sync box firmware `434879c` |
| Command | `ros2 launch arts_driver cameras.launch.py trigger:=true image_bag:=<dir>/bag image_bag_paused:=true footswitch:=true image_bag_extra_topics:=/arts_camera_sync/pulse`, ended by SIGINT |
| Bag | 2.7 GiB MCAP, machine-local at `$ARTS_LOCAL/benchmark_bags/footswitch_record_toggle_20261028/` with the launch log |
| Code | the commit that adds this report |

Operator sequence: right tap (start), ~16 s, right tap (stop), left tap, middle tap, right
held 4.8 s (start), ~9 s after release, right tap (stop).

## Results

| | Episode 1 | Episode 2 |
|---|---|---|
| Pulses recorded (index span) | 492 (492) | 428 (428) |
| Frames, left / right | 920 / 920 in total, equal to the pulses | |
| Press → first recorded pulse | 15 ms | 21 ms |
| Stop press → last recorded pulse | 185 ms | 176 ms |
| START / END mark, `header.stamp` = kernel time of the press | ✓ / ✓ | ✓ / ✓ |

- The only image gap in the bag is the 23.27 s pause between the episodes (both cameras).
- Mark delivery: kernel event stamp → recorder receive 7–11 ms.
- Stop overshoot of ~0.2 s is by design: END is published, then the recorder is paused after
  a 0.2 s settle so the END mark is written. Dataset segmentation uses the marks, not the
  first and last frames.
- `/footswitch/joy`: 4 messages recorded; the presses that STARTED each episode arrived while
  the recorder was still paused and are not in the bag (their time is in the START mark).
- Left and middle taps logged "not implemented yet"; no recorder change.
- Auto-repeat (first repeat after 250 ms, then every 34 ms, measured in the same session)
  was dropped: the hold produced one press and one release.

## Not measured

Pedal switch-to-kernel latency (USB HID polling) is not in these numbers; the stamps start
at the kernel event.
