# camera_sync v2 PCB — Stage B, module installed, no cameras, 2026-10-27

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Procedure: `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/design_package/04_test_procedure.md`, Stage B.
> Same board as `camera_sync_v2pcb_stageA_20261027.md` (Stage A PASS).

**Result: Stage B PASS (B1–B4).**

| | |
|---|---|
| Module | ESP32-DevKitC-32E; chip ESP32-D0WD-V3 rev 3.1, 40 MHz crystal |
| Firmware | `arts_camera_sync`, app version `69cd649`, ESP-IDF v5.5.5 (built on the production workstation) |
| Host link | CP210x (`10c4:ea60`) → `/dev/arts_camera_sync`, via a powered 7-port USB hub on a rear port |
| Meter | Amprobe 37XR-A |

## B1 — flash

`BLINK` and then `SELFTEST` flashed over `/dev/arts_camera_sync`; esptool "Hash of data
verified" for all three images both times.

## B2 — BLINK on J1.1 (GPIO 25 only)

DC volts, J1.1 to GND: display alternated between **0 V and 3.27 V** (the meter updates
faster than the 2 Hz toggle). PASS: full-swing toggling through the 560 Ω.

## B3 — SELFTEST, console

| Quantity | Measured | Expect |
|---|---|---|
| capture clock | `capture running: 80000000 Hz (12.500 ns/tick), 2 camera(s), loopback on` | 80 MHz |
| trigger | `30 Hz, 100 us pulse, pads 25/26/27` | — |
| trigger period | mean 33 333.000 µs, min 33 333.000, max 33 333.000, **spread 0.000 µs**, n = 359 | spread 0.000 |
| dropped records | 0 of 360 pulses | 0 |

Trigger outputs, meter in Hz mode on J1.1, J2.1, J3.1 to GND: all three show activity,
display unstable between ~40 and ~80 Hz. J1.1 (the pad whose period the capture counter
measures at exactly 30 Hz) reads the same way, so the instability is the meter's counter on a
0.3 % duty pulse. Recorded as: **all three outputs driven; rate not resolvable with this
meter.** Per-channel triggering is confirmed in Stage C.

## B4 — strobe channels, no cameras

cam0 and cam1: `no captures`, "N pulses with no strobe" equal to the pulse count; skew `no
captures`. PASS: pull-ups present (no floating-input artifact).
