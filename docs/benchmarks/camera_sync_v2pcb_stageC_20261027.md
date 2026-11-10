# Camera-sync PCB v2 — Stage C (cameras, one terminal at a time), 2026-10-27

Layer 4 (measured numbers). Procedure: `hardware/arts-camera-sync-v2/design_package/04_test_procedure.md`.
Board, module, firmware and toolchain as in `camera_sync_v2pcb_stageB_20261027.md` (SELFTEST build).

## Setup

| Terminal | Firmware label | Camera | Role | Host port (`docs/host_usb_topology.md`) |
|---|---|---|---|---|
| J1 | cam0 | BFS-U3-63S4C 24057132 | LEFT | P3 (bus 4-4, chipset xHCI), 5000 Mb/s |
| J2 | cam1 | BFS-U3-63S4C 24057131 | RIGHT | P6 (bus 2-1, Thunderbolt xHCI), 5000 Mb/s |

Both cameras firmware 1801.0.9.0, Spinnaker 4.2.0.88, `cameras.launch.py trigger:=true` at
commit `41d551a` (+ this report): BayerRG8, 3072×2048, exposure 5000 µs, gain 10 dB,
`TriggerSource` Line0, `RisingEdge`, `TriggerOverlap` ReadOut, `TriggerDelay` 379 µs,
`isp_enable` False. Trigger 30 Hz from the sync box. Roles: left/right swapped from Stage 0
(head viewed from the side facing the pedestal).

## Failure first observed (camera 24057132, before the fix)

Driver `IN: 17.2–17.4 Hz`; trigger→exposure n=1254, mean 16 644.848 µs, min 34.175, max
33 329.900 µs (uniform, not locked). Holding the ESP32 in reset gave `IN: 0.00 Hz`, so the camera
was triggered by the board. Node map: `User Set Default: UserSet1` (saved by collaborator A
with the Spinnaker GUI), `ISP Enable: 1`, `Acquisition Line Rate` 40 099.4, `Device Max
Throughput` 119 625 647, `Resulting Frame Rate` 17.2532. With the factory `Default` loaded
(volatile): line rate 128 700, resulting rate 59.6472, ISP off. Camera 24057131 also boots
`UserSet1` with ISP on (line rate 40 099.4). Camera flash not modified.

## Results (sync box reset, both cameras streaming, SELFTEST console, cumulative since boot)

| Quantity | n | mean | min | max | spread |
|---|---|---|---|---|---|
| Trigger period | 1142 | 33 333.000 µs | 33 333.000 | 33 333.000 | 0.000 |
| cam0 trigger→exposure | 1141 | 709.149 µs | 704.888 | 713.363 | 8.475 |
| cam0 exposure width | 1141 | 4976.594 µs | 4976.363 | 4976.700 | 0.338 |
| cam1 trigger→exposure | 1142 | 710.762 µs | 706.425 | 715.200 | 8.775 |
| cam1 exposure width | 1142 | 4977.874 µs | 4977.688 | 4978.025 | 0.338 |
| \|cam0−cam1\| (preview, not the E3 baseline) | 1140 | 2.919 µs | 0.000 | 9.638 | 9.638 |

Pulses 1143, dropped records 0. Pulses with no strobe: cam0 2, cam1 1. Driver: both
`IN: 30.00 Hz`, 0 % drop.

Single-camera run on J1 earlier (camera 24057132 alone, from reloaded `UserSet1`): n=1018,
trigger→exposure 708.169 µs (spread 8.638), width 4977.200 µs (spread 0.275), 1 pulse with no
strobe, `IN: 30.00 Hz`.

## `IspEnable` write after a power cycle

In the two-camera launch above the driver logged for `camera_left` (24057132): `setting
ImageFormatControl/IspEnable failed` and `set to: 1 instead of: 0`, yet it ran at 30.00 Hz: the
camera had stayed powered since an earlier run had written ISP off. Re-tested after unplugging
24057132's USB: the write failed again (`node ImageFormatControl/IspEnable not writable`) and
the camera ran at `IN: 17.20 Hz`. The results table above therefore depended on volatile state.

Change: a `trigger_mode_disarm` entry (TriggerMode Off) written first in the definition file.
Re-tested after unplugging BOTH cameras' USB: boot state on both `IspEnable 1`, `TriggerMode
Off`, line rate 40 099.4; the driver wrote `IspEnable 0` with no failure; both `IN: 30.00 Hz`.
Since TriggerMode was already Off at boot, why the earlier write was refused is not known.
