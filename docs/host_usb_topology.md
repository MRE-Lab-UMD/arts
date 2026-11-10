# Production workstation — USB topology and port plan

> **Role: host reference.** What the workstation's USB ports are (measured) and which device
> goes where (plan). Status lives in `AGENTS.md`; measured camera results in
> `docs/benchmarks/`. Re-map after any motherboard, BIOS or chassis change.

## Controllers (from `lspci` / `lsusb -t`, 2026-10-27)

| Controller | PCI | Root buses | IRQ (effective CPU on 2026-10-27) |
|---|---|---|---|
| CPU Thunderbolt 4 xHCI | `0000:00:0d.0` | bus 1 (USB 2, 1 port), bus 2 (USB 3, 1 port) | 127 (CPU 10) |
| Chipset xHCI | `0000:80:14.0` | bus 3 (USB 2, 14 ports), bus 4 (USB 3, 10 ports) | 154 (CPU 22) |

Bus 3 also carries internal devices (Bluetooth on port 14, an LED controller on port 12).

## Physical port map (measured 2026-10-27)

Method: one BFS-U3-63S4C with its own USB 3 cable plugged into each labelled port in turn;
`/sys/bus/usb/devices/*/speed` read on arrival. 5000 = USB 3 SuperSpeed (the camera's maximum),
480 = the port has no USB 3 lanes.

| Label | Physical | Bus-port | Controller | Speed with the camera |
|---|---|---|---|---|
| P1 | rear, black Type-A | 3-10 | chipset | **480 (USB 2 only)** |
| P2 | rear, black Type-A | 3-5 | chipset | **480 (USB 2 only)** |
| P3 | rear, blue Type-A | 4-4 | chipset | 5000 |
| P4 | rear, blue Type-A | 4-2 | chipset | 5000 |
| P5 | rear, USB-C | 4-6 | chipset | 5000 |
| P6 | rear, USB-C | 2-1 | **Thunderbolt** | 5000 |
| P7 | front, Type-A | 4-7 | chipset | 5000 |
| P8 | front, Type-A | 4-9 | chipset | 5000 |
| P9 | front, Type-A | 4-10 | chipset | 5000 |
| P10 | front, USB-C | 4-8 | chipset | 5000 |

The front Type-A ports are black but are USB 3: colour is not a reliable indicator on this
chassis. P6 is the only port on the second controller.

## Port plan

| Port | Device | Why |
|---|---|---|
| P3 | FLIR 24057132, RIGHT camera (sync-box J1 / cam0) | USB 3, direct, chipset controller |
| P6 | FLIR 24057131, LEFT camera (sync-box J2 / cam1; USB-C → Micro-B USB 3 cable) | the only port on the other controller: the two camera streams share no controller and no interrupt |
| P4, P5 (rear, USB 3) | Inverse3 left, right — direct, Haply's own USB-C cable | Haply's quick start (docs.haply.co/docs/quick-start, read 2026-10-28) says to connect the Inverse3 directly with the provided cable and to avoid any dongle or intermediary connection; it states no USB version. In the control path, so no hub and a USB 3 port, not a USB 2-only one. The device's negotiated speed is to be read on arrival |
| P7, P8 (front, USB 3) | Dynamixel adapters (instrument motor packs, left/right) | in the control loop: direct, no hub |
| P2 (USB 2) | console hub (powered): 2× VerseGrip transponders, Savant Elite2 pedal (`29ea:0100`, full speed), GOOVIS USB (power + head sensor) | console inputs at human rate, not in a haptic-rate loop; E6 measures end-to-end, so any hub latency is inside the reported number |
| P1 (USB 2) | bench hub (powered): camera-sync box, ESU-TAP, Xbox controller, keyboard, mouse | devices insensitive to host-side delay (sync box timestamps on board; ESU commands expire at 500 ms) |
| P9, P10 (front, USB 3) | spare; external SSD for bag export, BETWEEN recordings only (same controller as the right camera) | — |

**Swapped 2026-10-31 for cable length:** the Inverse3s moved from P7/P8 to P4/P5 and the
Dynamixel adapters the other way. All four ports are USB 3 on the same chipset controller,
direct, no hub, so the plan's reasons hold. Nothing binds to a port: `haply_bridge` maps the
Inverse3s by device ID (`arts_driver/config/haply.yaml`). E6 (2026-10-30) ran with the
Inverse3s on P7/P8.

The USB 2-only ports (P1, P2) take only devices with no real-time concern. Both hubs are in
use since 2026-10-31: P1 (patient-cart hub) carries the camera-sync box and the Xbox
controller; P2 (console hub) carries the Savant Elite2 pedal, the GOOVIS USB and both
VerseGrip transponders.

The camera-sync box is on a hub deliberately: its timestamps are taken on the board's own
80 MHz counter, so host USB latency does not enter the skew figure.

## Open

- **Pin both xHCI interrupts (127, 154) to housekeeping cores** persistently. Today both
  land off the isolated cores (2, 4, 6, 7) only by chance: their allowed mask is 0–23 and
  `irqbalance` is inactive. Re-run `deploy/bench/rt_latency.sh` after pinning.
- E4 records this table's camera rows (port, bus-port, controller) beside the result.
