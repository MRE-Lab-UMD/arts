# camera_sync v2 PCB — Stage A continuity, 2026-10-27

> **Role: measured numbers (layer 4).** One measurement session. Immutable once committed.
> Procedure: `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/design_package/04_test_procedure.md`, Stage A.

**Result: Stage A PASS (A1–A6).**

| | |
|---|---|
| Board | `arts-camera-sync-v2` (first fabricated revision), hand-fitted: sockets, screw terminals. Individual unit not serialized. |
| Module | Espressif ESP32-DevKitC-32E (ESP32-WROOM-32E); seated only for the A0 orientation step |
| State | unpowered, no USB, module removed for every measurement |
| Meter | Amprobe 37XR-A; lead offset not recorded (the 0.0–0.2 Ω readings bound it) |

## A1 — trigger series resistors (spec 560 Ω ± 6 Ω)

| From | To (signal-socket position) | Measured | Result |
|---|---|---|---|
| J1.1 CAM1 TRIG | `25` (pos 9) | 559.3 Ω | pass |
| J2.1 CAM2 TRIG | `26` (pos 10) | 560.0 Ω | pass |
| J3.1 CAM3 TRIG | `27` (pos 11) | 560.7 Ω | pass |

## A2 — strobe path and pull-ups (spec < 1 Ω; 1 kΩ ± 10 Ω)

| From | To | Measured | Result |
|---|---|---|---|
| J1.2 CAM1 STROBE | `34` (pos 5) | 0.0 Ω | pass |
| J1.2 | `3V3` (pos 1) | 991 Ω | pass |
| J2.2 CAM2 STROBE | `35` (pos 6) | 0.2 Ω | pass |
| J2.2 | `3V3` | 990 Ω | pass (at the lower bound) |
| J3.2 CAM3 STROBE | `VP` (pos 3) | 0.2 Ω | pass |
| J3.2 | `3V3` | 994 Ω | pass |

## A3 — loopback and the VP/VN trap

| From | To | Measured | Result |
|---|---|---|---|
| `VN` (pos 4) | `25` (pos 9) | 0.2 Ω | pass |
| `VN` | J3.2 | open | pass |
| `VP` (pos 3) | `25` | open | pass |

## A4 — grounds

J1.3 / J2.3 / J3.3 to socket `GND` and to each other: 0.0 Ω. Pass.

## A5 — isolation (all must be open)

J1.1↔J2.1↔J3.1; J1.2↔J2.2↔J3.2; any TRIG↔any STROBE; any terminal↔GND except the three GND
terminals; `3V3`↔`GND`: all open. Pass (reported as a group; individual pair readings not
recorded).

## A6 — boot pins (all must be open)

`SD0`, `SD1`, `SD2`, `SD3`, `CMD`, `CLK`: all open. Pass (reported as a group).
