# arts-camera-sync-v2

First published revision of the ARTS camera sync box.

| | |
|---|---|
| **Status** | Layout approved 2026-09-08. KiCad 9 source is in this directory. JLCPCB order queued (2 layer, 1.6 mm, HASL lead-free, green; PCBA R1–R6 + D1–D6 only). Not yet purchased or fabricated. No committed skew baseline. |
| **Toolchain** | KiCad 9 |
| **Fab house / options** | JLCPCB — order queued, awaiting purchase. 2 layer, 1.6 mm, HASL lead-free, green. Assembly: R1–R6 (0402) and D1–D6 (SOD-323) only. Sockets, Phoenix J1–J3, DevKitC, JP1 hand-fit. |
| **Known-good firmware** | none on this board yet. The breadboard rig ran `arts_camera_sync` firmware as of 2026-08-19 (micro-ROS build, 266 KB); this board must be re-baselined against whatever revision is current when it is populated. |
| **Supersedes** | the breadboard + OONO screw-terminal rig described in `docs/modules/camera_sync_wiring.md` |

Open `arts-camera-sync-v2.kicad_pro`. Libraries resolve through `${KIPRJMOD}`.

## What this board is for

It fires all cameras from one hardware edge and captures each camera's `ExposureActive`
strobe back on one 80 MHz counter, so L/R stereo skew is a hardware measurement — the
difference between two capture timestamps on a single counter, independent of ROS,
Spinnaker, DDS and any clock estimator.

The electrical function is small: three series resistors, three pull-ups, three connectors
and a socketed ESP32. The board exists not because the circuit is complex but because a
spring-clip contact on stranded wire fails intermittently, and intermittently here means a
dropped strobe on some pulses with a plausible skew computed from the rest — a silently
wrong number, which is the exact failure this box was built to eliminate.

## What it must beat

The breadboard rig measured skew mean 3.52 µs / max 10.44 µs with a **2.81 µs systematic**
offset, of which 0.72 µs was attributable to the strobe-side rise-delay mismatch and the
remaining ~1.9 µs to something upstream — 560 Ω tolerance changing the optocoupler drive
current, cable length, or camera-internal variation. The single highest-value change on this
board is therefore **1 % resistors from one reel instead of 5 % carbon films**, which
addresses the largest identified contributor for a few cents. See `02_bom.md`.

**No committed skew baseline may come from the breadboard.** Resistor instances, lead lengths
and contact resistance move the systematic term by the same order as the effect being
measured, so the breadboard number is a bring-up result and the board's number is the
published one.

## Contents

```
arts-camera-sync-v2.kicad_pro / .kicad_sch / .kicad_pcb
fp-lib-table  sym-lib-table
libraries/                    project-local, ${KIPRJMOD}
design_package/
├─ 01_schematic_and_connections.md
├─ 02_bom.md
├─ 03_layout_and_fab.md
└─ 04_test_procedure.md
```

`production/` is absent until a fabricated revision exists. Read `01` first.

## Third-party EDA assets

| Asset | Source | Downloaded | License |
|---|---|---|---|
| `ESP32-DEVKITC-32E` symbol, footprint, STEP (`--3DModel-STEP-269445`) | [SnapEDA / Espressif ESP32-DEVKITC-32E](https://www.snapeda.com/parts/ESP32-DEVKITC-32E/Espressif+Systems/view-part/) | 2026-08-19 | SnapEDA standard: CC-BY-SA 4.0 with the SnapEDA Design Exception (permits use in a design without share-alike on the board). Re-verify at next download. |

Attribution to SnapEDA identifies nobody. Do not copy this part from another project's tree;
the fetch recipe is the URL above.
