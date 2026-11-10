# arts-camera-sync-v2 — layout constraints and fabrication notes

## Stack-up and board

| | |
|---|---|
| Layers | 2 |
| Thickness | 1.6 mm |
| Copper | 1 oz both sides |
| Finish | HASL, lead-free |
| Silkscreen | white on green |
| Outline | as-built **80 × 55 mm** (≤ 100 × 80), filleted corners (~5 mm); pour follows the fillet |
| Mounting | 4 × M3 (3.2 mm) holes, one near each corner, ~5 mm from the edges |

**Both F.Cu and B.Cu carry a `GND` pour** at 0.3 mm clearance, outline-derived so the fill
follows the fillet. Short **B.Cu hops** are allowed so F.Cu stays uncrowded. After each hop,
place a **GND stitch via** within ~3 mm, on copper that exists on both layers (not on a
track, pad, or clearance halo). Adjacent hops may share one via. Add one stitch next to
TP2 (F.Cu-only pad) and one or two in the open pour on the USB side — that end has no PTH
GND. J1–J3 pin 3 already stitch; do not add vias on the TVS GND pads. Do not turn B.Cu into
a rat's nest. There is no high-speed content on this board, so this is about return-current
integrity next to an ESU rather than about impedance. As-built (layout approved 2026-09-08):
hop companions, a south row near D1–D6, a sparse grid on open pour, nothing in the antenna
keep-out, and no via fence along tracks. Do not add more.

**Length-matching the six camera runs is a review preference, not a timing requirement.**
FR4 delay is ~ps/mm and will not move the µs systematic the paper reports. A 45° fold on the
connector drop is enough to keep the short runs from looking unfinished; do not add long
serpentines to chase the longest net.

## Placement

Every signal lands on the DevKitC's **left** header row (see `01_schematic_and_connections.md`
§4). Trust the **3-D viewer** for USB vs antenna — the SnapEDA 2-D courtyard draws a
USB-looking box on the wrong end. As-built, pcbnew +X right: U1 is rotated 90°, **USB is +X
(right)**, **WROOM antenna is −X (left)**.

- **U1** with USB facing the board's **right** edge (cable clearance on that end). Routes
  may go **under the module** except in the antenna keep-out.
- **J1, J2, J3 along the south edge**, CAM1 / CAM2 / CAM3, wire entry facing outward. Leave
  room beyond that edge for the pigtails to turn without straining.
- **Diodes stay at the connectors** so the TVS remains on the pigtail side of the series R.
  **R1–R6 sit between the south header and those diodes** (R1–R3 560 Ω trigger, R4–R6 1 kΩ
  strobe). The connection table is still checkable: each Phoenix pin, its diode, and its
  resistor net name match §3.
- **JP1 between the module and the terminals**, on `TRIG_CAM1` (GPIO 25 pad), not on
  `TRIG_CAM1_OUT`.
- **TP1 (`+3V3`) and TP2 (`GND`) on the antenna / left edge.** Signal TPs (TRIG/STROBE
  CAM1–3) are a column on the USB / right edge.

Two clearances that are easy to miss and expensive to get wrong:

**The DevKitC's micro-USB connector must be reachable with a cable plugged in.** It is at the
**+X** end of the module. Leave ≥ 15 mm of clear board with no components taller than 2 mm.
This connector carries both power and the micro-ROS transport, so it is permanently occupied
in normal operation, not just during flashing.

**Antenna keep-out.** The WROOM-32E's PCB antenna sits at the **−X / USB-far** end. Grow the
copper keep-out so both F.Cu and B.Cu stay clear ≥ 2 mm past the WROOM PCB-antenna outline.
Tracks and pads stay allowed; pour stays forbidden. The keep-out is on the antenna end, not
under the USB connector. This board uses UART, not WiFi, so nothing breaks if the keep-out is
violated — but it costs nothing to observe and it avoids foreclosing a future wireless use.

## Strain relief

The pigtail must still be clamped so a tug cannot loosen a screw (decision 14). A pulled
contact drops the strobe on some pulses and leaves a plausible skew computed from the rest.

That clamp is **not on this PCB.** Cable-tie hole pairs behind J1–J3 were specified here
earlier and are withdrawn (2026-08-27): they spend board area on a job a case does better,
and the four M3 mounting holes already give the board a home in that case.

Strain relief is a later 3-D-printed enclosure task. The case must grip each camera
pigtail (tie bosses, a clamp bar, or equivalent) before the wires reach J1–J3. Until the
case exists, dress the pigtails with slack and do not tension them against the terminals.
Do not grow the outline, and do not add NPTH, for ties.

## Silkscreen

Silkscreen is a safety feature here, because the person wiring reads colours and labels, not
net names.

Required (as-built form is a single legend plus pin numbers, not a phrase on every pad):

- `CAM1` / `CAM2` / `CAM3` at the three terminals, and **1 / 2 / 3** at each position.
- One legend: `1 TRIG (BLK)` · `2 STROBE (WHT)` · `3 GND (BLU)`.
- `LOOPBACK — LEAVE CLOSED` beside JP1.
- Test point labels matching the net names in the netlist (`+3V3`, `GND`, TRIG/STROBE CAMn).
- Board name and revision: `ARTS CAMERA SYNC v2`.
- A short warning at the connector edge: `CONNECT BLU (OPTO_GND) FIRST`.

Forbidden:

- **Any text identifying a person, lab, institution, or repository.** This tree becomes the
  anonymous review capsule, and a photograph of this board is a tracked artifact like any
  other file in it.
- Do not silkscreen `36` or `39` anywhere near the module. Those GPIOs are labelled `VP` and
  `VN` on the DevKitC itself, and introducing a second naming scheme beside the first is how
  the two adjacent pins get swapped.

## Fabrication

Intended house: JLCPCB. Order with the stack-up above, and **assembly for R1–R6 (0402) and
D1–D6 (SOD-323)** — see `02_bom.md`. Sockets, Phoenix terminals, and the DevKitC stay
hand-fit. Prefer 45° or arcs; no 90° corners. Layout approved 2026-09-08; the fab order is
queued and not yet purchased. Boards have not arrived. `production/` stays empty until they
exist.

**Exports are generated from a reviewed commit, not from the working tree**, and are
committed under `../production/` only once the revision has actually been fabricated. Day-to-day
Gerbers are gitignored — an export regenerated on every change drifts out of step with the
source silently, which is worse than one being absent.

## Review checklist before ordering

The review happens against the design package, in this order. Item 2 is the one that matters
most and the one a tool cannot do for you.

1. DRC clean, and no unrouted nets.
2. **Pin by pin against `01_schematic_and_connections.md` §3 and §4, reading the physical
   footprint's pin numbering rather than the schematic symbol's.** DRC verifies geometry and
   says nothing about whether a footprint's pin 1 is the pin the schematic meant. That error
   class yields a board which assembles cleanly and mismeasures — the exact outcome this
   whole box exists to prevent.
3. Net count is 12 and the netlist matches §2 exactly, including that `JP1.1` sits on
   `TRIG_CAM1` and not on `TRIG_CAM1_OUT`.
4. `VP` carries `STROBE_CAM3` and `VN` carries `LOOPBACK`, not the reverse.
5. GPIO 6–11 have no nets at all.
6. Nothing on the `5V` pin.
7. Module footprint row spacing measured against a physical DevKitC with calipers.
8. USB clearance and antenna keep-out present.
9. Four M3 mounting holes near the corners (the case interface). No cable-tie holes
   on the PCB — pigtail strain relief is a case feature, not a board feature.
10. Silkscreen carries wire colours, and carries no identifying text.
