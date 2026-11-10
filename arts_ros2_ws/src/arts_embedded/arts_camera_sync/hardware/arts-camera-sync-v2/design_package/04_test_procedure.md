# arts-camera-sync-v2 — bring-up and acceptance procedure

Do these in order. Stage A happens with **no ESP32 installed and no power anywhere**, and it
is the step that catches the failure mode a fabricated board actually has: a correct-looking
layout wired to the wrong footprint pin.

Reference figures throughout come from the breadboard rig
(`docs/benchmarks/camera_sync_skew_breadboard_20260819.md`). They are what this board must
match or beat, not a specification it is allowed to differ from silently.

---

## Finding the pins on the bare board

The names below are labels, and several of them are NOT printed on this PCB:

- **J1, J2, J3** are the three 3-position screw terminals along the bottom edge, silkscreened
  `CAM1`, `CAM2`, `CAM3`. **`J1.2` = terminal J1, position 2.** Positions are printed `1 2 3`
  (1 = TRIG / black, 2 = STROBE / white, 3 = GND / blue).
- **J4, J5** are the two 1×19 header sockets the DevKitC plugs into. They are BOM items
  (`02_bom.md`) soldered into U1's footprint holes; the board has no `J4`/`J5` designator.
  Call the one nearer the screw terminals the **signal socket**. Every measured net lands
  on it; the far one carries only GND and unconnected pins.
- **`25`, `VP`, `3V3` etc.** are the DevKitC module's own silkscreen, so they are only
  visible with the module plugged in. Step A0 below transfers them onto the board before
  the module comes out. Position 1 of the signal socket is the **square pad** (visible on
  the underside once the socket is soldered), at the CAM1 end, counting toward CAM3.
- Unrelated name clash: the footprint's pad names `J2_n` / `J3_n` (used in
  `01_schematic_and_connections.md`) are Espressif's names for the module's two headers, not
  the J2/J3 screw terminals. `J2_n` = signal-socket position n; `J3_n` = far-socket position n.

| Signal-socket position | Module label | Net | Test point |
|---|---|---|---|
| 1 (square pad) | `3V3` | `+3V3` | TP1 |
| 3 | `VP` (GPIO 36) | `STROBE_CAM3` | TP8 |
| 4 | `VN` (GPIO 39) | `LOOPBACK` | — |
| 5 | `34` | `STROBE_CAM1` | TP6 |
| 6 | `35` | `STROBE_CAM2` | TP7 |
| 9 | `25` | `TRIG_CAM1` | TP3 |
| 10 | `26` | `TRIG_CAM2` | TP4 |
| 11 | `27` | `TRIG_CAM3` | TP5 |
| 14 | `GND` | `GND` | TP2 |
| 16, 17, 18 | `SD2`, `SD3`, `CMD` | must be open | — |
| 2, 7, 8, 12, 13, 15, 19 | `EN`, `32`, `33`, `14`, `12`, `13`, `5V` | must be open | — |

Far socket: positions **1 and 7 are GND** (same end as the square pad); 17, 18, 19 are
`SD1`, `SD0`, `CLK` and must be open; everything else is unconnected.

The test points are the same nets as the socket pins and are a faster probe target, but
Stage A exists to prove the socket pins themselves, so measure at the socket.

---

## Stage A — continuity, unpowered, module removed

**A0. Orient yourself (module in, NO power, no USB, no measurements).**
1. Seat the DevKitC in the sockets.
2. Read the module's labels beside the signal socket. Confirm `3V3` sits over the square
   pad at the CAM1 end and `GND` is the 14th pin from it. If `3V3` is at the other end, the
   module is in backwards: stop, it would be destroyed at power-on.
3. Mark the PCB beside the signal socket with a fine marker (or take a top-down photo) at
   positions **1 `3V3`, 3 `VP`, 4 `VN`, 5 `34`, 6 `35`, 9 `25`, 10 `26`, 11 `27`, 14 `GND`**.
4. Pull the module straight out.
5. Self-check the marks: signal-socket position 1 to TP1 < 1 Ω, position 14 to TP2 < 1 Ω.

**Why the module must be OUT for every measurement below:** seated, its regulator and
decoupling capacitors sit across `3V3`–`GND`, its flash chip hangs on the `SD`/`CMD`/`CLK`
pins, and every GPIO has protection diodes to the supply. The "open" checks in A5 and A6
would fail on the module, not the board, and the resistor readings in A1–A2 can be pulled
low by those parallel paths.

Multimeter on continuity or resistance. Probe the **socket** contacts (J4/J5), not the
module.

**A1. Series resistors.** Between each trigger terminal and its socket pin:

| From | To (module label = signal-socket position) | Expect |
|---|---|---|
| J1.1 `CAM1 TRIG` | `25` = pos 9 | 560 Ω ± 6 Ω |
| J2.1 `CAM2 TRIG` | `26` = pos 10 | 560 Ω ± 6 Ω |
| J3.1 `CAM3 TRIG` | `27` = pos 11 | 560 Ω ± 6 Ω |

**A2. Pull-ups, and the strobe path.** Each strobe terminal must read ~0 Ω to its socket pin
and 1 kΩ to the `3V3` pin:

| From | To | Expect |
|---|---|---|
| J1.2 `CAM1 STROBE` | `34` | < 1 Ω |
| J1.2 | `3V3` | 1 kΩ ± 10 Ω |
| J2.2 `CAM2 STROBE` | `35` | < 1 Ω |
| J2.2 | `3V3` | 1 kΩ ± 10 Ω |
| J3.2 `CAM3 STROBE` | **`VP`** | < 1 Ω |
| J3.2 | `3V3` | 1 kΩ ± 10 Ω |

**A3. The pin most likely to be wrong.** `VN` must read < 1 Ω to the `25` socket pin, through
JP1, and must **not** be continuous with J3.2:

| From | To | Expect |
|---|---|---|
| `VN` | `25` | < 1 Ω (the loopback, via JP1) |
| `VN` | J3.2 `CAM3 STROBE` | open |
| `VP` | `25` | open |

A `VP`/`VN` swap passes A2's first column if you probe by GPIO number instead of by
module label. `VP` is position 3 and `VN` is position 4; count from the square pad.

**A4. Grounds.** J1.3 / J2.3 / J3.3 each < 1 Ω to the `GND` socket pin, and to each other.

**A5. Isolation.** Open circuit between every pair below. Any continuity here is a short that
would drive a camera input from the wrong channel or from the supply:

- J1.1 ↔ J2.1 ↔ J3.1 (the three triggers, downstream of their resistors)
- J1.2 ↔ J2.2 ↔ J3.2 (the three strobes)
- any trigger terminal ↔ any strobe terminal
- any terminal ↔ `GND`, except the three `GND` terminals
- `3V3` ↔ `GND`

**A6. Boot pins.** `SD0`, `SD1`, `SD2`, `SD3`, `CMD`, `CLK` socket pins: open to everything.

Do not proceed past a failure in Stage A. Record the measured resistor values — they are the
starting point for interpreting the systematic term in Stage D.

---

## Stage B — module installed, no cameras

**B1.** Install the ESP32. Connect its USB. Flash the `BLINK` build.

**B2.** Meter on J1.1 to `GND`, DC volts. The pin toggles 0 ↔ 3.3 V at 2 Hz, so what you see
depends on the meter's update rate, and all three of these are a PASS:
- the display **alternates between ~0 V and ~3.3 V** (a meter updating faster than the
  2 Hz toggle samples each half; observed 2026-10-27 on an Amprobe 37XR-A: 0 V / 3.27 V);
- a fluctuating ~1.5–2.3 V (a slower meter averaging the two levels);
- a steady ~1.65 V (a meter that fully averages).

A **steady** 0 V or a **steady** 3.3 V is a fail: the pin is not toggling.

`BLINK` drives **GPIO 25 only** (`ARTS_SYNC_BLINK_GPIO` = the primary trigger), so J2.1 and
J3.1 read a steady 0 V here and that is NOT a fail. They are checked under `SELFTEST` in B3.

**B3.** Flash the `SELFTEST` build. On the console, confirm:

- `capture running: 80000000 Hz (12.500 ns/tick)` — the capture clock. If this reads
  160 MHz, every published skew figure is out by 2× and nothing else looks wrong.
- trigger period mean 33 333.000 µs with **spread 0.000 µs** over a few hundred intervals,
  and 0 dropped records. The loopback is working and the timebase is sound.
- **All three trigger outputs are driven.** `SELFTEST` fans the trigger out to GPIO 26 and
  27 as well, but as a 100 µs pulse at 30 Hz (0.3 % duty), which a DC voltmeter reads as
  ~10 mV. Use the meter's **Hz** mode on J1.1, J2.1 and J3.1 to `GND`: expect **30 Hz** on
  each. A meter that reads 0 Hz on all three including J1.1 is failing to trigger on the
  narrow pulse, not a board fault — then Stage C (each camera on its own channel) is the
  check for J2/J3.

**B4.** With no cameras attached, the strobe channels must report **`no captures`**, not a
number. If they instead report a delay equal to `CONFIG_ARTS_SYNC_PULSE_WIDTH_US` with
0.000 µs spread and a skew of exactly 0.000 µs, the pull-ups are missing or open — that
reading is the floating-input artifact from rung 5b and it is entirely fictitious. Stage A2
should have caught it; if B4 disagrees with A2, believe B4 and re-probe.

---

## Stage C — one camera

**C1.** Attach camera 1. **Blue wire into J1.3 first**, then black into J1.1, then white into
J1.2. Leave slack in the pigtail; do not tension the screws. Strain relief waits on the
3-D-printed case — there are no cable-tie holes on this board.

**C2.** Configure the camera through `arts_driver`'s triggered profile
(`cameras.launch.py trigger:=true`). Confirm the driver reports `IN: 30.00 Hz`, flat. Note
that `OUT: 0.00 Hz` alongside a healthy `IN` is not a fault — the driver only publishes when
something is subscribed.

**C3.** On the SELFTEST console, confirm:

| Quantity | Breadboard reference | Interpretation |
|---|---|---|
| trigger → exposure | 711.7 µs | first-row (`Any pixel`) assertion, plus `TriggerDelay` 379 µs, plus optocoupler in and out |
| exposure width | 4977 µs against a commanded `ExposureTime` of 5004 µs | the ~27 µs shortfall is the strobe rise delay |
| pulses with no strobe | 0 of 240 | any non-zero count is a wiring or pull-up problem, not a camera problem |

**The width check is the polarity check** and it is not optional. If the width comes out as
the trigger period *minus* the exposure (~28 000 µs) rather than the exposure itself, the
firmware is calling the wrong edge the exposure start and every absolute figure is the
exposure's *end*. That reading is stable, tight and plausible; the width is the only thing
that exposes it.

**C4.** Repeat C1–C3 for camera 2 on J2, and camera 3 on J3 if populated. Each channel is
verified alone before any pair is measured.

---

## Stage D — acceptance: the committed skew baseline

**D1.** Both cameras attached and streaming. Collect ≥ 700 pulses.

**D2.** Record from the console: per-camera mean delay and width, `|cam0 − cam1|` mean / max /
min, the systematic (difference of the two means), trigger period spread, and dropped-record
and missed-strobe counts.

**D3.** Repeat through the micro-ROS path — agent up, `ros2 bag record` of
`/arts_camera_sync/pulse` plus both `~/<cam>/meta` topics — and compute the same figures from
the bag. Two traps: the pulse topic is published **best-effort**, so `ros2 topic echo` needs
`--qos-reliability best_effort` (bag record adapts by itself); and `flir_camera_msgs` must be
on the path or rosbag2 records the pulse topic perfectly and the meta topics not at all,
without erroring.

Analyze with `analyze_sync.py --cam0-meta <meta topic of the camera on J1>`
(`/camera_right/meta` since 2026-10-28). Without it the software columns are paired with the
capture channels alphabetically, which is wrong when J1 is not the left camera; the per-pulse
join then drops from r ≈ 0.99 to ≈ 0.45 and the analyzer warns.

**D4.** Cast capture ticks to a **signed** integer before differencing. `uint32` subtraction
wraps and manufactures an 11-second "skew" that is exactly 2³² ticks.

**Acceptance criteria:**

| | Target | Breadboard achieved |
|---|---|---|
| skew, hardware | < 1 ms (expect < 100 µs) | mean 3.52 µs, max 10.44 µs |
| trigger period spread | ≤ 1 tick | 0.000 µs |
| missed strobes | 0, excluding pulses where the camera genuinely did not expose | 3 of 1001 |
| software cross-check | `camera_time` (L−R) residual << 1 ms | std 22.8 µs, 87 µs p-p |
| systematic offset | **report it**; expect ≤ 2.81 µs | 2.81 µs |

**Classify every missed strobe from the bag; do not assume it.** `analyze_sync.py` reports
`frame gaps` per camera from `camera_time`. Zero gaps means the camera exposed on every pulse,
so each missing capture is an edge the box did not register and counts against this
criterion. The breadboard's 3 of 1001 were attributed to non-exposure without that check. On
the PCB (2026-10-28) the check showed 16 of 8991 lost edges with no lost frame. They were the
driver mislabelling the edge type, fixed in firmware `7859c33`. If lost edges reappear,
build SELFTEST with `CONFIG_ARTS_SYNC_EDGE_DIAG=y`, which prints every capture event of an
anomalous pulse.

The systematic is the figure to watch, because it is the one the board was expected to
improve. If the 1 % resistors did their job it should fall below the breadboard's 2.81 µs.
If it does not, the residual is upstream of our wiring — cable length or camera-internal
variation — and that is a publishable finding rather than a defect, provided it is reported
rather than assumed.

The random term will **not** improve, and should not be expected to: the ±8.6 µs per camera
is the sensor's 8.3 µs line period, since the camera can only begin exposing on a line
boundary. It is irreducible at this ROI and independent between cameras. Shrinking the ROI
vertically shortens it proportionally.

**D5.** Commit the result as `docs/benchmarks/camera_sync_skew_v2pcb_<date>.md`, recording
the board revision, the measured resistor values from Stage A, the `trigger_delay` and
`ExposureTime` set on each camera, and the firmware revision. This is the **first
authoritative skew baseline** — the breadboard figures are bring-up results and are not
eligible.
