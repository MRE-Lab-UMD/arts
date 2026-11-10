# arts-camera-sync-v2 — schematic specification and connection table

**This is the specification the schematic is drawn from, and the document the finished
schematic is reviewed against, pin by pin.** It is not a description of a schematic that
already exists.

Authoritative pin map source: `../../../firmware/main/arts_sync_config.h`. If this document
and that header ever disagree, the header wins and this document is wrong.

---

## 1. Function

One ESP32-DevKitC-32E, socketed. Per camera channel:

- a **trigger output** from an ESP32 pad, through a series resistor, into the camera's
  opto-isolated `Line 0` input;
- a **strobe input** from the camera's `Line 1` (`ExposureActive`) output, with a pull-up to
  3.3 V, into an input-only ESP32 pad;
- the camera's `OPTO_GND` return to board ground.

Plus one **trigger loopback**: the primary trigger pad routed back into a second input-only
pad, so the trigger instant is captured on the same counter as the strobes and absolute
trigger-to-exposure latency is measurable.

Three channels are provided and all three are populated. The limit on the third camera is
firmware, not hardware: MCPWM group 0 offers exactly three capture channels and the loopback
consumes one, so activating camera 3 means `ARTS_SYNC_TRIGGER_LOOPBACK=n`. Populating the
channel anyway costs two resistors and one connector, and makes that a Kconfig change rather
than a soldering session.

---

## 2. Netlist

Twelve nets. `Jn.p` = connector n position p, `Rn.1/.2` = resistor legs, `Dn.1/.2` = TVS
legs, `L<n>` = DevKitC **left** header position n (see §4).

| Net | Members |
|---|---|
| `+3V3` | DevKitC `3V3` (L1) · R4.1 · R5.1 · R6.1 · TP_3V3 |
| `GND` | DevKitC `GND` (L14 / `J2_14`) · DevKitC `J3_1` · DevKitC `J3_7` · J1.3 · J2.3 · J3.3 · D1.2 · D2.2 · D3.2 · D4.2 · D5.2 · D6.2 · TP_GND |
| `TRIG_CAM1` | DevKitC `IO25` (L9) · R1.1 · JP1.1 · TP_TRIG1 |
| `TRIG_CAM1_OUT` | R1.2 · J1.1 · D1.1 |
| `TRIG_CAM2` | DevKitC `IO26` (L10) · R2.1 · TP_TRIG2 |
| `TRIG_CAM2_OUT` | R2.2 · J2.1 · D2.1 |
| `TRIG_CAM3` | DevKitC `IO27` (L11) · R3.1 · TP_TRIG3 |
| `TRIG_CAM3_OUT` | R3.2 · J3.1 · D3.1 |
| `STROBE_CAM1` | DevKitC `IO34` (L5) · R4.2 · J1.2 · D4.1 · TP_STR1 |
| `STROBE_CAM2` | DevKitC `IO35` (L6) · R5.2 · J2.2 · D5.1 · TP_STR2 |
| `STROBE_CAM3` | DevKitC `VP` = GPIO 36 (L3) · R6.2 · J3.2 · D6.1 · TP_STR3 |
| `LOOPBACK` | JP1.2 · DevKitC `VN` = GPIO 39 (L4) |

R1–R3 are 560 Ω (trigger series). R4–R6 are 1 kΩ (strobe pull-up). See `02_bom.md`.

---

## 3. Camera-side connection table

Each `Jn` is a 3-position screw terminal taking the bare-wire end of one FLIR GPIO pigtail.
The camera end of that pigtail is a 6-pin Hirose plug that is already fitted; only three of
its six wires are used.

| Terminal | Wire colour | Hirose pin | Camera function | Goes to |
|---|---|---|---|---|
| `Jn.1` | **BLACK** | 2 | `Line 0`, opto-isolated **input** | trigger, via the 560 Ω |
| `Jn.2` | **WHITE** | 4 | `Line 1`, opto-isolated **output** (`ExposureActive`) | strobe capture, with the 1 kΩ pull-up |
| `Jn.3` | **BLUE** | 5 | `OPTO_GND` | board `GND` — the return for **both** lines above |

**The three unused wires are cut back and insulated. They do not get a terminal.**

- **green** (pin 1) is `Line 3`, a non-isolated 3.3 V I/O whose absolute maximum input-high
  is **3.6 V**. Upstream's own `follower_example.launch.py` uses this pin as the trigger
  source; a 5 V drive there destroys the camera. Designing it out beats documenting it.
- **red** (pin 3) is a power pin.
- **brown** (pin 6) is the camera's **non-isolated ground**. Landing it anywhere near our
  `GND` shorts across the optocoupler and silently discards the ESU isolation that is the
  entire reason `Line 0` was chosen. A park terminal would make that mistake reachable by
  someone tidying up loose wires; a cut-and-heatshrunk wire does not.

**FLIR's warning applies and is not optional: connect `OPTO_GND` before applying any voltage
to a GPIO line.** In terminal order that means blue goes in first.

---

## 4. DevKitC connection table — the pin-by-pin review artifact

Every signal on this board lands on the DevKitC's **left** header row. That is a layout
convenience worth preserving: all three camera connectors can sit along one board edge and
nothing needs to route under the module.

Positions are counted from the `3V3` end of the left row. The **silkscreen label is the
primary key** — it is what a human reads — and the position is a cross-check.

| Left pos. | Silkscreen | GPIO | Net | Direction |
|---|---|---|---|---|
| L1 | `3V3` | — | `+3V3` | supply out (feeds the three pull-ups) |
| L3 | `VP` | **36** | `STROBE_CAM3` | input only |
| L4 | `VN` | **39** | `LOOPBACK` | input only |
| L5 | `34` | 34 | `STROBE_CAM1` | input only |
| L6 | `35` | 35 | `STROBE_CAM2` | input only |
| L9 | `25` | 25 | `TRIG_CAM1` | output |
| L10 | `26` | 26 | `TRIG_CAM2` | output |
| L11 | `27` | 27 | `TRIG_CAM3` | output |
| L14 | `GND` | — | `GND` | return |

Extra DevKitC GND pins on the **right** header (`J3_1`, `J3_7`) join the same `GND` net.
That is more members of one net, not a thirteenth net. Do not add a second ground. `J3_13`
and `J3_19` (`IO4` / `CLK`) stay unconnected.

**BOARD-LABEL TRAP, and it is the reason this table exists in two keys.** GPIO 36 and 39 are
silkscreened by their analog alternate function on the DevKitC and on every breakout for it,
so **no terminal is marked 36 or 39**: GPIO 36 = `VP` (SENSOR_VP), GPIO 39 = `VN`
(SENSOR_VN). They are adjacent to each other, so a swap is easy and produces no error — the
strobe for camera 3 and the trigger loopback would simply exchange places, and with camera 3
unpopulated the symptom is that the loopback appears dead while everything else works.

**Pins that must be left entirely unconnected:**

- `SD0`, `SD1`, `SD2`, `SD3`, `CMD`, `CLK` (GPIO 6–11, right header) are the SPI flash pins.
  Any net on them prevents boot. They are not available as convenient routing pins.
- `TX0` / `RX0` (GPIO 1 / 3) carry the micro-ROS serial transport out through the DevKitC's
  own USB connector. Nothing on this board touches them.
- `EN` and `IO0` are reset and boot-mode. The DevKitC has its own buttons; do not add more.
- `5V` — see design decision 4 below.

**Verify the footprint against a physical board with calipers before ordering.** Two
DevKitC row-spacing variants exist. DRC will happily pass a board that the module does not
fit. The ESU-TAP KiCad project carries an `ESP32-DEVKITC-32E` symbol, footprint and 3-D model
that has been through fabrication once. Its provenance was resolved 2026-08-21: the files are
a **SnapEDA (SnapMagic) download**, not the upstream author's own work (the symbol embeds
`SnapEDA_Link` properties; the STEP uses SnapEDA's `--3DModel-STEP-<id>` naming). So do NOT
copy it from ESU-TAP: download the identical part fresh from SnapEDA, verify the license
shown at download time (their standard is CC-BY-SA 4.0 with the SnapEDA Design Exception,
which permits use in designs without share-alike obligations on the board), vendor it under
`libraries/` in this revision, and record source URL + date + license in the revision README.
The ESU-TAP copy remains useful as the fabrication-proven reference to diff against.

---

## 5. Design decisions the drawer must not silently change

Each of these looks like an arbitrary choice and is not. If one seems wrong, raise it at
review rather than improving it in the layout.

**1. The loopback taps the pad, not the resistor output.** `JP1.1` connects to
`TRIG_CAM1` — the GPIO 25 node — and **not** to `TRIG_CAM1_OUT`. The loopback's job is to
timestamp the ESP32's own edge. Tapping after the 560 Ω puts the camera's optocoupler LED
capacitance into the loopback's edge and, worse, makes the loopback's timing depend on
whether a camera is plugged in — which turns the absolute trigger-to-exposure latency into a
function of what happens to be connected.

**2. The loopback is a normally-closed solder jumper, not a removable header.** It is
permanent wiring for the two-camera build: with `ARTS_SYNC_TRIGGER_LOOPBACK=y`, the loopback
capture is the *only* caller that opens a pulse record, so breaking it makes SELFTEST report
"no captures" on every channel even while both cameras trigger correctly at 30 Hz. A pin
header invites removal and that symptom names nothing. A cut-able track still allows GPIO 39
to be freed later without a respin. Silkscreen it `LOOPBACK — LEAVE CLOSED`.

**3. Do not length-match the channels.** FR4 propagation is ~6 ps/mm, so even a 50 mm
channel-to-channel mismatch is 0.3 ns — 0.024 capture ticks, four orders of magnitude below
the 2.81 µs systematic actually being chased. Spending layout effort here buys nothing. What
does set the systematic is resistor tolerance and optocoupler variation, which is a BOM
problem and is addressed in `02_bom.md`.

**4. There is no power input other than the DevKitC's own USB.** No barrel jack, no 5 V
screw terminal, nothing on the `5V` pin. That USB carries both the board's power and the
micro-ROS UART transport, so it is always present; a second source feeding `5V` while USB is
connected is a hazard for no benefit.

**5. No LED on any measured net.** An indicator LED across a trigger net loads the very edge
being timestamped. The DevKitC's own power LED is sufficient. If activity indication is ever
wanted it needs a spare GPIO and a firmware change, not a tap onto a measurement.

**6. Group each channel's passives at its own connector.** Electrically the pull-up could sit
anywhere on the net. Placing R(n+3), D(n+3) and Rn together beside Jn is what makes the
pin-by-pin review in §4 and §3 checkable by eye against the physical board, which is the
verification step that DRC cannot perform.

**7. TVS diodes go on the connector side of the series resistor.** That is where the threat
enters. Their capacitance is then charged through 560 Ω (τ ≈ 8 ns), which is negligible
against the optocoupler's own ~26 µs turn-off.
