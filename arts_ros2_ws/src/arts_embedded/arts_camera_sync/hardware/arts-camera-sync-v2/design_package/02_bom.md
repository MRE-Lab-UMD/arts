# arts-camera-sync-v2 — bill of materials

Quantities are for one board with all three channels populated. Distributor numbers are
DigiKey unless noted; equivalents are acceptable **except where a paragraph below says
otherwise**, and those exceptions are the whole point of this document.

| Ref | Qty | Part | Value / spec | Package | Mfr. P/N | DigiKey |
|---|---|---|---|---|---|---|
| U1 | 1 | ESP32-DevKitC-32E | ESP32-WROOM-32E, 4 MB | module, socketed | Espressif `ESP32-DevKitC-32E` | 1965-ESP32-DEVKITC-32E-ND |
| R1–R3 | 3 | resistor, trigger series | **560 Ω, 1 %**, 1/16 W, ≤50 ppm/°C | 0402 | Yageo `RC0402FR-07560RL` | 311-560LRCT-ND |
| R4–R6 | 3 | resistor, strobe pull-up | **1 kΩ, 1 %**, 1/16 W, ≤50 ppm/°C | 0402 | Yageo `RC0402FR-071KL` | 311-1.00KLRCT-ND |
| D1–D6 | 6 | TVS, bidirectional | 5 V bidir., **C ≤ 20 pF** | SOD-323 | Nexperia `PESD5V0S1BA,115` | 1727-3038-1-ND |
| J1–J3 | 3 | screw terminal, 3-pos | 5.08 mm pitch, PCB mount | TH | Phoenix Contact `1729128` (MKDS 1,5/3-5,08) | 277-1667-ND |
| J4, J5 | 2 | header socket, 1×19 | 2.54 mm, tin | TH | Sullins `PPTC191LFBN-RC` | S7017-ND |
| JP1 | 1 | solder jumper, normally closed | — | copper only, no part | — | — |
| TP1–TP8 | 8 | test point | 1.0 mm pad, no part | copper only | — | — |
| — | 1 | PCB | 2 layer, 1.6 mm, HASL lead-free | as-built **80 × 55 mm** (≤100 × 80) | see `03_layout_and_fab.md` | — |
| — | 4 | M3 × 8 standoff + screw | nylon | — | any | — |

The FLIR GPIO pigtail (6-pin Hirose to bare wires, one per camera) ships with the cameras
and is not a board BOM item. Pigtail strain relief is a 3-D-printed case feature, not a
board feature — no cable ties on this list.

---

## The two choices that are not substitutable

### 1 % resistors, and ideally from one reel

This is the highest-value line in the BOM and it costs cents. The breadboard rig measured a
**2.81 µs systematic skew**, of which 0.72 µs was attributable to the strobe-side rise-delay
mismatch (isolated by the fact that the two exposure *widths* differed by exactly that much)
and the remaining ~1.9 µs to something upstream — 560 Ω tolerance changing the optocoupler
drive current, cable length, or camera-internal variation.

The mechanism is direct: the 560 Ω sets the current through the camera's input LED, FLIR
specifies 3.5–7 mA, and optocoupler turn-on delay falls as drive current rises. A 5 % pair
can differ by 10 % in drive current between channels; a 1 % pair by 2 %. Taking parts from
one reel makes the remaining tolerance largely common-mode, and common-mode terms cancel out
of a difference — which is what the published number is.

The same argument applies to the 1 kΩ pull-ups, which govern the strobe's **rising** edge and
therefore the 0.72 µs term directly.

**Do not substitute 5 % carbon film.** That is what the breadboard used and reducing that
term is one of the two reasons this board exists.

### Low-capacitance SOD-323 TVS, not a through-hole equivalent

Through-hole TVS parts in SMA/SMB bodies are easier to hand-solder and have roughly 100×
the junction capacitance — around 1000 pF. Against the 1 kΩ pull-up that is a 1 µs time
constant on the strobe's rising edge, which is 4 % of the optocoupler's own ~26 µs delay and
therefore **not** negligible: it would add a channel-dependent term to the very edge that
dates the photons. The SOD-323 part's ~12 pF gives 12 ns, which is.

Dissipation in R1–R6 is ~17 mW; 0402 is fine. Keep **560 Ω / 1 kΩ, 1 %, one reel per value** —
that line still sets the µs systematic. Order JLCPCB **assembly for R1–R6 and D1–D6**. The
sockets, Phoenix terminals, and the DevKitC stay hand-fit. Do not solve soldering by choosing
a bigger resistor package, and do not add ferrite beads on the camera nets: they would add a
per-channel delay on the dated edges.

---

## Why a TVS at all, on a board this simple

Long unshielded pigtails run on a cart that also carries a monopolar electrosurgical unit,
and they terminate on input-only ESP32 pads that are not 5 V tolerant. The board's purpose is
to produce a number that is trustworthy *in that environment*; discovering ESU-coupled false
captures after a skew baseline has been committed is precisely the silently-wrong-number
outcome being designed out. The parts cost under two dollars total.

The optocouplers inside the cameras already isolate the camera's electronics from ours. The
TVS protects **our** side, which the optocouplers do not.

## Why the ESP32 is socketed rather than reflowed

Two reasons, both from AGENTS.md decision 14. A reproducer should not need reflow to build
this board, and a dead module should be a two-minute swap rather than a rework job. The cost
is two 19-position sockets and about 8 mm of board height.
