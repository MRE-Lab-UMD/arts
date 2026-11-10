# Camera sync box — bench wiring guide

> **Doc layer 3 of 4 — substep / bench procedure.** This file tells you what to DO at the
> bench, in time order. It deliberately carries no status and no architectural reasoning; if
> this file and another disagree about either, the other one wins. See the documentation
> contract in AGENTS.md.
>
> | | Where | For |
> |---|---|---|
> | Decisions + **authoritative status** | AGENTS.md decision 14, Stage 5 checklist | why the box exists; what is done |
> | Module overview | [`pilot_flir_camera.md`](pilot_flir_camera.md) Stage 5 | where this fits in the camera module |
> | **You are here** | this file | wiring, meter checks, rungs 5a–5e, symptom table |
> | Measured numbers | [`../benchmarks/camera_sync_skew_breadboard_20260819.md`](../benchmarks/camera_sync_skew_breadboard_20260819.md) | the figures and the exact rig they came from |
>
> Firmware source: `arts_ros2_ws/src/arts_embedded/arts_camera_sync/`.
> Published PCB (first revision): `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/`.

## How to use this guide

It is in three parts, and they are meant to be used differently. **Start at section 1 and read
Part I straight through before touching hardware** — it is short, and every paragraph in it
exists because skipping it produces a failure that looks like a software bug.

| Part | Sections | How to use it |
|---|---|---|
| **I — Understand** | 1–3 | Read once, in order, before anything is connected. |
| **II — Reference** | 4–5 | Nothing to do here. These are the maps you look up *while* building. |
| **III — Do it** | 6–9 | The actual procedure, in order. Section 9 is where hardware gets wired. |
| Appendix | 10 | Symptom table, for when something misbehaves. |

One naming warning: the rungs in section 9 are called **5a–5e** because they are Stage *5* of
`pilot_flir_camera.md`, not subsections of this file. They have nothing to do with section 5
here.

---

# Part I — Understand

## 1. What you are building, in one picture

The ESP32 is a metronome. It emits a short electrical pulse 30 times a second. That pulse
tells both cameras "expose NOW", so the two images are taken at the same instant rather than
whenever each camera's own oscillator felt like it. Each camera then answers on a second wire
— `ExposureActive`, which goes true for exactly as long as its sensor is integrating light —
and the ESP32 timestamps those answers against the same counter that produced the pulse. The
difference between the two answers IS the stereo skew, measured in hardware, with no
computer, operating system, or network in the loop.

Per camera there are exactly three wires:

```
  ESP32-DevKitC (in the screw-terminal breakout)          Blackfly S, rear GPIO connector
  ─────────────────────────────────────────────           ───────────────────────────────

   GPIO 25 ──────[ 560 Ω ]────────────────────────────►   pin 2   BLACK   Line 0, OPTOIN
                                                          (trigger in — this is an LED
                                                           inside the camera, so the
                                                           resistor sets its current)

                       ┌──[ 1 kΩ ]──┐
   3V3 ────────────────┘             ├───────────────◄─   pin 4   WHITE   Line 1, OPTOOUT
                                     │                    (ExposureActive out; cannot
   GPIO 34 ◄─────────────────────────┘                     drive the line itself, hence
                                                           the pull-up. MEASURED: idles
                                                           LOW, RISES while exposing.)

   GND ───────────────────────────────────────────────►   pin 5   BLUE    OPTO_GND
                                                          (the return for BOTH lines above)
```

Camera 2 is the identical three wires to `GPIO 26` / `GPIO 35` / another `GND` terminal, with
its own 560 Ω and its own 1 kΩ. The three remaining pigtail wires — green (pin 1), red
(pin 3), brown (pin 6) — are **cut back or taped off and never connected**. The cameras are
powered over USB3; nothing on this connector supplies or needs power.

## 2. Why there are two resistors, and why they are wired differently

Both resistors above are doing completely different jobs, and confusing the two is the most
common way this build goes wrong.

**The trigger input is a light-emitting diode**, part of an optocoupler that isolates the
camera's electronics from ours. You drive an LED with *current*, not voltage, and the series
resistor is what sets that current. FLIR specifies 3.5–7 mA; 560 Ω lands at about 5.5 mA,
comfortably mid-band. The obvious choice of 1 kΩ gives 3.1 mA by FLIR's own table, *below*
minimum, which triggers the camera sometimes and not others.

**The strobe output needs a pull-up**: internally it is an opto-isolated transistor that can
connect pin 4 to pin 5 but cannot push the line up. With no pull-up resistor the wire floats
and you measure nothing at all.

Which direction it moves when the sensor exposes was MEASURED at rung 5d, and it came out
opposite to what the open-collector description predicts: **the line idles LOW and RISES while
exposing**, so exposure start is the *rising* edge and `ARTS_SYNC_STROBE_ACTIVE_LOW` is `n`.
Do not reason this out from the datasheet — the wrong choice yields a stable, tight, entirely
plausible number that is silently the exposure's *end* (5687 µs instead of 712 µs). The
firmware now captures BOTH edges and publishes the exposure width precisely so a mislabelled
polarity cannot hide: the width must equal the commanded exposure time, and under the wrong
polarity it comes out as the trigger period minus the exposure instead.

Because the rising edge is the one that dates the photons, the pull-up value matters more than
an open-collector reading suggests — it governs that edge's slew. **1 kΩ, not 10 kΩ**: at
10 kΩ, 4–5 % of strobes were missed outright and the edge arrived ~19 µs late. See section 9.

So the two resistors have different *shapes* in the circuit, and this is the mental image to
carry into section 5:

- The 560 Ω sits **in series** — in the middle of the path, ESP32 on one side, camera on the
  other. Current flows *through* it.
- The 1 kΩ sits **off to the side** — from the 3.3 V rail down onto a node that the camera and
  the ESP32 already share. Nothing flows through it except while the transistor pulls down.

### What the screw-terminal breakout does and does not solve

The breakout replaces the DevKitC's fragile 0.1" pin headers with screw terminals, and that is
a real win here: the FLIR pigtail wires are fine stranded copper, which never grips reliably
in a breadboard hole but clamps perfectly under a screw.

It does not solve one thing. A resistor needs **two** separate connection points, and a screw
terminal is a single electrical node — putting the resistor leg and the camera wire into the
*same* `25` terminal shorts the resistor out and drives the camera's opto LED with no current
limit at all. So the resistors live on the breadboard, and the breadboard is bridged to the
breakout with jumper wires. The division of labour that minimises unreliable joints:

- **Screw terminals** take the ESP32 side (`25`/`26`, `34`/`35`, `3V3`, `GND`) and the camera's
  **blue** wire, which goes straight to `GND` with nothing in between.
- **Breadboard** takes the resistors and the camera's **black** and **white** wires.

### This wiring is a bench rig and is NOT the deliverable

Decided 2026-08-18. The breakout-plus-breadboard arrangement exists to bring the design up and
to find out what the real board needs; it never goes on the cart. Three reasons, worst first.
A breadboard contact is a spring clip on tinned stranded wire and it fails *intermittently*,
which in this system does not raise an error — it drops the strobe on some pulses and leaves a
plausible skew computed from the rest, the exact failure shape the whole box exists to
eliminate. The cart also carries a monopolar ESU, and unshielded flying leads hand back the
noise immunity that motivated the opto-isolated line in the first place. And "buy an OONO
breakout and arrange it like this" is not a build instruction anyone can reproduce.

The deliverable is a small two-layer PCB the DevKitC plugs into on headers — socketed
deliberately, so a reproducer needs no reflow and a dead board is a two-minute swap — carrying
the passives, screw terminals for the vendor pigtails, the
loopback as a track rather than a jumper, and the third camera's channel footprinted. That
board is `arts-camera-sync-v2` (KiCad 9) under
`arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/`. Same form
as `arts_esu`, and Gerbers are publishable where a hand-wired rig is not. Pigtail strain
relief is a 3-D-printed case feature (amended 2026-08-27), not hole pairs on the PCB.

MEASUREMENT CONSEQUENCE, and it is the reason the sequencing matters: **no committed skew
baseline may be taken on this rig.** Resistor instances, lead lengths and contact resistance all
move the systematic term — which is the same order as the 2.63 µs systematic offset measured at
rung 5e — so a breadboard baseline would have to be retaken on the final board anyway. Finish
5e here, let its findings feed the schematic (1 kΩ rather than 10 kΩ, active-high strobe, the
379 µs `TriggerDelay` floor and strain relief are all inputs this rig produced), then fabricate
and baseline on the board.

### Breadboard, in 30 seconds

A breadboard's holes are connected in **rows of five** — the five holes on one side of the
centre channel form one electrical node. Holes in the same row are the same wire; the centre
channel separates left from right; the two long strips down the edges are power rails running
the full length of the board.

So a resistor "connects row 2 to row 7" simply by having one leg in each. That is the whole
model, and it gives you a self-checking rule: **anything sharing a row is electrically one
wire, so if two things share a row that should not be joined, that is the bug.**

## 3. Three rules that hold for the whole session

Read these before anything is connected, not after.

**1. Connect blue (OPTO_GND) before anything else, every time.** This is FLIR's own
instruction, and it is not ceremony. Until pin 5 has a return path, energising pin 2 forces
current to find its way out through whatever else is connected, at whatever voltage the
floating input settles to. Order of assembly: blue, then black, then white. Unplug in reverse.

**2. Nothing goes on the green wire, pin 1, ever.** Its non-isolated input is rated to a
maximum high level of 3.6 V — no headroom above the 3.3 V rail, and 5 V destroys it. Upstream's
`follower_example.launch.py` triggers on `Line3`, which *is* this pin; ARTS uses `Line0`
instead, both to remove the footgun and because a monopolar ESU shares this cart and the
optocoupler is the only barrier between its ground currents and the camera's front end. Cut the
green wire back and tape it. Same for red (pin 3, a 3.3 V source we do not need) and brown
(pin 6, camera power ground — tying it to our ground would defeat the isolation we are paying
36 µs of propagation delay for).

**3. Make every wiring change with the ESP32's USB cable out.** Every rung in section 9 that
adds a wire assumes this. It costs seconds and removes a whole class of accident.

---

# Part II — Reference

Nothing here is a step. These are the two maps that section 9 keeps pointing at.

## 4. Terminal map

### Two GPIOs are not labelled by number

Espressif silkscreens two pins by their analog alternate function instead of their GPIO number,
and every DevKitC breakout inherits that. **There is no terminal marked `36` or `39`; looking
for one and concluding the pin is absent is the expected first reaction.**

| Firmware pin | Terminal label | Role |
|---|---|---|
| GPIO 25 | `25` | camera 1 trigger out |
| GPIO 26 | `26` | camera 2 trigger out |
| GPIO 27 | `27` | camera 3 trigger out (reserved) |
| GPIO 34 | `34` | camera 1 strobe in |
| GPIO 35 | `35` | camera 2 strobe in |
| GPIO 36 | **`VP`** (SENSOR_VP) | camera 3 strobe in (reserved) |
| GPIO 39 | **`VN`** (SENSOR_VN) | trigger loopback in |

`VP` and `VN` are the only two aliases in the whole map, and they are adjacent to each other,
so confirm which is which against the strip order before wiring the loopback — swapping them
puts the loopback on the pin camera 3 will eventually want.

### Which terminals get used at all

Only two of the OONO/CZH-Labs board's four strips are involved. Nothing connects to the
`D0…5V` strip or the `4…18` strip.

```
       STRIP 3                              STRIP 4
   ┌────────────┐                       ┌────────────┐
   │    19      │                       │    33      │
   │    21      │                       │    34   ●──┼─→ cam 1 strobe   (via breadboard)
   │    22      │                       │    35   ●──┼─→ cam 2 strobe   (via breadboard)
   │    23      │                       │    VN   ●──┼─→ loopback  = GPIO 39, rung 5b only
   │    25   ●──┼─→ cam 1 trigger       │    VP      │   = GPIO 36, camera 3 someday
   │    26   ●──┼─→ cam 2 trigger       │    EN      │
   │    27      │   camera 3 someday    │    TX      │   ← never touch: USB console
   │    32      │                       │    RX      │   ← never touch: USB console
   │    3V3     │                       │    3V3  ●──┼─→ breadboard "+" rail
   │    GND  ●──┼─→ cam 1 BLUE          │    GND  ●──┼─→ cam 2 BLUE
   └────────────┘                       └────────────┘
```

Both strips carry a `3V3` and a `GND`, and every `GND` terminal on the board is the same node,
so giving each camera's blue wire its own terminal is purely for tidiness — one camera per
strip, which also makes it obvious at a glance that both are grounded.

## 5. The wiring target

### Camera 1 as signal flow

```
  trigger    STRIP 3 "25"  ──jumper──→  row 2  ──[ 560 Ω ]──→  row 7  ──→  BLACK   pin 2
                                                                            (opto LED in)

  strobe     STRIP 4 "3V3" ──jumper──→  + rail ──[ 1 kΩ ]──→  row 12 ──→  WHITE   pin 4
                                                                 │          (open collector)
                                                                 └──jumper──→ STRIP 4 "34"

  ground     STRIP 3 "GND" ─────────────── direct wire, no breadboard ───→  BLUE    pin 5
```

Row 12 has three things in it and row 7 has two — that is the series-versus-off-to-the-side
distinction from section 2, made concrete.

### Breadboard node map — the finished state

Everything listed on one line shares one row, i.e. is one wire.

```
  + rail │ jumper from STRIP 4 "3V3"   ·   1 kΩ #1 leg A   ·   1 kΩ #2 leg A
 ────────┼───────────────────────────────────────────────────────────────────────
  row  2 │ jumper from STRIP 3 "25"    ·   560 Ω #1 leg A
  row  7 │ 560 Ω #1 leg B              ·   cam 1 BLACK (pin 2)
  row 12 │ 1 kΩ #1 leg B              ·   cam 1 WHITE (pin 4)   ·   jumper → "34"
 ────────┼───────────────────────────────────────────────────────────────────────
  row 17 │ jumper from STRIP 3 "26"    ·   560 Ω #2 leg A
  row 22 │ 560 Ω #2 leg B              ·   cam 2 BLACK (pin 2)
  row 27 │ 1 kΩ #2 leg B              ·   cam 2 WHITE (pin 4)   ·   jumper → "35"
 ────────┼───────────────────────────────────────────────────────────────────────
         │ cam 1 BLUE → STRIP 3 "GND"      cam 2 BLUE → STRIP 4 "GND"
         │ (blue wires never touch the breadboard)
         │
         │ LOOPBACK: STRIP 3 "25" ──────── STRIP 4 "VN"    ← PERMANENT, not a test jig
```

**The loopback jumper stays in for good.** It is added at rung 5b and never removed. This is not
optional tidiness: with `CONFIG_ARTS_SYNC_TRIGGER_LOOPBACK=y` the loopback edge is the *only*
caller of `open_record()` in `sync_engine.c`, so it is the reference edge that opens every pulse
record. Pull the jumper and no record is ever opened, nothing reaches the ring buffer, and
SELFTEST reports zero pulses and "no captures" on every channel — **even with both cameras
triggering correctly and publishing images at 30 Hz**. That failure looks like a dead capture
path rather than a missing jumper, which is why it is called out here twice.

(Only if a third camera is ever added does the jumper go away, and then
`ARTS_SYNC_TRIGGER_LOOPBACK=n` must be set in the same change so camera 0's strobe becomes the
reference instead — see section 4 and AGENTS.md decision 14.)

**Do not build this all at once** — section 9 builds it in five increments so that a failure
always has exactly one suspect.

The 5-row gaps are deliberate on two counts: a 1/4 W resistor's body spans them comfortably
without bending the legs sharply, and the 5-row band between the two cameras means a stray leg
cannot bridge camera 1's strobe into camera 2's trigger. **Keeping the cameras in visibly
separate bands is the one layout choice that matters technically**, because crossing a strobe
wire into the wrong capture pin does not fail — it reports a confident skew number that is
actually one camera measured against itself.

Camera 3, if it ever arrives, continues the pattern at rows 32/37/42 with `27` and `VP`, using
a third pair of resistors.

### Continuity sweep — how each wire is verified

Run the rows tagged for a rung at the end of that rung, **USB cable out**, meter on continuity
or the lowest Ω range. This verifies by measurement what looking cannot.

| Rung | Probe A | Probe B | Expect |
|---|---|---|---|
| 5b | `25` terminal | `VN` terminal | ≈0 Ω — the loopback jumper, which stays in from here on |
| 5c | `25` terminal | cam 1 black wire end | ≈560 Ω |
| 5c | `GND` terminal | cam 1 blue wire end | ≈0 Ω |
| 5d | `3V3` terminal | cam 1 white wire end | ≈1 kΩ |
| 5d | `34` terminal | cam 1 white wire end | ≈0 Ω (same node) |
| 5d | `3V3` terminal | `GND` terminal | open — **any low reading is a short; stop** |
| 5e | `26` terminal | cam 2 black wire end | ≈560 Ω |
| 5e | `35` terminal | cam 2 white wire end | ≈0 Ω (same node) |
| 5e | `25` terminal | `26` terminal | open |
| 5e | `34` terminal | `35` terminal | open — low means the two strobes are joined |

The last two rows are the ones that catch the mistake that would otherwise silently corrupt
the measurement rather than break it.

---

# Part III — Do it, in this order

## 6. Step 1 — parts, and verify the resistors

| Qty | Part | Notes |
|---|---|---|
| 1 | ESP32-DevKitC-32E | seated in the screw-terminal breakout |
| 1 | Screw-terminal breakout for DevKitC | every ESP32 pin becomes a labelled screw terminal |
| 2 | FLIR GPIO pigtail, 6-pin Hirose HR10A-7P-6S | one per camera; bare stranded ends |
| 2 | 560 Ω resistor | trigger current limit, one per camera |
| 2 | 1 kΩ resistor | strobe pull-up, one per camera |
| 1 | Breadboard + jumper wires | holds the resistors |
| 1 | Multimeter | verification, not optional — and it replaces the LED in rung 5a |
| (0–1) | LED, any color | **optional.** Nice for rung 5a, but the meter does the same job better. The DevKitC's only onboard LED is a power indicator, so there is nothing on the board to reuse. |

Both resistor values are stock items in the BOJACK 25-value kit (1 Ω … 1 MΩ, 5 %, 1/4 W), so
nothing has to be chained. 1/4 W is ~100× the power these resistors will dissipate.

### Read the label, then verify with the meter

BOJACK ships each value taped to its own labelled strip. Take resistors from the strips marked
**560 Ω** and **1 kΩ** — but still meter them, because loose resistors migrate between strips
and a wrong trigger resistor is the single most confusing failure in this build.

Colour bands: these are 5 % carbon film, so four bands — two digits, a multiplier, then a gold
tolerance band. **Hold the resistor with the gold band on the right** and read left to right.

| Value | Bands (left → right) | Reads as |
|---|---|---|
| **560 Ω** | green, blue, brown, gold | 5, 6, ×10 = 560 |
| **1 kΩ** | brown, black, red, gold | 1, 0, ×100 = 1 000 |

Two near neighbours to not confuse with 560 Ω: 5.6 kΩ is green-blue-**red**-gold (one
multiplier step up) and 56 Ω would be green-blue-**black**. The multiplier band is the one that
matters; on a small carbon-film body brown and red are genuinely hard to tell apart under warm
light, which is why the meter check follows. Note the 1 kΩ pull-up and 5.6 kΩ share the same
red multiplier band, so the *first* band separates them: brown versus green.

Meter check:

1. Turn the dial to **Ω**. On an auto-ranging meter that is all; on a manual one pick the
   2 kΩ range for both values.
2. Touch one probe to each lead, resistor held **out of the circuit** and lying on the bench.
3. Pinch the probe tips against the leads — do not hold both metal tips *and* both leads with
   bare fingers, or you are measuring your own skin in parallel and the reading drops.
4. Accept: **532–588 Ω** and **950–1050 Ω** (5 % tolerance). A meter reading `1`, `OL`, or
   blank means the probes are not making contact or the range is too low.

### Prepare the wire ends

5 mm of stripped conductor is right for a screw terminal. For the two ends that go into
**breadboard holes** — black and white — twist the strands tightly in one direction into a
single stiff bundle first, and tin them if you have an iron. Fine stranded copper splays inside
a breadboard hole and makes contact that is fine until you nudge the bench. An intermittent
connection here imitates a firmware bug exactly, and you will not suspect the wire.

### If you ever need a value the kit lacks

Two rules, worth knowing because you will need them for other ARTS boards:

- **Series** (end to end, one path): resistances add. 470 Ω + 100 Ω = 570 Ω.
- **Parallel** (both ends joined, two paths): for two equal resistors the result is half. Two
  1 kΩ in parallel = 500 Ω.

For this build, the acceptable window on the trigger resistor is **470–680 Ω** (6.6 mA down to
4.6 mA). All three of 470, 560 and 680 Ω are in the kit individually, so chaining is a last
resort — every extra joint is another intermittent contact.

**The pull-up value is NOT a don't-care, and this guide used to say it was.** The original
text specified 10 kΩ and reasoned that the pull-up governs only a lazy recovery edge, because
an open-collector output was assumed to pull *down* when asserted. Measured 2026-08-18, this
camera does the opposite: Line 1 idles low and RISES at exposure start, so the pull-up governs
the very edge that dates the photons. Going from 10 kΩ to 1 kΩ was measured to:

| | 10 kΩ | 1 kΩ |
|---|---|---|
| Strobe rise delay (camera's own `ExposureTime` minus measured width) | 45.7 µs | 26.8 µs |
| Pulses with no strobe recorded | 4–5 % | **0 of 240** |
| Exposure-width spread | 0.5 µs | 0.15 µs |

The dropped strobes are the reason this is not cosmetic: a slow edge loitering near the input
threshold was being missed outright a few percent of the time. Do not go above 1 kΩ. Going
below buys almost nothing — fitting the two points gives ~2.1 µs per kΩ plus a fixed ~25 µs
that is the optocoupler's own turn-off time and no resistor can touch.

## 7. Step 2 — verify the pigtails, before any wiring

Meter on continuity, one probe on a Hirose pin and the other on each bare end in turn, checked
against the pinout table in [`pilot_flir_camera.md`](pilot_flir_camera.md).

**Verify rather than trust** — pigtails do get built with different colour conventions. But be
clear about what is actually at stake, because overstating it trains people to discount real
warnings. **With a 3.3 V driver and a 560 Ω series resistor on every output, no single
mis-identification destroys hardware.** Pin 1's absolute maximum input-high is 3.6 V, so even
driving it by mistake is within spec from an ESP32; the "5 V destroys the camera" hazard is real
but applies to 5 V boards, which is why ARTS uses Line 0 and why the guide says never to put
5 V on the green wire.

What a mis-identification actually costs here is two things. A trigger that does not work, which
is recoverable and self-evident — the camera simply never fires. And, more insidiously, tying
**brown (pin 6, the camera's own power ground)** to our ground, which shorts across the
optocoupler and silently discards the isolation that is the whole reason Line 0 was chosen over
Line 3. Nothing breaks; the ESU protection is just gone.

So this step is about correctness and isolation, not about saving a camera. It is still worth the
five minutes, and the empirical test at 5c — does the camera actually fire at the pulse rate —
independently confirms the blue and black identification for you.

Then cut back and tape off green (pin 1), red (pin 3) and brown (pin 6) so they cannot touch
anything later.

## 8. Step 3 — flash the firmware

Build modes come from `idf.py menuconfig` → *ARTS camera sync* → *Run mode*. `BLINK` and
`SELFTEST` keep the serial console so you can read output; `MICROROS` hands UART0 to the
micro-ROS transport and the console goes silent by necessity.

Flash from a shell with **no ROS sourced** — ESP-IDF's own Python venv and ROS's `PYTHONPATH`
fight each other, so start clean with `env -i bash -l` if your terminal auto-sources an overlay:

```bash
. ~/arts_local/oem_drivers/esp_idf_env.sh          # NOT ESP-IDF's export.sh — see the README
cd ~/arts/arts_ros2_ws/src/arts_embedded/arts_camera_sync/firmware
idf.py set-target esp32                            # first time only
idf.py menuconfig                                  # pick the run mode
idf.py -p /dev/arts_camera_sync build flash monitor    # ctrl-] exits the monitor
```

`menuconfig` and `build` need no board attached; `flash` and `monitor` do. **Always pass
`-p /dev/arts_camera_sync`** — that stable path comes from the udev rule written by
`deploy/modules/camera_sync.sh host_deps`. Omitting it makes `idf.py` autodetect, which happens
to work with exactly one USB-serial device plugged in and silently picks the wrong one otherwise.

Two things that fail in ways that do not look like their cause:

- **`dialout` group membership only takes effect after a fresh login.** `host_deps` adds you,
  but until you log out and back in (or run `newgrp dialout`), flashing fails with a permission
  error on the port that reads like a bad cable. Check with
  `id -nG | tr ' ' '\n' | grep dialout`.
- If it stalls at `Connecting........`, hold the board's **BOOT** button and release it once
  flashing starts. The DevKitC normally auto-resets into the bootloader by itself, so needing
  this is the exception.

Confirm the run mode landed before flashing — the choice is easy to leave unsaved:

```bash
grep ARTS_SYNC sdkconfig
```

Full build notes, including where the toolchain lives and why it is not in `~/.espressif`, are
in `arts_ros2_ws/src/arts_embedded/arts_camera_sync/README.md`.

For the next section, flash **`BLINK`** first.

## 9. Step 4 — build and bring up, rung by rung

Each rung adds one block of the section 5 node map and either passes or tells you which wire is
wrong. Do not skip ahead; **no camera is connected until the pulse has been proven on the
meter.** Overview of what each rung adds:

| Rung | What gets added | Firmware |
|---|---|---|
| 5a | nothing — meter straight onto the `25` terminal | `BLINK` |
| 5b | one jumper, `25` → `VN` — **stays in for every later rung** | `SELFTEST` |
| 5c | camera 1 rows 2 and 7 — blue first, then black | `SELFTEST` |
| 5d | camera 1 row 12 and the `+` rail | `SELFTEST` |
| 5e | camera 2, rows 17/22/27 | `SELFTEST`, then `MICROROS` |

### 5a — the pulse exists, no cameras

Prove the ESP32 drives the pin before anything is attached to it. **A multimeter is the
primary instrument here and no LED is required.**

*With the meter (preferred):* black probe in a `GND` terminal, red probe on `25`, dial to
**DC volts** (20 V range on a manual meter). The `BLINK` build is a 2 Hz square wave at 50 %
duty. That is a quantitative check — it confirms the logic level, not just that something is
happening.

**Two different readings both count as a pass**, and the second one is the one that gets
misread as a failure. A meter that tracks quickly shows the display jumping between roughly
0 V and 3.3 V, sometimes landing in between because a handheld DMM samples only 2–3 times a
second against a 2 Hz wave. A meter that averages instead shows a **steady ~1.65 V** — exactly
half of 3.3 V, which is the mean of a 50 % duty square wave and is equally conclusive. What
fails is a rock-steady 0 V or a rock-steady 3.3 V: either means the pin is not toggling.

The firmware prints its banner once and then loops silently, so no further console output is
expected during this rung. Silence is not a hang.

**THE SAME METER CHECK READS ~10 mV ON THE SELFTEST BUILD, AND THAT IS ALSO A PASS.** This
catches people out, because the number looks like a dead pin (observed 2026-08-18). The reading
is a duty-cycle average and the two builds have wildly different duty cycles:

| Build | Pulse train on GPIO 25 | Duty | Meter reads |
|---|---|---|---|
| `BLINK` (rung 5a) | 2 Hz square wave | 50 % | ~1.65 V, or jumping 0 ↔ 3.3 V |
| `SELFTEST` / `MICROROS` (5b onward) | 100 µs pulse every 33 333 µs | 0.3 % | **~10 mV** |

3.3 V × (100 µs / 33 333 µs) ≈ 9.9 mV, so anything in the 8–20 mV range is the expected
answer — a handheld DMM has several mV of offset at the bottom of its range, and 16.4 mV was
measured on a known-good board. What would indicate a real fault is a rock-steady 0.0 mV (pin
not toggling) or 3.3 V (stuck high).

Do not try to improve this measurement. A DMM cannot resolve a 0.3 % duty cycle meaningfully;
it is a sanity check that the pin is alive, and the real verification is the SELFTEST printout
(rung 5b) or the camera actually following the pulses (rung 5c).

*With an LED, if you have one:* `25` → 560 Ω → LED long leg (anode); LED short leg (cathode) →
`GND`. The LED occupies the exact wiring position a camera will later, which is the small extra
thing it buys you.

Either way, do **not** judge this rung on the `SELFTEST` build. Its real 100 µs pulse at 30 Hz
is 0.3 % duty, so an LED looks dark and a meter reads the average — about 10 mV — both of which
look like a dead pin. That is precisely why `BLINK` exists: production timing is invisible to
slow instruments and proves nothing.

*Passes when:* the meter swings 0 ↔ 3.3 V (or the LED blinks) in `BLINK` mode.
*If not:* wrong terminal, an LED in backwards (it conducts one way only), or the board is not
running — check the console banner in `idf.py monitor`.

### 5b — loopback, still no cameras

USB out, then jumper `25` to **`VN`** — there is no terminal marked `39`, per section 4. **This
jumper is permanent; do not remove it at any later rung** (see the note under the node map for
what happens if you do). Flash `SELFTEST` and watch the report it prints every 2 s.

Measured on this bench 2026-08-18, with no cameras attached:

```
I arts_sync: capture running: 80000000 Hz (12.500 ns/tick), 2 camera(s), loopback on

--- pulses 419  dropped-records 0  12.500 ns/tick
  trigger period         n=418    mean 33333.000 us   min 33333.000   max 33333.000   spread 0.000
  cam0 trigger->exposure n=419    mean   100.000 us   min   100.000   max   100.000   spread 0.000
  cam1 trigger->exposure n=419    mean   100.000 us   min   100.000   max   100.000   spread 0.000
  |cam0-cam1| SKEW       n=419    mean     0.000 us   min     0.000   max     0.000   spread 0.000
```

Three things in that output are worth checking separately. **`12.500 ns/tick`** confirms the
capture clock is the 80 MHz APB clock as designed — if it says 6.250, every timing number the box
ever reports is off by exactly 2× and nothing else would look wrong (AGENTS.md decision 14).
**`pulses` climbing with `dropped-records 0`** proves the loopback capture path works end to end.
**`spread` on the trigger period** is the trigger's own jitter, measured in hardware: 0.000 µs
means below one 12.5 ns tick, which is the floor this instrument can resolve.

`mean 33333.000 us` against an ideal 33 333.33 µs is not an error. The MCPWM timer runs at 1 MHz,
so the period register holds an integer 33 333 ticks and the true rate is 30.0003 Hz. It is a
quantization of the configured rate, identical on every pulse, and it cancels out of every
difference the box reports.

### The two camera lines will LIE at this rung — read this before believing them

They should read `no captures`, because nothing is wired to `34` and `35` yet. Instead they
report a confident `100.000 µs` with zero spread, and a skew of exactly `0.000 µs`.

**That skew figure is meaningless and must never be quoted.** 100.000 µs is exactly
`CONFIG_ARTS_SYNC_PULSE_WIDTH_US`, which gives the mechanism away: GPIO 34–39 are input-only
**with no internal pull-ups**, so an unwired strobe pin floats at high impedance and capacitively
follows the trigger edges being driven on the physically adjacent pads. The channels were at that
point armed on the falling edge only (`ARTS_SYNC_STROBE_ACTIVE_LOW` still defaulted to `y`, which
rung 5d later disproved), so they saw the *end* of the pulse and reported the pulse width. Two floating pins coupling from the same source agree perfectly, hence
skew 0.000.

The consequence is worth internalising because it outlives this rung: **a broken or disconnected
strobe wire does not read `NO_CAPTURE`, it reads a plausible number.** The 1 kΩ pull-up added at
5d is what fixes this — it holds the line firmly at 3.3 V so coupled charge cannot swing it, and
only the camera's transistor can pull it down. Until that resistor is in, treat every
`trigger->exposure` and `SKEW` figure as noise.

If you want to confirm it is coupling rather than a wiring fault, meter continuity from `25` to
`34` and from `25` to `35`. Both must read open. A short there would produce the identical
symptom, and it would be a real defect.

*Passes when:* `12.500 ns/tick`, `pulses` climbing, `dropped-records 0`, mean period matching the
configured rate, and trigger-period spread at or near 0.000 µs. Ignore the camera lines.

*If not:* `pulses 0` with everything reading `no captures` means the reference edge is not
arriving. Most likely the jumper is in `VP` (GPIO 36) rather than `VN` (GPIO 39) — they are
adjacent, and GPIO 36 has no capture channel in the two-camera build, so a swap reports nothing
at all rather than a wrong number.

### 5c — one camera, trigger only

Leave the 5b loopback jumper in place. Build rows 2 and 7: **blue into STRIP 3 `GND` first**,
then the jumper from `25` to row 2, the 560 Ω from row 2 to row 7, and black into row 7. Nothing
on white yet. Run the 5c rows of the continuity sweep in section 5.

Note that `25` now feeds two things at once — the loopback jumper and the trigger chain. That is
correct and intended; both are loads on the same pad, and 5.5 mA into the camera plus a
high-impedance input is nowhere near the pad's limit.

One powered check before the camera is mated: with `SELFTEST` running, meter `25` against `GND`
on DC volts. It should read ~10 mV (the average of a 0.3 % duty pulse). A steady 3.3 V means the
pad is stuck high and the camera's opto LED would be driven continuously — harmless to the LED,
but the camera would sit permanently triggered and the measurement would be meaningless.

Mate the pigtail to the camera. This is easy to defer without noticing, because every other
rung so far has been ESP32-only — but a camera with `trigger_mode: On` and no cable is
indistinguishable from a broken one.

Then start the driver:

```bash
ros2 launch arts_driver cameras.launch.py trigger:=true enable_right:=false
```

*Passes when:* the driver's status line reads `IN: 30.00 Hz`. **Read `IN`, not `OUT`.** The
driver only publishes when something is subscribed (`camera.cpp:827`), so `OUT: 0.00` with no
subscriber is correct behaviour rather than a fault, and mistaking it for one sends you
debugging a camera that is already working. For the end-to-end check, run
`ros2 topic hz /camera_left/image_raw` in a second terminal and watch `OUT` follow `IN`.

*If not:* `IN: 0.00` with `Acquisition timeout, restarting streaming!` every second is the
no-pulse-arriving signature. Usual causes, in order: the pigtail is not mated, the ESP32 is not
running, or the trigger resistor is really 5.6 kΩ.

PASSED 2026-08-18 on camera 24057131: `IN: 30.00 Hz` flat, matching the firmware's 30 Hz
exactly. Note that both `RisingEdge` and `FallingEdge` pass and the test does not discriminate
between them — a 100 µs pulse presents one of each per period, so either yields 30 Hz. ARTS
keeps `RisingEdge` because it anchors the exposure to the same instant the ESP32's loopback
timestamps; on `FallingEdge` every measured trigger-to-exposure delay would silently carry
`CONFIG_ARTS_SYNC_PULSE_WIDTH_US` inside it and would move whenever that width was retuned.

### 5d — one strobe

Add row 12 and the `+` rail: jumper `3V3` → `+` rail, 1 kΩ from `+` rail to row 12, white into
row 12, and a jumper from row 12 to `34`. Run the 5d rows of the continuity sweep — including
the `3V3`-to-`GND` short test, which is the one that stops the board booting.

`SELFTEST` now prints a trigger-to-exposure delay for camera 0, and this rung is where the
decision-15 question gets answered, because the two possible answers differ by three orders of
magnitude.

Budget the expected number from three terms. The camera's own `TriggerDelay` contributes
**379 µs** — that is the FLOOR on this model, not a tunable default, so it is present whether or
not anyone sets it. The optocoupler contributes ~50–60 µs (18 µs in, 36 µs out). And the sensor's
`ExposureActive` assertion contributes either ~0 (first-row assertion) or a full sensor readout
of ~17 ms (last-row assertion). So:

| Firmware behaviour | Expected reading |
|---|---|
| Asserts at first-row exposure start | hundreds of µs |
| Asserts at last-row exposure start | ~17.4 ms |

MEASURED 2026-08-18 on camera 24057131 with the 1 kΩ pull-up: **711.7 µs**, spread 8.5 µs,
every one of 240 pulses captured — so first-row assertion, and that is now a recorded property
of firmware 1801.0.9.0 rather than a guess. Alongside it, `cam0 exposure WIDTH` read 4977.2 µs
(spread 0.15 µs) against the camera's own reported `ExposureTime` of 5004 µs.

That 26.8 µs shortfall is the strobe's rise delay, and it is worth understanding rather than
ignoring, because it sits inside the 711.7 µs as a bias. It is available for free on every
frame — `ExposureTime` from the `~/<cam>/meta` topic minus the measured width — so it can be
subtracted per camera per frame rather than assumed constant. It is also common-mode, so it
cancels out of the L/R skew the paper reports.

**Read the WIDTH line before you trust the delay line.** It should equal the exposure time
you asked for. If it instead reads roughly the trigger period minus the exposure (~28 ms
here), the firmware has the strobe polarity backwards and the delay above is really a delay
to the exposure's END — which is exactly what happened on the first 5d run, producing a
stable 8.5 µs-spread figure of 5687.7 µs that looked entirely credible.

Record whichever you measure. It cannot be configured — BFS-U3-63S4C firmware 1801.0.9.0 has no
`ExposureActiveMode` node — so the mode becomes a measured fact, which is a stronger claim than a
configured one.

**The 379 µs floor incidentally removes an ambiguity.** The floating-pin artifact of 5b reports
exactly `CONFIG_ARTS_SYNC_PULSE_WIDTH_US` (100.000 µs, zero spread), and neither expected reading
above is anywhere near it, so a `100.000` here unambiguously means the pull-up or the white wire
is not doing its job and you are reading crosstalk rather than the camera.

*Passes when:* the delay is stable pulse to pulse AND is not equal to the configured pulse width.
*If not:* a wandering figure usually means the pull-up is missing (line floating); an absent one
means `LineSource` has not been set to `ExposureActive` on the camera — see the open item in
`pilot_flir_camera.md`.

### 5e — both cameras

Wire camera 2 identically in rows 17/22/27, on `26` / `35` / STRIP 4 `GND`, and run the 5e rows
of the continuity sweep. Confirm the two absolute delays agree to within a few hundred
nanoseconds, then flash `MICROROS`, start the agent, and record `/arts_camera_sync/pulse`
alongside the `~/<cam>/meta` topics.

*Passes when:* skew from `exposure_delay_ticks` and skew from the `camera_time` spread in
`meta` tell the same story. They are independent measurements of the same physical quantity,
so agreement is the acceptance criterion, and the hardware figure is the one the paper quotes.
Commit the baseline under `docs/benchmarks/`.

FIRST HARDWARE SKEW, 2026-08-18, SELFTEST console path only: **mean 3.4 µs, max 10.6 µs,
min 0.000 µs over 720 pulses**, with cam0 at 711.64 µs / width 4977.27 µs and cam1 at
709.01 µs / width 4976.55 µs.

MICROROS BAG CROSS-CHECK, 2026-08-19, still breadboard, still not the committed baseline:
34.5 s of `/arts_camera_sync/pulse` + both `~/meta` topics. Hardware **|cam0−cam1| mean
3.52 µs, max 10.44 µs over 1001 pulses** (systematic 2.81 µs) — same number as SELFTEST.
`camera_time` periods match the 33 333.000 µs trigger; the (L−R) residual after removing
the two-clock offset is std 22.8 µs / 87 µs peak-to-peak, which is the software figure
and is allowed to be larger. Cast ticks to a signed int before subtracting: a `uint32`
wrap reports 53.7 s and looks like a broken bag. `ros2 topic echo` needs
`--qos-reliability best_effort` or it prints nothing against a live 30 Hz publisher.
See `docs/benchmarks/camera_sync_skew_breadboard_20260819.md`.

Two things to know when reading your own numbers against these. The systematic part is the
2.63 µs difference of the means, and the exposure widths tell you how much of it is the
strobe's own rise-delay mismatch — here 0.72 µs of it, leaving ~1.9 µs upstream in the
trigger path. The random part is ±8.6 µs per camera and is almost certainly the sensor's
line period (~17 ms readout ÷ 2048 rows ≈ 8.3 µs), since a camera can only begin exposing on
a line boundary. That means it is irreducible by any wiring or firmware change, and the only
lever on it is a shorter ROI.

**Strain-relieve the pigtails before trusting a run.** The Hirose leads are barely long
enough to reach from the breakout to the breadboard, so a wire can tension itself out of its
hole with nothing looking disturbed. That is how camera 2 first presented: trigger working at
30 Hz, strobe reading `no captures`. Tape the cable to the bench a few centimetres back from
the breadboard so the wire ends carry no load.

---

## 10. Symptom table

| Symptom | Most likely cause |
|---|---|
| Camera publishes nothing at all | No pulses arriving. `trigger_mode: On` with no trigger looks exactly like a dead camera. |
| Triggers intermittently, worse when warm | Trigger current below 3.5 mA — resistor too large, or a loose joint adding resistance. |
| Strobe delay absent or wandering | Pull-up missing, or `LineSource` not set to `ExposureActive`. |
| One camera's delay reads `NO_CAPTURE` | Strobe wire in the wrong capture terminal, or captured on the wrong edge (`ARTS_SYNC_STROBE_ACTIVE_LOW`). **Check first that the wire is still in its hole** — the Hirose leads are short enough to tension themselves out, and the camera keeps triggering perfectly while the strobe goes dead, which reads as a firmware fault. |
| One camera captures, the other reads `no captures` while both trigger at 30 Hz | Meter the failing camera's capture terminal against `GND` with the driver running: steady 3.3 V means the pull-up is intact and nothing is driving the node, so the break is between the breadboard row and the camera; ~0.5 V means the signal is arriving and the problem is on the capture side; ~0 V means the node never reaches the pull-up. |
| Skew is suspiciously, perfectly zero | Either both strobes landed on the same capture pin (one camera measured against itself), or both strobe pins are floating and coupling from the trigger — check whether `trigger->exposure` equals the configured pulse width exactly, which is the floating case. |
| `trigger->exposure` reads exactly `CONFIG_ARTS_SYNC_PULSE_WIDTH_US` with 0.000 spread | The strobe pin is floating, not connected. Input-only GPIOs 34–39 have no internal pull-up, so an unwired pin capacitively follows the trigger's falling edge. Fit the 1 kΩ pull-up. |
| Board will not boot after wiring | Something was landed on GPIO 6–11 (SPI flash) or on a strapping pin (0/2/5/12/15). |
| Console silent but board seems alive | A `MICROROS` build is flashed. UART0 belongs to micro-ROS in that mode; reflash `SELFTEST` to debug. |
