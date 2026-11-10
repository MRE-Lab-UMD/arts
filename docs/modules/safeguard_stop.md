# Safeguard stop (ARM HOLD)

> **Doc layer 2 of 4 — module overview.** What this hardware stop is, and why it
> is a safeguard stop rather than an emergency stop. Status belongs in AGENTS.md
> (Phase 4). This file does not own an independent status list.
>
> | | Where | For |
> |---|---|---|
> | Decisions + status | AGENTS.md Phase 4 | why, and what is done |
> | **You are here** | this file | the stop: behavior, wiring logic, PolyScope, paper sentence |
> | Bench procedure | [`safeguard_stop_wiring.md`](safeguard_stop_wiring.md) | build and test, in time order |
> | Parts | [`../bom/bom.csv`](../bom/bom.csv) | list prices and product URLs |

The name of this work is **safeguard_stop**. The blue button legend stays
**ARM HOLD**. “Safety stop” is easy to read as the red emergency stop, and this
box is the yellow safeguard stop.

It has no ROS package. `arts_safety` is a later software logger. It does not
receive these wiring files.

Vendor for every line: **AutomationDirect**. Product links use `/pn/<SKU>`.
**Shopping source of truth is [`../bom/bom.csv`](../bom/bom.csv).** This README is the architecture; the BOM is what you send for a parts review or paste into an ADC cart.

---

## 0. What we are *not* building

A dual-function cell (HOLD + yellow-mushroom E-stop + RESET + tool/energy dump)
was drafted and then dropped. It needed a second `SCR73-280005` ($456) plus
stations, and it assumed a commercial-OR operating model this project does not
have. ARTS is a research cell, not a device for real surgery.

**There is no cell-level Emergency Stop.** Category 1 (power off, brakes, sag)
stays on the **three teach-pendant mushrooms**, one per arm, as Universal Robots
ships them. Someone who needs to kill the cell hits all three pendants.

**There is one ARM HOLD box**, at the PC / supervisor desk. It has a blue latch
and a green RESET. The blue freezes all three arms (Category 2: motors on,
hang in place, no sag). Unlatching the blue is not enough. Tap the green.
Auto-resume on release is the wrong everyday control. There is no second
station: only the person at the supervisor desk can freeze all three arms
together. Each pendant red button still stops that one arm.

EI factory jumpers **stay in** on every controller. Do not land anything on
`EI0`/`EI1`. A cell mushroom is not in the architecture. One Viper only.

---

## 1. What this is, in one paragraph

The patient cart is three UR arms that a teleoperator or a supervisor must be
able to freeze together without dropping gravity compensation. One hardware HOLD
does that: a dual-channel latch into an IDEM Viper `SCR73` (monitored manual
start), whose six N.O. contacts break each arm’s Safeguard Stop pair
(`SI0`/`SI1`). The relay stays dropped until the HOLD button is out
**and** RESET is tapped. The function is not a ROS topic. It does not replace the pendant
E-stops, and it does not dump the
da Vinci disc-drive 24 V or the ESU forward path — those stay on their own
power domains until a later, separate decision.

---

## 2. Locked choices (do not re-litigate without a reason)

1. **Safeguard Stop for the cell freeze. Pendant Emergency Stop for a real
   kill.** HOLD uses `SI0`/`SI1` only. UR E-stop drops gravity compensation;
   that is the wrong everyday control, and a *cell* E-stop that does the same
   to all three arms at once is not worth a second $456 relay on a lab bench.
2. **One HOLD+RESET station, at the supervisor desk.** Blue latch + green
   momentary. No surgeon-console station and no cell E-stop station; that
   was decided 2026-10-20 as final for this platform. One Viper. No second
   relay.
3. **HOLD requires a manual reset, same pattern as E-stop.** Press blue →
   freeze, stay frozen until RESET. Wire the Viper for **S22 monitored manual
   start**, not S21 automatic. On each arm, PolyScope
   `config_in[0], config_in[1]` is **Unassigned** (there is no menu item
   named Automatic). The green tap is still required before the arm will
   start, because the SI jumpers are out and K1/K2 stay off until that tap.
4. **One IDEM Viper `SCR73-280005`.** Six of seven N.O. contacts go to the three
   dual-channel SI pairs. UR’s own rule is that more than two machines sharing
   a stop needs a safety controller; daisy-chaining three SI loops is not the
   design. A cheaper 3-contact relay cannot fan out to three dual-channel arms.
5. **Keep the External Control URCap** as the motion path (`headless_mode:=false`).
   Remote Control on the two UR5e is still useful for ordinary recoveries; it is
   no longer required as “the thing that brake-releases after a cell E-stop.”
   The CB3 has no Local/Remote setting.
6. **Tools and ESU are *not* on this HOLD.** See §3. Pendant E-stop also does
   not dump them (it is per-arm, joints only). That residual is accepted for
   this bench. The unused 7th contact can later open a MOTION-ENABLE into the
   tool ESP32s without buying another relay.
7. **No ROS-only kill** as the cart freeze. `arts_safety` may later *log* the
   relay auxiliary contact. It must not be the path that stops hardware.

---

## 3. Tools, ESU, and the pendant E-stops (residual, accepted)

A disc-drive Dynamixel bus is a separate power domain from the UR joints.
Pressing a pendant mushroom kills **that arm’s** joints and leaves its wrist
motors live. HOLD as specified here also leaves the wrists live (torque on).
The ESU-TAP forward path is likewise untouched. The ESU (DRE ASG-300) has no external stop
input to wire into, unlike the UR control box, so its stop is its own power switch; energy
otherwise ends when the pedal is released, and software energy ends by itself 0.5 s after the
computer stops renewing it (the board's lease; measured 0.506 s, `docs/modules/esu.md`).

That is the residual this lab cell accepts, in exchange for not buying a
Category 1 cell dump. Mitigations that cost nothing now:

- Train: freeze = the blue HOLD at the PC desk; resume = blue out, then the green
  RESET; kill = **all three** pendant mushrooms, not one.
- Leave the SCR73’s 7th N.O. unwired but labelled. When `arts_dv_tool_driver`
  exists, that contact can become MOTION-ENABLE (torque stays on, last disc
  setpoints held). Do **not** dump tool 24 V on HOLD — the wrists can flop on
  their own springs.
- Haply / GOOVIS / the PC stay off this rail. They are not patient-side actuators.

Do not quietly re-introduce a cell E-stop “for the tools” later without a new
parts decision. That is the $456 path that was just declined.

---

## 4. Parts

**Parts are rows in [`../bom/bom.csv`](../bom/bom.csv)** (category `safety`),
AutomationDirect list prices and product URLs. Total **$783.40** at list
price, 2026-09.

- 1× `SCR73-280005` (HOLD, **S22 monitored manual start**)
- 1× station: `SA106-40SL` + blue `GCX3194-24L` + 1 pack of 2 NC `ECX1030-2`
  + `ECX1670-B16` + green `GCX3102` + `ECX1670-B38`
- 1× `PSL-24-060` + `H10086HF-6P-P10` + `DIN8-P10`
- 50 ft `PMTC-18U-4BK-1`: two supervisor-desk runs, three short robot
  cables, supply jumpers. Do not splice.
- 20 ft `H11131803-1` (18 AWG, black / white / green) plus 1× `BRY5266NP`
  NEMA 5-15P plug. Black → `PSL-24-060` **L**, white → **N**, green taped.
  The supply has no ground screw.

Do **not** buy a second `SCR73`, any `ES-P-230001`, extra NO pack `ECX1040-2`
(the green button already has 1 NO), `SEU31-280007`, fiberglass
`CF1PB22`/`CF2PB22`, or 51 mm `SA100SL`.

Confirm S22 manual-start pins from
https://cdn.automationdirect.com/static/manuals/safetyrelays/102581_viper.pdf
on the bench; do not copy them from memory.

---

## 5. Wiring logic (what you will land)

Keep the two safety **channels as two wires** all the way. Never jumper SI0–SI1.
Each UR box uses **that box’s** 24 V through the relay’s dry contacts. Do not
share CB3 24 V with an e-Series box. Leave every EI jumper in place.

```
PC / supervisor desk
  SA106: blue HOLD + green RESET
         │
         ▼
  Patient-cart large box (H10086…)
  PSL-24-060 ──► A1/A2 of SCR73
  S22 monitored manual start
```

**Buttons into the relay:** the one blue button’s two NC blocks are the two
channels. The green NO is S11–S22. Reset will not take while that blue is
still in.

**Per UR controller (repeat ×3), SI jumpers removed, EI jumpers left in:**

- Robot `24V` → relay contact N → `SI0`
- Robot `24V` → next contact → `SI1`

---

## 6. PolyScope / driver (manual, on each arm)

On **every** controller, Installation → Safety → I/O:

1. Remove **SI** factory jumpers only. **Leave EI jumpers in.**
2. Safeguard Stop stays the yellow SI pair. On each UR5e, set
   `config_in[0], config_in[1]` to **Unassigned**. On the UR5 CB3, disable
   Safeguard Reset (it defaults to input pins 0 and 1; the screen is the
   older Safety I/O page, not the e-Series `config_in` list). PolyScope may
   warn that reset is automatic. Accept that warning. The green button is
   the reset that actually closes SI.
3. Teach pendant mushrooms stay as they are (local to that arm). Train:
   before the first start, blue out and tap the green (the arm will not
   start without that tap); freeze is the blue at the PC desk and the whole
   pendant screen turns yellow; resume is blue out, then the green; a real
   kill is all three pendant E-stops.

e-Series Remote Control is two switches, in this order. PolyScope uses
**Manual** for “programs may be edited” and **Local** for “the pendant is
in charge.” Day to day the arm sat in Automatic + Local, so one tap reached
Remote, and a power cycle while already Remote comes back Remote. Opening
Installation → Safety forces operational Manual, and from that state the
pendant refuses Remote
("To ensure safe usage, Remote Control cannot be enabled from Manual mode.").
The SI wires did not remove Remote.

1. Blue out. Tap the green RESET so K1 and K2 are on and the yellow screen
   is gone.
2. Top right of the pendant: change **Manual** to **Automatic**. Confirm if
   it asks.
3. Then choose **Remote Control**. Settings → System → Remote Control only
   has to be Enabled once; it was already on if this arm was remote before.

Keep the External Control URCap program. Dashboard `play` starts **that**
program. Leave ROS `headless_mode:=false` until an arm has been brought up
by hand.

CB3: no Remote Control setting. Same URCap program, Dashboard where the CB3
dashboard allows.

After HOLD: pop the blue, then tap the green → SI closes, arms
hang then resume the paused program; `controller_stopper` may drop the servo
until `robot_program_running` is true again. Unlatching blue alone must not
resume. RESET while the blue is still in must not resume.

After a pendant E-stop: twist **that** mushroom, brake-release that arm, Play
External Control. The other two arms were not in E-stop unless their pendants
were pressed too.

---

## 7. Bench work, in time order

**Follow [`safeguard_stop_wiring.md`](safeguard_stop_wiring.md).** That file is the session-by-session
bench guide (tools → button box → DIN box → power test with no robots → one
arm → three arms). The list below is only the pass/fail headlines.

**A. DIN box dry, no robots** — safeguard_stop_wiring.md Sessions 0–3

1. Mount PSU + SCR73. Confirm 24 V on A1/A2, LEDs as in the Viper manual,
   **S22 monitored manual start**.
2. Wire HOLD + RESET. Confirm: press HOLD → all six used N.O. open; unlatch
   HOLD → still open; then RESET → close. RESET while HOLD is still in → stays
   open.
3. Confirm the two channels disagree (open only one NC) → relay refuses to
   close even after RESET. That is the dual-channel check.

**B. One UR5e, HOLD**

4. SI jumpers out. One contact pair into SI0/SI1. Press blue → hang, gravity
   compensation on, **no** sag like a pendant E-stop. Unlatch → still frozen.
   RESET → resume.
5. Confirm a PolyScope program / External Control both see it.

**C. Three arms**

6. All six SI pairs landed. One HOLD freezes all three. Blue out leaves them
   frozen. Green tap lets them move again.
7. Each cabinet uses its own 24 V through its own relay pair. EI jumpers
   stay in.

**D. Tools / ESU** — not in this build. When those boards exist, decide
MOTION-ENABLE on the 7th contact as a separate edit. Do not dump 24 V on HOLD.

---

## 8. Paper — what may and may not be claimed

The paper text lives in the manuscript repo as a system-architecture
subsection ("Stop Architecture"), not as a numbered contribution and not as
a SOTA row. The evaluation table's Safety line (scripted near-collision, QP
margin) is a different measurement.

Bounds that text must keep:

- No ISO 13849 performance level for the cart as a whole.
- No cell Emergency Stop; none was built.
- The validation is functional (freeze, latch, reset on all three arms), plus
  E8 on ONE arm (left UR5e, 1 kg, 2026-11-01,
  [`e8_arm_hold_stop_20261101.md`](../benchmarks/e8_arm_hold_stop_20261101.md)):
  from the UR's safeguard-stop report, within 1 mm of rest in ~100 ms at 100 and
  250 mm/s, stopping distance 7.8 / 18.4 mm, hold drift ≤ 0.014 mm, no jump on
  reset. At surgical speeds (2026-11-03,
  [`e8_arm_hold_stop_slow_20261103.md`](../benchmarks/e8_arm_hold_stop_slow_20261103.md)):
  0.72 mm at 10 mm/s and 1.46 mm at 20 mm/s; the distance is nearly proportional to
  speed (~73-78 ms of travel). Quote only those, with their scope: the button-to-report
  delay is not measured, and the other arms and payloads are not measured.
- The teleop follower disengages on any UR safety stop until the pedal is pressed
  after it clears (`arts_teleop/teleop_follower.py`), so a hand moved during a
  HOLD is not followed on reset (seen on the real arm in the E8 session).
- No comparison of HOLD behavior to dVRK.
- Nothing from the funder-demo stack.

---

## 9. Where this lives

These pages are the tracked copies:

1. This file — what the stop is.
2. [`safeguard_stop_wiring.md`](safeguard_stop_wiring.md) — bench steps.
3. [`../bom/bom.csv`](../bom/bom.csv) — the part rows. Do not create a second BOM.
4. `AGENTS.md` — the Phase 4 checkbox (HOLD is cart infrastructure, not a
   new phase). One-line headline only. Status lives there.
5. Do **not** put this in `arts_safety`. That package is later software that
   may log the relay’s spare contact. It must not grow a software E-stop
   that bypasses §2.7.
6. `arts_dv_tool_driver` firmware — MOTION-ENABLE only if §3 is revisited.
7. Decision 9 mapping file — HOLD is **not** a VerseGrip or footswitch function.
   Those stay clutch / energy / record marks.
