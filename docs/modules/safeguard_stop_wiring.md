# Safeguard stop — wiring and bench test

> **Doc layer 3 of 4 — bench procedure.** Do this at the bench, in time order.
> The design lives in [`safeguard_stop.md`](safeguard_stop.md). Parts live in
> [`../bom/bom.csv`](../bom/bom.csv). Status belongs in AGENTS.md.

Read one session. Finish it. Do not open a robot cabinet until Session 3 passes.

---

## What the finished system does

There is **one** gray button box, at the PC / supervisor desk. It has two buttons:

- **Blue** = ARM HOLD. Press it so it stays in. All three arms freeze in place.
  They do not fall. Press it again so it pops out. The arms **stay frozen**.
- **Green** = RESET. Works only after the blue is out. Tap it (press and let
  go). The arms may move again.

A real emergency is still the **red button on each robot's teach pendant**.
That stops one arm only. ARM HOLD is all three arms, no falling.

---

## Session 0 — Tools, extra bits, unpack

### Tools

| Tool | Why |
|---|---|
| Small flat screwdriver | Screw terminals on the relay and the power supply |
| #2 Phillips screwdriver | DIN-rail screws, box covers, button nuts. A smaller driver chews the rail-screw heads |
| Wire cutter (side cutters) and wire stripper | 18 AWG cable. Not scissors |
| Multimeter (beep / continuity + DC volts) | Prove wires before you add robots |
| Drill and cable glands or rubber grommets | Holes in the large box |
| Marker and tape (or a label printer) | Every cable end gets a name |
| Flashlight | Inside the robot cabinets |

Nice to have: ferrules and a ferrule crimp. The relay terminals take two
ferruled 18 AWG wires under one screw.

### Unpack and check

- [ ] Red relay, printed `SCR-73-i` / `SCR73-280005` (Viper). The black box is the power supply, not the relay.
- [ ] Black DIN power supply `PSL-24-060`
- [ ] Large gray box, about 10 × 8 × 6 in (`H10086…`)
- [ ] DIN-rail kit `DIN8-P10` (two rails, four screws, four washers; you use one rail)
- [ ] One small gray box with **two** 22 mm holes (`SA106-40SL`)
- [ ] One blue latching button `GCX3194-24L` (it stays in when you press)
- [ ] One green momentary button `GCX3102` (it springs back)
- [ ] One pack of NC contact blocks `ECX1030-2` (two blocks)
- [ ] One `OFF-ON` legend plate and one `RESET` legend plate
- [ ] 50 ft reel of `PMTC-18U-4BK-1` (4-wire control cable)
- [ ] Mains cord `H11131803-1` and plug `BRY5266NP` (or an existing black / white / green cord)

Keep the Viper manual in reach:
https://cdn.automationdirect.com/static/manuals/safetyrelays/102581_viper.pdf

### The relay screws you will use

Copy this onto a sticky note and keep it on the box:

```
POWER     A1 = 24 V plus     A2 = 24 V minus

HOLD channel 1     S11  →  blue NC1  →  S12
HOLD channel 2     S13  →  blue NC2  →  S10
RESET              green button between S11 and S22

No wire on S21. S21 is automatic restart.

Robot 1 (first UR5e)   13–14  and  23–24
Robot 2 (second UR5e)  33–34  and  43–44
Robot 3 (camera UR5)   53–54  and  63–64
Leave 73–74 empty. Never use 81–82.
```

"Channel 1 / channel 2" means **two separate wires**, like two locks on one
door. Never join SI0 to SI1. Never join S12 to S10.

### The cable

`PMTC-18U-4BK-1` has **three black wires plus one green**. Each black has a
tiny white number printed on it: **1**, **2**, **3**. The green may carry a
yellow stripe. It is **not** black / white / red. If you cannot read a
number, find the matching end with the meter's beep and tape both ends.

Cut with side cutters, not scissors. Slice only the outer jacket with a
razor. Strip each inner wire about 8 mm with the stripper.

---

## Session 1 — Button box

### 1.1 Blue button: replace its switch

The blue button arrives with one **NO** block on the back (connects only while
pressed). ARM HOLD needs **NC** (connected when the button is out, open when
it stays in).

1. Unclip the stock NO block. Do not use it.
2. Clip the two `ECX1030-2` NC blocks into the two clip slots **side by side**.
   Do not stack one behind the other. A stack is about 85 mm and will not fit
   the 74 mm box.
3. Put a tape tag `NC1` on one block. The other is `NC2`. The tag is a name,
   not insulation.

Ignore any tiny extra lamp screws on the holder. ARM HOLD works without the
LED.

**Meter, continuity mode, one block at a time, probes on that block's two
screws:**

| Blue | Beep? |
|---|---|
| In (stays down) | **No beep** |
| Out (popped up) | **Beep** |

A beep with the button in means that block is still an NO.

### 1.2 Green button

Keep the block that came on `GCX3102`. Resting: no beep. Pressed: beep.
Released: no beep.

### 1.3 Assemble the box

1. Blue in one hole, green in the other.
2. `OFF-ON` plate under the blue nut, overwritten **ARM HOLD**. No red STOP
   legend. `RESET` plate under the green nut.
3. Tighten the 22 mm nuts so the buttons cannot spin.

### 1.4 Two cables, measured on the real route

The blue button has two switches and the green has two screws: six signal
wires. Each cable has four, so the box needs **two cables**, `PC-1` and
`PC-2`, of equal length.

Measure along the floor or cart frame from the supervisor desk to where the
large box will sit on the patient cart, not a straight line through the air.
Add about 1 m of slack. The large box lives on the patient cart beside the
UR controller cabinets, so the three robot cables stay short.

One 50 ft reel covers both `PC` runs, three short robot cables, and the
supply jumpers on a typical cart layout. Check your route before cutting.

**`PC-1`**

| Wire | In the button box | At the relay (Session 3) |
|---|---|---|
| Black **1** | NC1, first screw | S11 |
| Black **2** | NC1, other screw | S12 |
| Black **3** | Green, first screw | S11 |
| Green | taped, unused | taped, unused |

**`PC-2`**

| Wire | In the button box | At the relay (Session 3) |
|---|---|---|
| Black **1** | NC2, first screw | S13 |
| Black **2** | NC2, other screw | S10 |
| Black **3** | Green, other screw | S22 |
| Green | taped, unused | taped, unused |

Knot or gland each cable inside the box so a tug cannot pull the screws.

### 1.5 Check at the loose ends (no power)

Blue out, green at rest:

- `PC-1` black 1 ↔ `PC-1` black 2: **beep**
- `PC-2` black 1 ↔ `PC-2` black 2: **beep**
- `PC-1` black 3 ↔ `PC-2` black 3: **no beep**

Blue in: the first two beeps stop. Green pressed: black 3 ↔ black 3 beeps,
whether or not the blue is in. That last case is correct; the relay, not
the button, refuses a reset while the blue is in.

Close the button box.

---

## Session 2 — Large box

Two parts go inside: the **black** 24 V supply and the **red** Viper relay.
No buttons. The loose grey plastic pieces are outside feet for bolting the
box to the cart later. They are not part of the rail or the circuit.

### 2.1 One rail across the middle

1. Open the cover like a book. Do not pinch the seal.
2. The inner posts are **smooth**. The `DIN8-P10` screws are thread-cutting
   and make their own thread.
3. Use **one** rail. It bridges the open middle from a post on one wall to a
   post on the opposite wall, about halfway up. Not along a wall and not on
   the lowest posts, because cables enter from the bottom.

### 2.2 Layout, then screw

```
   |<-------- rail, about 7 in -------->|
   | empty | black supply | gap | red Viper | empty |
```

The supply takes about 2.2 in, the Viper about 1.8 in, plus a finger gap.
Leave empty rail at both ends. Printed screw names and the lamp row face the
cover.

Before screwing, check: nothing touches a side wall, the top or bottom wall,
or the closed cover. Then, with a #2 Phillips, a washer and screw through a
rail slot into a post at each end. Stop when the washer is flat and the rail
does not rock. Two screws are enough.

To clip a part: hook the top groove over the rail's top lip, press the
bottom until it clicks. To remove: flat screwdriver in the bottom tab, tip
it off.

### 2.3 Drill before wiring the relay

A cable cannot pass a gland once both ends are landed, so drill first, with
the supply and relay out of the box.

1. Drill the wall that faces the **floor**. Not the cover, not the hinge wall.
2. **Six holes:** two for `PC-1` / `PC-2`, one for the mains cord, three for
   the robot cables. Size each hole to its gland; the mains cord is thicker
   than the signal cable.
3. Fit the glands. Pass the loose relay ends of `PC-1` and `PC-2` and the
   stripped end of the mains cord through from outside. A molded plug does
   not fit a gland: fit the plug after, or re-land the cord.

---

## Session 3 — Wire the supply and relay, no robots

Do this on the table, then clip both parts back on the rail. Robot screws
stay empty.

### 3.1 Supply to relay

Cut one 15 cm piece of the cable, strip off the jacket, and use **two of the
black wires** only. Tape the unused black and the green.

- Supply **+V** → relay **A1**
- Supply **−V** → relay **A2**

### 3.2 Mains cord, then 24 V check

Cord **unplugged** while you screw it.

- Black → supply **L**
- White → supply **N**
- Green → tape the bare end. `PSL-24-060` has no ground screw. Do not land
  green on L, N, +V or −V.

At the plug: black to the brass screw, white to the silver screw, green to
the green screw.

Plug in. **L and N are 120 V; do not touch them.** Meter on **DC volts**:
red probe on A1, black probe on A2. Expect about 24 V (23–26 is fine).
POWER lamp on, other lamps off. Unplug before touching any other screw.

### 3.3 The PC cables onto the relay

The relay has two tiers of screws on each side. On the lamp side, the
**taller inner** tier is A1, S11, S21, S22, then 91, 92, 101, 102. The
**shorter outer** tier is 13, 23, 33, 81, then 43, 53, 63, 73. 13 sits next
to A1 but is the shorter screw. On the rail-clip side, the inner tier is
S12, S13, S10, A2 and the outer tier is 14, 24, 34, 82, 44, 54, 64, 74.

- `PC-1` black **1** and `PC-1` black **3** → **S11** (two wires under one screw)
- `PC-1` black **2** → **S12**
- `PC-2` black **1** → **S13**
- `PC-2` black **2** → **S10**
- `PC-2` black **3** → **S22**

No wire on S21. Nothing on the robot screws yet. Never 81 or 82. Green
stays taped. Tug every wire.

### 3.4 Meter, unplugged

Blue out, green at rest: S11 ↔ S12 beep, S13 ↔ S10 beep, S11 ↔ S22 no beep.
Blue in: the first two stop. Green pressed: S11 ↔ S22 beeps, with the blue
in or out.

### 3.5 Lamp test

Blue out. Plug in.

| Lamp | Meaning | Blue out, before green |
|---|---|---|
| POWER | 24 V on A1/A2 | On |
| CH1 | S11–S12 closed | On |
| CH2 | S13–S10 closed | On |
| RESET | S11–S22 closed, only while green is held | Off |
| K1, K2 | Safety contacts closed; arms allowed to move | Off |

1. Hold green: RESET on, K1/K2 still off.
2. Release green: RESET off, **K1 and K2 on**. The reset is the release, not
   the press. Holding green is not a reset.
3. Blue in: CH1, CH2, K1, K2 off at once.
4. Green while blue is in: RESET lights, K1/K2 stay off.
5. Blue out: K1/K2 **stay off**. Popping the blue is not a start.
6. Tap green: K1/K2 on.

### 3.6 Contact check and broken-wire test

Meter on screws 13–14: beep only while K1/K2 are on. All seven pairs
(13-14 … 73-74) behave the same. Tape 73–74; it stays unused.

Broken wire: with K1/K2 on, unplug the cord, unscrew **only** `PC-2` black 2
from **S10**, plug in, blue out, tap green. K1/K2 must **refuse**. Unplug,
restore the wire, plug in, tap green, K1/K2 on. That proves the two
channels are really two.

**Session 3 pass = you may open a robot cabinet.** Unplug the large box
before moving it.

---

## Session 4 — First robot only (one UR5e)

Clear the workspace. Pendant in hand.

### 4.1 The rule

The yellow safety terminal block reads, top to bottom: `24V`, `EI0`, `24V`,
`EI1`, `24V`, `SI0`, `24V`, `SI1`, with white two-pin jumpers.

- Remove **only** the two jumpers bridging `24V`–`SI0` and `24V`–`SI1`.
- **Leave the EI jumpers in.** Without them the arm latches emergency stop.
- Do not touch the grey I/O blocks beside it.
- Each robot uses **its own** 24 V. Never the large-box supply, never another
  robot's.

### 4.2 Robot cable

Cut one cable from the large box into this cabinet with slack. Label both
ends (for example `ARM-L`). All four wires are signals. Flag the green
`SI1 — not earth`; do not land it on a cabinet earth screw.

| Wire | Viper screw | Robot terminal |
|---|---|---|
| Black **1** | **13** | the `24V` beside SI0 |
| Black **2** | **14** | **SI0** |
| Black **3** | **23** | the `24V` beside SI1 |
| Green | **24** | **SI1** |

### 4.3 In the cabinet

Power off. Unplug the robot. Photograph the jumpers first. Pull the two SI
jumpers and bag them inside the door. Land the four wires. Power the robot
and the large box.

### 4.4 PolyScope (UR5e)

**Installation → Safety → I/O** (safety password required):

1. Safeguard stop stays on the yellow SI pair. Do not assign Safeguard to a
   `config_in` row.
2. Set `config_in[0], config_in[1]` to **Unassigned**. There is no menu item
   called Automatic; Unassigned is it. Left on Safeguard Reset, the arm waits
   for a reset on grey terminals this build does not wire, and never starts.
   Apply, and accept the warning that reset is automatic. The green button is
   still required: K1/K2 stay off until it is tapped.
3. Do not change Emergency Stop.

Editing safety settings leaves the arm in operational **Manual**. Remote
Control is refused from Manual ("To ensure safe usage, Remote Control cannot
be enabled from Manual mode."). After the green tap, switch **Manual →
Automatic**, then **Remote**.

### 4.5 Start, then HOLD

The factory jumpers used to hold SI closed. Now **tap the green before you
start the arm**, every session. Starting first gives a safeguard stop.

1. Blue out, tap green, K1/K2 on. Start the arm and move it slowly.
2. Press **blue**. The arm stops and holds. It does not drop. The UR5e
   pendant screen turns **yellow** (safeguard stop), not emergency stop.
3. Pop blue out. The arm stays frozen.
4. Tap green. The yellow clears; press Play if a program was running.

Compare once with the pendant red button: red brakes the arm and needs
re-initialization; blue keeps it stiff.

---

## Session 5 — The other two arms

Same steps, next relay pairs:

| Arm | Viper screws | Robot terminals |
|---|---|---|
| First UR5e | 13–14 → SI0, 23–24 → SI1 | that cabinet's 24V |
| Second UR5e | 33–34 → SI0, 43–44 → SI1 | that cabinet's 24V |
| Camera UR5 (CB3) | 53–54 → SI0, 63–64 → SI1 | that cabinet's 24V |

On the lamp side, 81 sits between 33 and 43; on the other side, 82 sits
between 34 and 44. Never land a wire on 81 or 82. 73–74 stay empty.

**UR5 CB3 PolyScope** is the older screen: **Installation → Safety → I/O**,
turn Safeguard Reset off (it defaults to input pins 0 and 1). There is no
`config_in` list and no Remote Control on the CB3. Its screen may not turn
fully yellow; the pass is that the arm stops, stays up, and reports
safeguard stop.

### Final checks, all three powered

1. Blue in: all three freeze, none drop.
2. Blue out: all three stay frozen.
3. Tap green: all three may move again.

Close the large-box lid.

---

## If something is wrong

| What you see | Likely cause | What to do |
|---|---|---|
| POWER lamp off | No 24 V on A1/A2 | Unplug. Check +V to A1, −V to A2. Meter about 24 V DC. |
| CH1 off, CH2 on (blue out) | Channel 1 open | `PC-1` black 2 not on S12, or NC1 still the stock NO block |
| CH1 and CH2 on, green does nothing | S22 not connected, or holding green | Confirm `PC-2` black 3 on S22. **Tap**, do not hold. |
| K1/K2 on as soon as power is applied | Automatic restart wired | Remove the wire or jumper on S21. |
| Robot in emergency stop after the cabinet was opened | EI jumpers pulled | Put EI jumpers back. Only SI jumpers come out. |
| Robot will not start, says safeguard stop | Green not tapped, blue still in, or `config_in[0,1]` still Safeguard Reset | Blue out, tap green. Set that row to Unassigned. |
| Remote Control refused | Arm in operational Manual | Manual → Automatic, then Remote. |
| One arm freezes, another does not | That arm's pair not landed, or its SI jumpers still in | Meter that pair while pressing blue. |
| Arm **drops** when blue is pressed | Wires on EI, not SI | Stop. Restore EI jumpers. Move wires to SI0/SI1. |

---

## How people use it (print this)

Before the first start of a session, pop the blue out and **tap the green
RESET**. The arms will not start until that tap. To freeze, press the
**blue ARM HOLD**: the arms hang in place and the UR5e pendants turn yellow.
Pop the blue out when ready. The arms will **not** start yet. Look around,
then **press and release the green RESET**. A red pendant button stops only
that one arm and brakes it. In a real emergency press **all three** red
pendant buttons.

---

## Not in this build

- One ARM HOLD station only, at the supervisor desk. No second station.
- No cell emergency stop, no yellow mushroom, no second Viper.
- No wires to the instrument disc-drive motors or the ESU.
- Relay screws 73–74 stay empty.
