# Module: arts_dv_tool_driver — the adapters' disc servos

**Layer 2 (module doc): what the module is.** Status lives in `AGENTS.md` (Phase 4).

## What it is today

`disc_drive` (C++, `src/disc_drive_node.cpp`): one node per PSM, in the arm's namespace,
velocity-commanding the five Dynamixel servos that turn a da Vinci X instrument's input discs.
A port of collaborator A's working Humble setup with A's values — XC330-M288, protocol 2.0,
1 Mbps, IDs 1-5, velocity mode, velocity PI 500 / 1600, profile velocity 0, zero at start-up,
torque off on exit — and these departures:

- Operating Mode is an EEPROM register; A wrote it on every start. `disc_drive` reads it and
  refuses to start unless it is already velocity mode or `allow_mode_write:=true`.
- Every write is checked; position and velocity are read in one sync read; a command older
  than 0.5 s zeroes the velocities; 10 consecutive failed reads stop the node (fault).
- The bus comes from `config/disc_buses.yaml` by FTDI serial (`/dev/serial/by-id`), not by
  `/dev/ttyUSBn` order.

```bash
ros2 launch arts_dv_tool_driver disc_drive.launch.py arm:=psm_left
# or arts_bringup psm_left_tool_actuator:=true, or the GUI row
```

Topics: `<psm>/tool_actuator_target` (JointState velocities, rad/s, `disc_1..disc_5`) in,
`<psm>/tool_actuator_states` out. The gamepad's LB jogs the discs (A's tool-jog mode 0).

`dynamixel_sdk` 4.0.3 (the release A's README names) is built from the tag in
`middle_overlay_ros2_wss/oem_ros2_ws/oem_ros2.repos`, not from the rolling apt package.

## Instrument files (2026-11-02)

`config/instruments/<ref>_<name>.yaml`, one per instrument ARTS owns: 471006 large needle
driver, 471205 fenestrated bipolar forceps, 470179 monopolar curved scissors, 470183 permanent
cautery hook. Each holds the joint set, the disc-from-joint coupling
(`[disc_1, disc_2, disc_4, disc_5] = C [roll, wrist_pitch, wrist_yaw, jaw]`, disc 3 unused),
joint and jaw limits, and the geometry still to be measured (null). Values: collaborator A's
model, sign for sign (the needle driver's pitch/yaw rows are mirrored relative to the others;
the hook uses A's cautery-spatula entry, unverified). Provenance written as "the ARTS team".
Not yet consumed by any node.

## Instrument engagement (homing), 2026-11-02

**Decision (PI, 2026-11-02): collaborator A's scheme, as is.** Its current-capped position
mode is the adapter designer's safe mode, which A adopted; ARTS keeps their work rather than
change it under them. So the engagement makes A's Operating Mode switches (EEPROM): extended
position (4) for the start move, current-based position (5) for the spin and for the limits,
velocity (1) after. Each write is logged with a per-run count; one that would not change the
mode is skipped.

`include/arts_dv_tool_driver/engage_sequence.hpp` ports A's `tool_homing_node` (intake commit
`5f25078`) value for value: start counts `{100, 5000, 2100, 1700, 1800}` at profile velocity
150; spin discs 1, 4, 5, 2 at profile velocity 22, goal current 45/50 mA, stall at 40/45 mA or
< 4 counts per tick for 4 ticks; limits at 100 mA (disc 1 stall 85, disc 4 stall 67 at profile
velocity 20), centred within 15 counts; jaw with disc 2 in freedrive, disc 5 out at 60 mA
(stall 48 mA, profile velocity 15), back at 150 mA pushing disc 2, both centred within 45
counts; velocity mode at goal current 1193. The centred counts become the joints' zero. The one
change to the flow: A ran spin and limits from one joystick button; `disc_drive` makes each
moving step its own service, so the operator gives a go before each:

```bash
ros2 launch arts_dv_tool_driver disc_drive.launch.py arm:=psm_left
ros2 service call /psm_left/disc_drive/go_to_start std_srvs/srv/Trigger   # no instrument
#   ... fit the instrument ...
ros2 service call /psm_left/disc_drive/spin std_srvs/srv/Trigger
ros2 service call /psm_left/disc_drive/find_limits std_srvs/srv/Trigger   # >= 5 s later
ros2 topic echo /psm_left/disc_drive/engage_state                         # phase
ros2 service call /psm_left/disc_drive/stop std_srvs/srv/Trigger          # torque off, final
```

**Tick period.** A's stall rules count ticks of its homing loop, whose 10 ms timer callback
makes 12 single serial reads, so A's real tick is the callback's duration. On the simulated
adapter in `test/engage_sequence_test.cpp` (5 checks pass; its timing check is a failure
case), ticks of 8 ms or less lock the discs in short of their stops and 10 ms mis-centres disc
4; 12 to 100 ms home correctly. So `engage_tick_period` must be A's real period, measured by
`test/a_tick_period_probe.py` (read-only) on each bus. Until it is set, the services refuse.
MEASURED 2026-11-02, no instruments fitted: median **12.00 ms** on both buses (1.0 ms per
read: one USB frame each at the 1 ms latency timer), so A's tick is 12 ms, set per bus in
`config/disc_buses.yaml`; during engagement the bus loop runs at exactly that period. Margin:
the simulation homes from 10.5 ms. The first run on each bus had one failed read / one serial
exception ("multiple access on port?") and clean repeats; ModemManager, which probes serial
ports, is a suspect, not verified.

First adapter run, 2026-11-02, psm_left, no instrument: `go_to_start` wrote mode 1 -> 4 on
all five servos (5 EEPROM writes, logged), moved every disc to A's start counts (read back
101 / 4999 / 2099 / 1699 / 1800 for 100 / 5000 / 2100 / 1700 / 1800; holding at 11-22 mA),
reached `wait_for_tool` after 5 s; `stop` turned the torque off. The servos stay in mode 4, so
the next `disc_drive` start needs `allow_mode_write:=true` (one extra 4 -> 1 write per servo
that A's node, which writes mode 4 directly at start, does not make).

First `spin`, 2026-11-02, psm_left, large needle driver 471006 fitted: FAILED SAFE. All four
discs "locked in" within 17 ms at their start counts without turning; torque off, nothing
moved. Cause (ARTS's node, not A's scheme): the spin request holds the bus through A's two
50 ms mode-switch pauses, and the loop then caught up with back-to-back ticks, which read as
"no movement for 4 ticks". Fixed twice over: the loop no longer catches up, and the sequence
ignores a tick less than 0.9 periods after the last one. New check `BurstTicksAreIgnored`
(6 checks pass), seen failing with the guard disabled.

Second `spin` with the needle driver (ticks correctly 12 ms apart): all four discs again
"locked in" after 4 ticks, ~3 counts each; torque off. Dry run with NO instrument, recording
position and present current every 12 ms: a FREE disc also "locks in". From a standing start
disc 1 moves 0, 1, 2, 4 counts in its first four ticks (accelerating) while its current
reaches 42 mA against the 40 mA stall threshold, so both of A's stall tests fire before the
disc is up to speed (at full speed, 344 counts/s is 4.1 counts per 12 ms tick, on the 4-count
threshold). So A's automatic spin, at A's values and the measured 12 ms bus tick, cannot tell
a free disc from an engaged one. The PI reports that A engages instruments MANUALLY with the
gamepad, not with this routine, so the routine is unproven on this hardware; the adapter
designer may have a working one. The simulated adapter missed this: it has no acceleration
and no current while turning freely (10 mA).

**Manual engagement (A's practice), 2026-11-02.** `disc_drive` in velocity mode plus the
gamepad jog (`arts_bringup psm_left_tool_actuator:=true gamepad:=true`): hold LB, turn each
disc until it drops into the instrument, align, then
`ros2 service call /<psm>/disc_drive/set_zero std_srvs/srv/Trigger` sets the joint zero at
the present counts (velocity phases only; no motion, no mode write).

The zero is VISUAL, not measured: wrist straight in line with the shaft (pitch, yaw), jaw
just closed (contact), roll as fitted; a few degrees of uncertainty. Centre-of-range homing
needs current-capped stalls, which the velocity mode cannot do without load on the cables;
the adapter designer's routine, when available, replaces it. First one: 2026-11-02, psm_left,
large needle driver 471006, engaged by hand (gamepad), zero at disc counts
[570, 1476, 2100, 3637, 1506]; holding 14-32 mA.

## Automatic homing (the adapter designer's method), 2026-11-04

Step 1 of the instrument zero (step 2 is a gamepad touch-up, decision below). `tool_homing` (`src/homing/`, launch `tool_homing.launch.py
arm:=<arm> tool:=forceps|scissors|hook|needledriver`) is the adapter designer's node, ported with its algorithm
and calibration unchanged (intake notes: `$ARTS_LOCAL/code_intake/tool_homing/NOTES.md`). From
motor current alone: roll and pitch go to the midpoint of their mechanical limits, then M5 is
centered (load step in its opening sweep, or its engaged start) and M2 closes until it touches
M5. Home = wrist straight, jaws closed, centered. Per-tool values in `config/homing/<tool>.txt`
(forceps measured by the designer; scissors provisional), per-adapter start positions and jaw
offsets keyed by the adapter's FTDI serial, because they follow the adapter hardware (the two
adapters were swapped between the arms on 2026-11-04 ~10:50).

Sequence: launch (the discs go to their start positions, instrument OFF) -> fit the instrument
-> gamepad A (Y stops, torque off; the gamepad is the jaw moves' deadman) -> COMPLETE writes
`$ARTS_LOCAL/tool_homing_logs/home_<serial>.yaml` -> Ctrl+C -> `disc_drive.launch.py
zero_from_homing:=true`, which takes those counts as the joint zero. The counts survive mode
changes and restarts while the servos stay powered (mode 5 -> 1 -> 5, torque off, unchanged,
2026-11-04); disc_drive refuses if any disc is > 300 ticks from its homed count (a power cycle
drops multi-turn counts by whole turns). A's engagement services are disabled
(`engage_tick_period: 0`): their measured tick period was for 1 Mbps.

Checked without motion: builds in the clean chain; refuses before any motor moves on an
unknown tool, an unknown adapter, and (disc_drive) a missing home file.

**Update, 2026-11-04 evening** (the designer's afternoon of bench work, re-ported the same way):
two more tools, `hook` (470183; no jaws, so its one jaw disc M5 is homed like pitch: limits ->
midpoint) and `needledriver` (471006; `slow_jaws`, `rp.pitch_fast false`: fast pitch on this
stiff tool gave false limits); start positions changed on both adapters (FTBEO5G1 M4 3748 ->
5796; FTBEQBRA M2 7048 -> 9096, M5 4872 -> 2824); a strict tool-file check (a missing,
unknown or out-of-range key refuses); his learn-a-tool mode (`calibrate:=true`, which rewrites
the tool file and `config/homing/calibration/<tool>_history.csv`, so point `config_dir:=` at
the source tree). The hook's completion now also writes the home file. The launch offers each
tool only on the adapters he validated it on, as his GUI does:

| Tool | FTBEO5G1 (psm_right now) | FTBEQBRA (psm_left now) |
|---|---|---|
| forceps 471205 | validated, but M4 start changed since: re-check | validated, M2/M5 starts changed since: re-check |
| scissors 470179 | validated | not offered (pitch range shorter than the file) |
| hook 470183 | validated | validated |
| needledriver 471006 | being tuned; calibrated jaw-angle snaps never run | validated |

disc_drive refuses a home file homed as another instrument (tool -> instrument ref map in
`disc_drive.launch.py`). Re-checked without motion: all four tool files pass the strict check
on both adapters (ports pointed at a missing device: the node stops at "not connected"); a
missing key refuses; scissors on FTBEQBRA, an unknown tool and a forceps zero for the needle
driver refuse.

**First real homings, 2026-11-04 night** (`docs/benchmarks/tool_homing_hook_psm_left_20261104.md`):
cautery hook, psm_left (FTBEQBRA; the swap checked by hand), 10 refits, all COMPLETE in ~41 s.
Roll and M5 repeat in counts (3 and 21 ticks); pitch counts drift up every fit (3826 -> 4292,
both limits together) while the physical home repeats (photos, 0.52° std). So the motor-to-
instrument relation of disc 4 shifts between fits and the homing absorbs it. Hypothesis, not
verified: the disc slips at the pitch push stop (limit 2 is a push stop in every run); if it
can also slip in use, the zero goes stale after homing (ask the designer). Open: needle driver
and forceps on this port; clicking error of the photo method.

**Large needle driver (471006) on psm_left, 2026-11-04 night** (raw: `$ARTS_LOCAL/tool_homing_logs/`
`run_lnd*_psm_left_*.log`, `home_FTBEQBRA_lnd*.yaml`). Tool 1: the first fit STOPPED on the
node's own range check (pitch range 2368 > 2200, push stop: slipped or badly engaged disc; no
home file written); after a refit, three COMPLETE runs (29-48 s, the third after moving jaws and
wrist by hand), home counts within roll 2, pitch 44, M5 3, M2 14 ticks. PI's visual check: pitch
and jaws right, but the jaws sit at the SAME yaw tilt every run (< 5°). Tool 2 (another 471006),
one run: M5 3082 (identical: in the file's "engaged start" mode M5 is placed at a fixed count,
start - calibrated offset, not measured from the tool), M2 8792 (+210 ticks from tool 1, placed
by contact), and NO visible yaw tilt. So the homed zero repeats per instrument but differs from
instrument to instrument by a few degrees; the cause is in the instrument (cable tension, wear:
hypothesis), since adapter, method and M5 count were the same.

**Decision (PI, 2026-11-04): the instrument zero for data collection.** The recorded action is
the absolute tip-pose target through FK that includes the instrument joints (decision 17), so a
zero off by a few degrees biases every recorded tip orientation of that instrument, and data
from swapped instruments would disagree. The zero must be the geometric pose (wrist straight,
jaws centered, closed) on every instrument. Procedure until the freeze and for the paper:
1. `tool_homing` (automatic engagement, range finding at the mechanical stops, centering; its
   range checks also catch a slipped or badly engaged disc);
2. `disc_drive zero_from_homing:=true`, then a gamepad touch-up to the visual center in
   instrument mode, then X (`set_zero`). Log the homed counts and the final zero each time: the
   difference is that instrument's offset, measured for free.
3. Label each physical instrument and record which one each session used.
The touch-up changes ONLY the visibly tilted joints, and the zero is set in that session
(`set_zero`); it is not saved or reused across sessions. Jaw command and grip (PI, 2026-11-05), after dVRK (its large needle driver file: jaw qmin -40
deg "to allow stronger torque", force capped by the jaw's `ftmax` 0.16). Every jaw input sends
ONE unitless command on `<arm>/tool_jaw_target`, clamped to [-10, 90]: 0..90 = opening as a
share of the instrument file's jaw max, 0 = just closed, 0..-10 = grip. disc_drive drives the
jaw toward it at the file's jaw_gain (below 0 the target goes toward the file's jaw min, -10
deg) and caps the grip current: `jaw_grip_cap` x (-command / 10), 1000 at -10, 500 at -5, never
below `jaw_touch_cap` 150 (the designer's jaw move current, so a free jaw still closes); a
closing jaw is held while either jaw disc draws >= the cap. Collaborator A runs uncapped:
velocity mode, no current setting, his opening limit of 0 defined but never applied, so a jaw
blocked by a needle pushes up to the servos' Current Limit (read as 2352 on all ten servos,
2026-11-05; 1193 is only the designer's homing hold). Inputs: VerseGrip lever (`jaw_follower`):
Hall >= 10 -> 0..90 (A's 10..29, ~5 % per step), Hall 10 -> 9 -> 0..-10. The released lever
rested at 7-9 and a squeeze reached 6 in the 2026-11-02 bags (`grip_jaw_check`: 0 only twice),
so the lever has 1-3 grip steps: light or full. Gamepad RT: released -10, first third of the
travel -10 -> 0, the rest 0 -> 90. Not capped: the direct disc jog (disc mode); do not squeeze
jaws there. Checks: unit tests 19/19 (command mapping, clamp of -1000 and NaN, cap; the cap's
opening check seen failing on a mutated cap), mock jaw_follower and gamepad_jog PASS. Not on
the instrument yet. A policy's jaw (`tool_tip_target.jaw`) is not wired to this yet. First right-arm needle-driver homing (FTBEO5G1, the designer's jaw snaps, never
run before): COMPLETE in 38.5 s; PI touch-up then set_zero. Not a reference zero: the touch-up
needed disc mode (instrument mode could not close the jaws past 0 before the -10 deg change) and
moved roll by accident (disc 1 -165 ticks), which the operator could not reproduce. Instrument
mode also makes an accidental roll easy (roll is on the left stick).
Second right-arm homing, 2026-11-05 night (`run_lnd_psm_right_3.log`): COMPLETE in 38.2 s; the
homed jaw sat ~13 deg open (instrument mode at -10 read -9.7 deg with no grip current), closed
by a disc-mode touch-up (discs 2 and 5), then set_zero; a 41 deg yaw jog then bent the wrist
in the direction the model projects into the FLIR view, jaws shut. Earlier that evening a
manual engagement zeroed at power-up (before the discs engaged) read roll 168 / yaw -98 deg
in instrument mode and drove the jaw discs to the grip current, turning the wrist aside: the
zero must follow engagement, which homing guarantees. A yaw that looked mirrored on this
adapter led to a per-adapter yaw sign and then a disc 2-5 swap (both reverted, never
committed); the cause was the badly seated, badly zeroed instrument. Both adapters use the
same plain disc mapping.
Later (imitation learning): a 3D-printed alignment jig per instrument type replaces the visual
touch-up (auto-home, low current, jig on the wrist, set_zero, jig off), so the zero is defined
mechanically rather than by eye. Paper wording: "homed automatically by engagement, range
finding at the mechanical stops, and centering"; no consistency claim.

## Instrument joints (2026-11-02)

`disc_drive` with `instrument_file:=<config/instruments/ file>` (bringup passes the file named
by `<psm>_tool` in `arts_driver/config/instruments.yaml`) runs collaborator A's teleop path,
value for value (`include/arts_dv_tool_driver/instrument_joints.hpp`): joints = coupling^-1 x
disc angles since the zero; disc velocities = coupling x joint velocity targets; each joint
target slowed near its limits by v <= 6 (limit - q), and each jaw blade (yaw +- jaw/2) kept
within the yaw limits, as A does. A joint with no limits in the file (the hook's jaw) is held
at 0. Interim topics `<psm>/tool_joint_states` / `<psm>/tool_joint_target` until the
ros2_control component puts the joints in `joint_states`. The gamepad (hold LB) has A's
tool-jog buttons: A = disc mode, B = instrument-joint mode (roll left stick X, pitch right
stick X, yaw d-pad X at `tool_jog_rad_s`, A's pi rad/s; RT = jaw opening as a position at gain
10 /s), X = set_zero. Checks: `test/instrument_joints_test.cpp` (6, the needle driver's
values; the limit checks seen failing with the limit rule removed) and
`arts_teleop/test/gamepad_jog_mock_test.py` [8]-[10] (all 17 checks pass). The coupling
itself (signs, ratios) is A's.

**First coupling check, 2026-11-02, psm_left, large needle driver 471006** (engaged by hand,
visual zero at disc counts [685, 1438, 2100, 3622, 1569], gamepad instrument mode at 0.5 rad/s;
bag `$ARTS_LOCAL/bench_bags/lnd_coupling_check_20261102`): the PI moved roll, pitch, yaw and
jaw one at a time and saw each move alone, in the expected sense (visual verdict: A's coupling
is right for this instrument). From the bag, 33 single-joint segments: roll 29-35 deg per
push, pitch 28-57, yaw 26-61, jaw 0 to 85 deg and back repeatedly, stopping at 90.5 deg on its
90 deg limit; the other joints moved at most 0.9 deg during any segment. These are the
driver's computed joints, so the bag shows clean tracking; the physical decoupling is the
visual check.

**Second coupling check, 2026-11-02, psm_right, fenestrated bipolar forceps 471205** (same
procedure; visual zero at disc counts [4297, 587, 5446, -4602, 5246]; bag
`$ARTS_LOCAL/bench_bags/forceps_coupling_check_20261102`): PI verdict, each joint moves alone
in the expected sense. 51 single-joint segments after the zero: roll up to 98 deg per push,
pitch up to 66 (limit 70), yaw up to 49, jaw 0 to 90 deg and back; the other joints moved at
most 1.0 deg. (The session's raw range includes the disc jogging before the zero.) Both
arms' instruments for the paper are now checked; the scissors and hook files remain A's
values, unchecked.

**The zero is not repeatable (limitation, PI 2026-11-02).** Collaborator A zeroes the same
way: the disc counts when its node starts, re-settable with the gamepad X button. A visual zero
differs by a few degrees between sessions and instruments, which teleoperation tolerates
(the operator closes the loop by eye) but imitation learning does not: a policy trained on
recorded joint angles sees a drifting offset. Automatic homing that holds across instruments
of different cable tension and wear is a research problem in itself and is NOT attempted
before the ISMR deadline. The paper states this in its limitations (one sentence: automatic
homing / zeroing is under development), not hidden.

## Jaw from the Haply grip lever (2026-11-02)

`arts_teleop/scripts/jaw_follower.py`: collaborator A's grasp path, value for value. Hall
value -> opening fraction clamp((h - 10) / (29 - 10), 0, 1) (A's range, 1 = open) x the
instrument's jaw limit, followed at the file's `jaw_gain` (A's grasper gain: needle driver 10
/s, forceps 5 /s, added to the instrument files), published at 100 Hz on
`<psm>/tool_joint_target` (roll/pitch/yaw 0). The bridge publishes the pen only on change, so
the last grip value is used while the bridge's 500 Hz cursor is fresh. `gate:=teleop` follows
only while the teleop follower is engaged (A holds the jaw with the clutch out); `gate:=always`
for checking the lever alone. `test/jaw_follower_mock_test.py`: 7 checks pass, including the
stale-bridge stop and the not-engaged hold.

First run with the pen, 2026-11-02, psm_right, forceps 471205, right VerseGrip 1237
(`gate:=always`, bag `$ARTS_LOCAL/bench_bags/grip_jaw_check_20261102`): the PI's lever reads
Hall 0 closed to 29 fully open, so A's 10..29 holds as is (29 = A's open point; 0..10 is a
dead band that keeps a not-quite-closed lever's jaw shut). Jaw 1.572 rad at Hall 29, 0.73-0.83
rad at 20-22. PI: easier to control than the gamepad trigger. In teleop the jaw jumps to the
lever's position at engagement (A's grasp is absolute), so close the lever before the pedal.

**First teleop with the instrument, 2026-11-02, psm_left, needle driver** (`run_teleop_real.sh`
with `ARTS_TOOL`, two 10 min runs, bags `$ARTS_LOCAL/teleop_left_lnd_20261102`,
`..._run2_20261102`): the arm followed the handle and the jaw the lever. PI: the jaw lags a
fast lever. Measured, both runs: hall->jaw lag 155 ms (cross-correlation). Budget: (1) the Hall
value CHANGES only every 78-100 ms (median 86-102 ms; ~11 Hz) although pen messages arrive
every 4 ms, and 60 % of run 2's changes skip 3+ steps; a full squeeze took 73 ms, inside one
update, so short open-close motions are never seen; resolution 4.7 deg of jaw per Hall step
(integers 9..29). This is the grip add-on / pen firmware (a question for collaborator A and
Haply), not the bridge. (2) A's jaw gain 10 /s: a 100 ms time constant. (3) The discs: run 1
was capped by disc_drive's 30 rpm (an ARTS addition; per-disc clipping also bends a coupled
command), fixed by a separate `max_rpm_instrument` (75 rpm, below the servos' Velocity Limit
350 = 80 rpm, read 2026-11-02) applied by scaling all discs together; run 2 peaked at 2.85 rad/s
per disc, short of the cap, because the command is short-lived.

**Grip sensing is a hardware limit (limitation, PI 2026-11-02).** A's jaw gain 10 /s is kept
for the submission (raising it would cut ~60 ms of the 155 ms but adds an untested change before
the freeze, and cannot recover squeezes shorter than ~2 Hall updates). The VerseGrip's Hall
reading is not a feature Haply advertises; collaborator A found it and uses it as is. The paper
states the limit in one sentence: ~11 Hz updates in 4.7 deg steps of jaw; masters with faster
grip sensing (e.g. Force Dimension) cost about ten times as much, against ARTS's low-cost aim.

## Measured

2026-11-01, read-only probe (ping + reads, no writes), both buses: ids 1-5 present on each,
model 1240 (XC330-M288), operating mode 1 (velocity), torque off, hardware error 0, input
5.4-5.5 V. So `disc_drive` needs no EEPROM write on these boards.

## Open

- The FTDI latency timer: at the driver's default 16 ms every bus read waits 16 ms and the
  loop ran at 62.5 Hz instead of 200 Hz (2026-11-01). `deploy/modules/dv_tool_driver.sh`
  installs a udev rule setting 1 ms at every plug (A set it by hand): **199.2-199.4 Hz** after
  it, with the plain sync read. SUPERSEDED as the bus rate on 2026-11-03: Fast Sync Read runs
  at 333 Hz (see "Bus rate" below); 199 Hz is history, not the current rate. The rule's first version used `DRIVERS` with `ATTRS{idVendor}` and never matched (udev
  needs every parent key to match one device); seen failing (still 16 ms), then fixed.

## Bus rate (2026-11-03)

The plain sync read costs a fixed 5 ms per cycle (each servo answers in its own packet): 200.0 Hz
whatever `rate_hz` asked (300 / 400 / 500 Hz requested, 5.00 ms +- 0.05-0.07 ms). Protocol 2.0
Fast Sync Read (one combined reply; servo firmware >= 45, ours 53 on all ten) caps at **333 Hz**
(3.0 ms) with the servos' factory Return Delay Time; asking for more runs
flat out and is steadier than asking for exactly 300 Hz (3.334 +- 0.478 ms). 10 min flat out on
psm_left: 200 277 cycles, 333.2 Hz, 3.001 +- 0.056 ms, p99 3.06, max 6.18 ms, one gap over 2x
the period, no failed read; psm_right 30 s: 333.2 Hz, 3.002 +- 0.063 ms. Now the defaults
(`fast_sync_read: true`, `rate_hz: 500`). The 9-joint controller runs at the UR's 500 Hz and
the wrist at the bus's rate (each module at its best measured rate, integrated in one
controller; decision 17).

**2026-11-04: the 333 Hz cap is the Return Delay Time, not the baud** (the "one USB frame each"
explanation above was wrong). `test/bus_rate_probe.cpp` (disc_drive's cycle, torque off, 3000
cycles flat out, 0 failed reads in every run; log in `$ARTS_LOCAL/dv_bus_rate_20261104/`). The
servos were found changed since 2026-11-01, not by ARTS: all ten in mode 5 (current-based
position), and psm_left's Return Delay Time (EEPROM 9) set to 0 while psm_right's is the factory
250 (500 us).

| Bus | Baud | Return delay | Rate | Period |
|---|---|---|---|---|
| psm_right | 1 Mbps | 500 us | 333.6 Hz | 2.997 +- 0.075 ms |
| psm_left | 1 Mbps | 0 | **499.2 Hz** | 2.003 +- 0.066 ms |
| psm_left | 2 Mbps | 0 | **986.6 / 982.7 Hz** (two runs) | 1.014 +- 0.109 ms |

psm_left was put back to 1 Mbps afterwards (498.4 Hz re-measured) because the adapter
designer's tools talk at 1 Mbps. So 500 Hz needs only Return Delay Time 0 on every servo
(an EEPROM write, baud unchanged); 2 Mbps doubles it again. Not yet run through `disc_drive`
itself; the 10-min run that a report needs is still open.

**Adopted 2026-11-04 afternoon** (PI and adapter designer): Return Delay Time 0 and 2 Mbps on all
ten servos. Probe right after: FTBEO5G1 953.0 Hz (1.049 +- 0.212 ms), FTBEQBRA 983.3 Hz
(1.017 +- 0.117 ms), 0 failed reads; psm_right alone at 1 Mbps with delay 0 first gave 498.6 Hz.
`disc_buses.yaml` carries `baud_rate: 2000000`; disc_drive and tool_homing open the bus at it.
Still owed: 10 min through disc_drive at 500 Hz on each module, torque on and discs moving.

## Bench notes, 2026-11-01 (no instrument fitted)

- Bus identity: each FT232H board is fixed under its adapter, so the FTDI serial is the
  adapter's identity. `FTBEO5G1` = psm_left, `FTBEQBRA` = psm_right, verified by jogging disc 1
  on each (A's launch had `ttyUSB0` = right: enumeration order differs per PC).
  Since 2026-11-04 ~10:50 the adapters (with their boards) are swapped between the arms:
  `FTBEQBRA` = psm_left, `FTBEO5G1` = psm_right (PI-reported, `disc_buses.yaml` updated; not
  yet re-verified by a jog). Homing files are keyed by serial and are unaffected.
- First torque-on runs on both adapters: discs 1, 2, 4, 5 jog from the gamepad and stop on
  release; disc 3 held at 0 by the jog mapping, as in A's code.
- Disc layout (psm_right, each ID turned alone at 5 rpm for 3 s, ~90 deg each, the others
  still; viewed at an angle with two motors on the right and three on the left): disc 1 =
  bottom right, disc 2 = top right, disc 3 = bottom left, disc 4 = middle left, disc 5 = top
  left.
- Disc 3 and the instruments (PI's hands-on check): most instruments cannot turn disc 3 at all
  (e.g. fenestrated bipolar forceps, where disc 1 is the two-way shaft roll). SureForm 45:
  turning the shaft turns disc 3, but turning disc 3 does not turn the shaft. Stapler 45: the
  shaft would not rotate by hand (possibly locked). Not investigated further yet; the older
  note "stapler blade on disc 5" is unconfirmed.
- A's homing / disc engagement (current-capped spin, limit search) and the per-instrument
  coupling, as reviewed: port as logic behind an explicit "instrument fitted" step. (The note's
  "without A's uncapped start-up move" is superseded by the PI's 2026-11-02 decision to keep
  A's scheme as is: the start move, in extended position mode with no current cap, runs
  before the instrument is fitted.)
- The ros2_control component and the instrument joints in `<psm>/joint_states` (decision 18).
