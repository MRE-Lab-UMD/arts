# Decision 17 — One position-commanded control hierarchy; teleop and autonomy differ only in the reference source

> **Role: decision record.** Locked 2026-10-28. Amends the pipeline sentence of
> [decision 1](01_orocos_retired.md) (the two rate-grouped managers stand unchanged) and
> settles the architecture half of the AGENTS.md item "Control-law decision from the
> three-way comparison". Implementation status lives in AGENTS.md, Phases 1–2.

## The decision

Every arm is always driven by **position commands**, through one hierarchy. What changes
between teleoperation, autonomy and benchmarking is only which source writes the reference.

```
reference source (exactly one active per arm)
  teleop follower   Inverse3 → clutch, scaling, camera-frame map → tip-pose target + tip twist
  policy follower   a learned policy's absolute tip-pose targets, at the policy's rate
  goal mover        point-to-point goals: a tip pose, or a joint goal (skips the task layer)
        │  absolute tip-pose target (+ tip-twist feed-forward when the source has one)
        ▼
task layer      resolved-rate law, q̇ = argmin ‖J q̇ − (V_ff + K·e)‖² + damping,
                subject to the RCM as a HARD equality constraint and joint-rate limits
                (later: collision velocity dampers); q̇ integrated to joint targets
        ▼
joint layer     Ruckig (velocity / acceleration / jerk) + position limits
        ▼
hardware        per-arm forward position command: 500 Hz UR5e manager, 125 Hz UR5 manager
```

Joint targets cover the UR arm (6) and the instrument's roll, pitch and yaw (3). Jaw opening
is commanded directly, not through the task layer.

**The recorded action is the absolute tip-pose target** the task layer consumes (frame named
in the TF tree, decision 12), together with the jaw command and the id of the active source.
A teleop demonstration and a policy rollout therefore write the same field, and a policy
trained on teleop data is replayed through exactly the path that recorded it.

## Why

- **Autonomy research is position-commanded.** Policies (SRT-H-style and most imitation
  learning) output target poses; ISO 9283:1998 repeatability (move to one pose from 30 random
  starts, for 5 poses, on ARTS and on dVRK-Si) accepts only position goals. A velocity-only
  stack would need a second path for these, and two paths means two sets of limits to trust.
- **Teleop keeps the feel of velocity control.** Collaborator A's resolved-rate law,
  `V_ff + K·e`, is what the funder-demo operators found comfortable. Here it is the task
  layer, integrated to positions, so the operator gets velocity feed-forward responsiveness
  while the arm still receives position commands that Ruckig and the limits bound. Streaming
  `speedj` to the controller box (A's transport) moves the integration out of ARTS's control
  and out of the recording.
- **The RCM as a hard constraint keeps orientation at full weight.** Collaborator B integrates
  the same law but satisfies the RCM by weighting, with orientation reduced to a 0.005-weight
  soft objective; that is the most likely source of B's wrist inaccuracy (2026-10-27 review,
  unverified). A QP equality constraint removes the trade-off.
- **One set of guards for every source.** Rate clamps, damping near singularities, collision
  dampers and contact stops sit in the task and joint layers, so they apply to teleop, a
  policy and the goal mover alike.

## What comes from collaborators A and B, and MOPS

Ideas only, re-derived from published methods and cited in the paper; no code. Nothing from
`code_intake/` is built, sourced or copied into the tree (AGENTS.md, code intake), and
nothing without an open-source licence enters the capsule.

| Idea | Seen in | ARTS home |
|---|---|---|
| Resolved-rate tip control with feed-forward, `V_ff + K·e` | A (and B, since 2026-09-25) | task layer |
| Joint-rate clamps, damping ramps near singularities | B | task + joint layers |
| IK branch lock (no elbow/wrist flip mid-motion) | B | task layer, plus a branch / gimbal-lock regression test before any real-arm run |
| Per-arm speed scaling from capsule distances | A, B | task layer (collision dampers, Phase 2) |
| Contact stop on TCP force above a baseline | A, B | safety, beside ARM HOLD |
| UR + EndoWrist adapter, software RCM | MOPS (published) | cited as ARTS's closest ancestor |

## Where each part lives (decision 2 layering, sourcing rules unchanged)

| Part | Layer |
|---|---|
| UR driver | `ur_ros2_ws` middle overlay, pinned; installed by `deploy/modules/ur.sh`; no MoveIt (decision 11) |
| Haply bridge | `oem_ros2_ws` middle overlay, pinned; thin config in `arts_driver` |
| pinocchio, coal, proxsuite | `open_source_software/`, pinned, built before `arts_ros2_ws` |
| Reference sources, task and joint layers, guards | `arts_ros2_ws`: `arts_controllers`, `arts_teleop`, `arts_safety` |
| Procedure-specific autonomy | application overlays only |

## Not decided by measurement

The AGENTS.md item planned a three-way measured comparison (ARTS servo, A's velocity control,
B's position control) on our arm. With A and B shown to share one law (2026-10-27) and the
deadline at 2026-11-10, the architecture is decided by the argument above instead. What stays
measured: E5 loop health, E6 teleop latency, E8 stop time, and the branch / gimbal-lock
regression test.

**Before the 2026-11-04 freeze**, E5, E6 and E8 run on the existing
`CartesianServoController` (DLS IK → Ruckig) in free space, without the RCM. The RCM QP task
layer comes after the deadline; the paper states this.

## Amendment 2026-10-30 — the resolved-rate law with feedforward is in the controller

The first teleop session on the real left UR5e (72 s, scale 0.25) showed a 140 ms lag from
command to arm motion. Replaying its recorded commands on mock isolated 85 ms of it in the
controller: in "position" tracking Ruckig plans to ARRIVE AT REST at every new target, so a
moving target is followed by an arm that keeps braking. `CartesianServoController` now has
`tracking.mode: rate`, this decision's law in joint space: Ruckig, in its velocity
interface, is driven to `J⁺ V_ff + K (q_ik − q_cmd)`, with V_ff the tool twist on
`~/target_velocity` (the teleop follower sends the Haply's measured cursor velocity, scaled
and rotated). It is not the "target position plus target velocity" form, which was tried on
2026-08-12 and made Ruckig reverse for runway (see the source comment). Mock replay of the
same session: lag 83 ms (position) → 79 ms (rate, V_ff = 0) → 29 ms (rate + V_ff). K = 20
put the wrist joints into a 0.2–0.9° limit cycle (Ruckig's jerk-limited velocity change acts
as a delay); K = 10 and 5 settle to 0.007°; K = 5 is used. Rate mode is now the configured
mode (`config/ur5e_servo.yaml`); smoke, singularity and follower tests pass in it. E5 was
measured in position mode. The remaining real-arm lag (UR servoj look-ahead 30 ms, arm
dynamics) is measured in the next real session, not predicted.

**Same day, after the second real session.** With the feedforward the operator found fast
back-and-forth motion reproduced at roughly 1/10 of the hand (scale 0.25 × ~0.4): the 60 mm/s
tool cap (from the v1.4 reference's profile) and the provisional joint limits (5 rad/s²,
50 rad/s³) clipped a 4 Hz shake. An operator shaking an instrument expects it to shake, so
for teleop the cap is 250 mm/s and the joint limits 20 rad/s² / 500 rad/s³ (operator
decision; `config/ur5e_servo.yaml` gives the reasoning). Raising them exposed a flaw in the
rate law: a shake beyond the limits made the arm DRIFT, up to 2.9× the commanded amplitude
on mock, because Ruckig clips the velocity unevenly while the feedforward keeps pushing. The
feedforward is now weighted by (1) its own feasibility, the joint acceleration and jerk it
needs against the limits (full ≤ 70 %, none ≥ 100 %), (2) Ruckig's acceleration saturation,
and (3) a 10–20 mm tracking-error backstop. Mock shake sweep (`test/rate_sweep_test.py`,
fails above 1.05×): 2 Hz and 4 Hz × 15 mm followed in full (1.00–1.03, lag 30–37 ms), 4 Hz ×
30 mm 0.66, 6–10 Hz damped to 0.07–0.19, never amplified. Session 1 replayed on mock: 8 ms.
`servoj` stays: the limits, not the UR command, were binding. The real-arm response is
measured next.

**speedj option (2026-10-30).** The third real session followed fast shakes in full (median
1.00 of the command at 3-5 Hz) but with a steady ~110 ms from command to measured motion,
~100 ms of it on the UR side of servoj (look-ahead smoothing, minimum 0.03 s, and the UR's
own servo; gain already at its 2000 maximum). The controller now has an opt-in
`command_interface: velocity`, which the UR driver executes as `speedj(qd, 40 rad/s², 2 ms)`
with no look-ahead; it requires rate mode, and the rate law then starts every cycle from the
MEASURED joints, because velocity commands close no position loop by themselves. On a lost
command stream the UR script brakes with `stopj` (4 rad/s²) after 0.04 s. Launch argument
`command_interface:=position|velocity`, default position. Mock: smoke, singularity, shake
sweep and follower tests pass in both. Real-arm A/B with the same scripted shakes
(`test/shake_ab_real_arm.py`, 10 mm at 1, 2, 4 Hz) decides; until then servoj stays.

**speedj is the default (2026-10-30, after the A/B).** Same scripted shakes on the real left
UR5e, lag from command to the measured joints: `servoj` 34 / 35 / 40 ms at 1 / 2 / 4 Hz,
`speedj` 4–5 / 5 / 9 ms, equal amplitude; a hard-killed driver under `speedj` left the arm
still and stopped the pendant program
([`servoj_speedj_ab_20261030.md`](../benchmarks/servoj_speedj_ab_20261030.md)). The arm
launch's `command_interface` now defaults to `velocity`; `position` (servoj) remains for
comparison and for a config without rate mode. E6 with `speedj`: 58 / 54 ms motion to motion
on video ([`e6_teleop_latency_20261030.md`](../benchmarks/e6_teleop_latency_20261030.md)).

**Correction to the paragraphs above.** "~110 ms from command to measured motion, ~100 ms of
it on the UR side of servoj" and session 1's "140 ms" were computed by composing
base_link → tool0 from recorded `/tf`, which mis-times the arm (each `/tf` message carries
only part of the chain; on the E6 bag the same method quantised the lag to 51 / 102 ms).
Recomputed from `/joint_states` (handle stamp → measured joints, whole session,
`arts_teleop/bench/e6_bag_latency.py`): session 1 (position tracking, servoj) 111 ms,
session 3 (rate + feedforward, servoj) 84 ms, E6 (rate + feedforward, speedj) 58 ms. The UR
side of `servoj` is ~35–40 ms, not ~100 ms. Of the remaining ~45 ms (window median), the
scripted shake with an exact analytic feedforward reaches the joints in ≤ 10 ms, so most of
it lies between the Haply velocity feedforward and the controller (its weighting and the
K = 5 tracking term) — the next lever if E6 must go lower; not investigated.

## Amendment 2026-11-01 — the RCM task layer is implemented (mock), switchable live

The task layer of the diagram above now exists inside `CartesianServoController`, before
the deadline rather than after it as the 2026-10-28 text planned. The structure:

```
reference --> task QP (proxsuite 0.7.3)               --> Ruckig            --> projection onto     --> joints
              min |W (J qd - v)|^2 + l |qd - qd_rest|^2    (unchanged: vel,      the RCM rows
              s.t. RCM rows = (2 equalities)               acc, jerk limits)     (closed form)
                   insertion depth in [min, max]
                   joint velocity + position boxes
```

- **The law** is this decision's `V_ff + K e` in task space (`e` = tip position error and
  the rotation error's log), with the same feedforward weighting as the IK path.
- **The RCM rows** ask the shaft's material point at the foot of the RCM point to move normal
  to the shaft at `gain x deviation`, so a deviation decays and cannot accumulate (the
  "will the RCM drift?" question): it is a feedback constraint on the measured state, not an
  open-loop one.
- **Orientation under the RCM** is the roll about the shaft only (`rcm.orientation_task:
  roll`): 6 UR joints minus 2 constraint rows leave exactly 4 DoF, the tip position and the
  roll, so the task is met exactly and the shaft direction follows from the tip position.
  That is what every RCM platform without an instrument wrist does (the open laparoscopic
  platform commands pivot, roll and insertion). `full` is for when the wrist joints join the
  controller; then all three axes are tracked at full weight, which is this decision's
  argument against B's 0.005 weighting.
- **A posture term** (the damping pulls toward the joints at activation and at each switch)
  keeps the joints from drifting where the task leaves a direction free; without it a round
  trip through wrist_2 = 0 came back with joints 0.6 deg different.

**Switching** is a service, `<arm>/set_rcm` (decision 18 amendment), with no controller swap.
ON is refused unless the shaft already passes within 2 mm of the point and the tip is
inserted 5–300 mm beyond it, so enabling never pulls the arm. After either switch the
controller holds until a newer target arrives, and the teleop follower disengages until the
pedal is pressed again (with the RCM on, the shaft orientation follows from the tip, so the
orientation the follower held is stale after the switch). The tool-tip offset is frozen
while the RCM is on (it defines the shaft line), the RCM is off at every activation, and a
measured deviation beyond 5 mm latches a hold. RCM OFF keeps the IK law that E5, E6 and E8
were measured with (`tracking.solver: ik`); `solver: qp` runs the same QP without the RCM
rows and passes the same mock suite, but has not run on a real arm.

**Two designs failed on mock first, and why they fail is the reasoning behind the structure:**

1. *QP → Ruckig, nothing after.* Ruckig smooths each joint on its own and cannot see the
   constraint; its jerk-limited lag between the QP's joint velocity and the one sent held
   the shaft 0.22 mm (median) off the point on a 1 Hz Lissajous. The projection removes
   exactly that lag in the two normal directions: 0.002 mm on the same motion.
2. *Limits inside the QP, Ruckig bypassed.* One-step acceleration and jerk boxes cannot plan
   braking the way Ruckig does, so velocity overshoots: a 2 Hz shake came out 1.32x the
   command. Ruckig stays; the QP keeps only the velocity and position boxes.

Also found: capping the tool's whole angular speed (the IK path's governor) throttled every
lateral move under the RCM, because pivoting about a point 100 mm behind the tip turns tip
speed into angular speed (v / depth); the QP caps the task (tip speed, commanded roll)
instead. A warm-started proxsuite solve after an infeasible one occasionally reported a false
PRIMAL_INFEASIBLE; one cold retry clears it. proxsuite's `update()` allocates a temporary per
vector, so the vectors are written into the model directly (`test/task_qp_test.cpp` counts
allocations at the malloc level and was seen failing on a mutant that allocates).

**Mock results** (`test/rcm_mock_test.py`, 300 mm instrument, RCM 100 mm behind the tip,
velocity feedforward): 0.5 Hz x 10 mm Lissajous plus 10 mm insertion, kinematic deviation
median **0.0013 mm**, p95 0.0046, max 0.0057 mm, tip tracking 0.47 mm RMS, shaft tilt
5.69 deg against atan(10/100) = 5.71; at twice the rate and 1.5x the size (beyond the 1 rad/s
joint limit) the tip lags, the RCM holds (max 0.012 mm); the same motion with the RCM off
leaves the point by 12.9 mm, so the metric can fail. Insertion floor held at 5.00 mm. QP
solve 1.8 µs mean, 21 µs worst on a non-isolated core. The follower test teleoperates under
the RCM (checks 9). Shake sweep, smoke and singularity tests unchanged on the IK path.

**These are not paper numbers.** On mock the measured joints are the commanded joints, so the
deviation is the controller's own consistency. The paper's S1 is `<arm>/rcm_state` recorded on
the real UR5e during teleop: the same definition as the open laparoscopic platform's (minimum
distance from the commanded pivot to the infinite shaft line, forward kinematics of the
recorded joints), so the two compare directly; the UR's tracking will add to it. MOPS's
method could not be read (the IEEE page refused automated access); the comparison rests on
the open platform's published definition. (Read the same day; see the amendment below.)

## Amendment 2026-11-01 (b) — the adapter's geometry, joint weights, MOPS read

**The da Vinci X adapter's shaft is PERPENDICULAR to wrist_3.** Collaborator A's model (the
stack that drove these arms with `speedj`) puts the shaft along tool0 +y, crossing the wrist_3
axis 88.5 mm beyond the flange, wrist pitch 600.0 mm and yaw 609.0 mm along it
(`arts_driver/config/tool_tips.yaml`, `da_vinci_x_wrist`; B's CAD-derived model says 88.8 mm
and ~593.3 mm, a discrepancy to settle from the adapter's CAD). Consequences:

- wrist_3 swings the shaft; it does not roll it. Rolling the shaft about itself with the UR
  alone moves all six joints, the elbow and shoulder included, which is exactly the motion
  that brings arms toward each other. So under the RCM the default `rcm.orientation_task` is
  now **`none`**: the task is the tip position only, and the remaining DoF (the roll) is
  held by the posture term. `roll` stays available for a coaxial tool; shaft roll returns
  with the instrument's roll disc, `full` with the wrist.
- **Per-joint weights** on the QP's damping/posture term (`task_qp.joint_weights`): the
  weighted least-norm resolution of redundancy, as the MOPS paper does to prefer instrument
  joints. The six UR joints are one category (equal weights); the instrument joints get
  lower weights when they join. With six joints under the RCM the problem has no redundancy
  and the weights do nothing; `test/task_qp_test.cpp` checks the split on a redundant pair
  (seen failing with the weights ignored). A hard "UR joints never spin the shaft" row
  (collaborator B's) is held back as the fallback if the weights prove insufficient.
- `set_rcm` with no depth reuses the last engaged depth (`rcm.default_insertion_depth`
  before the first), so off → move → on relocates the point onto the new shaft.

**MOPS, read** (Schwaner et al., ISMR 2021). RCM off: weighted least-norm resolved rate,
W favouring the instrument joints. RCM on: the extended-Jacobian method of Aghakhani et al.
(ICRA 2013), insertion depth as an extra variable, a proportional term K_RCM(p_trocar −
p_RCM) pulling the shaft back, Moore–Penrose solve; the trocar point is set by hand. The paper
reports **no RCM error**; its accuracy number is the vision-in-loop trajectory error (1.2 mm /
0.3°, UR5e). The released code (`mops_core`) contains **no RCM controller**: a first draft was
added and removed in 2021 as unfinished; what ships is 9-DoF weighted-DLS pose IK at 125 Hz
with joint weights 0.2 (UR) / 1 (instrument). So the RCM comparison is with the open platform
only; MOPS is compared through its trajectory protocol (cut to future work).

**Why the RCM does not drift here, by construction.** The point is fixed in the base frame;
each cycle the RCM rows command the shaft's normal velocity at the point to remove `gain` ×
the current deviation (a position feedback, 0.1 s time constant), and with `speedj` the cycle
starts from the measured joints, so nothing is integrated open-loop. Drift belongs to schemes
that constrain only ṗ_RCM = 0 without the position term, or that make the RCM soft so a
persistent task pull trades it away. What the real arm can add is lag (a velocity-dependent
offset that vanishes at rest) and geometry error (constant); the 30 min S1 drift slope is the
test.

## Amendment 2026-11-03 — the instrument wrist joins the controller (collaborator A's design)

E9 (full-pose teleop, PI 2026-11-02) needs the tip ORIENTATION, which a UR alone cannot give
under the RCM. Collaborator A drove these arms this way for weeks (read at intake commit
`18063e2`, 2026-11-02); ARTS ports A's design rather than inventing one, and keeps its own
tested pieces only where they are the same constraint written differently.

**Joints.** Nine: the UR's six and the instrument's roll, wrist pitch and wrist yaw (yaw =
the common bend of the two jaws). The jaw OPENING stays outside the kinematic problem, as in
A's: it does not move the frame at the base of the jaws, and it is commanded directly (the
jaw follower). The three wrist joints are added to the controller's pinocchio model at
configure time from the instrument geometry, the same way the tip frame is added: roll about
the shaft line (88.4 mm from the flange along tool0 z, shaft along tool0 +y), pitch 0.600 m
along the shaft, yaw 9.0 mm beyond it, the tip frame on the yaw axis with z along the shaft
(at zero wrist angles this is exactly the `da_vinci_x_wrist` tip preset). These are A's
values, hard-coded in A's model for every instrument; 88.4 mm matches the adapter CAD
(88.388 mm), the other two are bring-up values to be measured per instrument after the
deadline (docs/instrument_identification.md). The URDF gains the instrument chain later, for
viewing.

**The RCM frame** is A's frame E: the distal end of the shaft, on the pitch axis, after the
roll joint. The RCM rows, the insertion depth and the published deviation are computed from E,
no longer from the tip, which now turns away from the shaft with the wrist.

**Constraint rows.** A writes the RCM as three rows with the insertion rate as a tenth unknown
(10 x 10, square). ARTS keeps its two rows normal to the shaft with the depth computed from the
geometry (2026-11-01 amendment): eliminating A's insertion rate from A's three rows gives
exactly these two, and they are already measured (0.0013 mm mock, 0.0135 mm on the left UR5e).
A's ROLL ROW is added as an equality: the UR's angular velocity about the shaft (measured in a
frame fixed to the flange, before the instrument roll) equals `-k_roll` x the shaft's roll
away from its value at engagement. All rotation about the shaft is then the instrument roll's,
and the elbow and shoulder do not swing toward the other arms to roll the shaft. With the RCM
on: 9 joints, 2 RCM rows + 1 roll row + 6 task rows, square, as A's. With the RCM off (open
surgery): the roll row stays, 2 DoF are redundant and the posture term with joint weights
favoring the instrument resolves them (MOPS's 0.2 : 1 ratio; in ARTS's QP a LOWER weight is
cheaper, so UR 1.0, instrument 0.2).

**Task.** Full orientation at full weight (`rcm.orientation_task` is `full` whenever the wrist
is present), A's gains as the starting values (position 1.5 /s, orientation 5.0 /s, RCM 2.5
/s, roll 2.5 /s), measured before they are kept. The QP, Ruckig and the exact RCM projection
are unchanged, now over nine joints.

**Rates: each module at its best measured rate, integrated in one controller.** The
controller runs at the UR5e's 500 Hz over all nine joints. The UR joints are commanded through
ros2_control as before. The wrist velocities go out on `<arm>/tool_joint_target` every cycle
and `disc_drive` applies the newest at its bus limit, 333 Hz (Fast Sync Read, 2026-11-03); the
wrist state comes back on `<arm>/tool_joint_states` and is used like the UR's measured joints,
the newest sample each cycle. A wrist state older than `wrist.state_timeout` holds the arm and
zeroes the wrist; `disc_drive` also zeroes a joint whose target is older than 0.5 s, per joint,
so the jaw follower (jaw only) and the controller (roll, pitch, yaw) do not overwrite each
other. A ros2_control hardware component for the discs, inside the arm's manager, is the later
form; the 3 ms bus cycle against the 2 ms loop needs its own asynchronous design.

**Mapping (follower).** A's: VerseGrip orientation in the Haply base, a fixed Haply-to-screen
rotation per hand tilted by the GOOVIS angle (A: 45 deg), the screen taken as the camera frame,
the camera pose from the ECM's forward kinematics and a nominal view frame on the scope (from
ARTS's CAD; `ecm_stereo_optical_frame` in the ROS optical convention, z into the scene, so point-cloud
depth is +z; the screen is it turned 180 deg about x; PI, 2026-11-03), then the arm base. Absolute orientation with the residual (< 10 deg) frozen at
engagement; position incremental, scaled 0.25; A's 10 deg alignment gate before teleop
activates; pedal UP = active, pressed = disengaged; the middle pedal lets the arm rotate the
instrument toward the stylus until the gate opens; VerseGrip button b straightens the wrist.
(Corrected 2026-11-03 from A's code: camera teleop is A's RIGHT pedal and "bring to stylus" the
middle; the PI aligned visually and used neither helper. ARTS: left = clutch, middle = camera,
right = record; "bring to stylus" on VerseGrip button b (PI); straighten not ported.)

**MIS / open surgery** is the RCM switch, per arm, at the supervisor station (arts_gui), never
from the master: "MIS" at launch waits for the cannula, "RCM on here" (the existing guard: the
shaft within 2 mm) after insertion; MIS -> open only with that arm's teleop disengaged and the
instrument confirmed clear of the cannula; the mode is shown on the GOOVIS overlay.

**Not changed.** The 6-joint paths (IK and QP) that E5, E6 and E8 were measured with stay as
they are; the wrist is a configuration (`wrist.enabled`), off unless the arm has an instrument
drive.

**Implemented, mock (2026-11-03).** `wrist.enabled` in `CartesianServoController` (Ruckig at 9
DoF with the last three disabled on a 6-joint arm; the QP with A's roll row as an optional third
equality, `test/task_qp_test.cpp` 11 checks, the roll-row exactness check seen failing with the
row's right-hand side broken). Launch: `ur_arm.launch.py wrist_instrument:=<file>` /
`arts_bringup wrist:=true`; on mock `fake_instrument.py` stands in for disc_drive.
`test/wrist_mock_test.py` (needle driver limits), 12 checks pass: 9 joints commanded; the
tip-offset service refused; RCM off, 20 deg about tip x and y and a 30 deg roll reached within
0.02 deg at <= 0.004 mm, the roll taken by the instrument roll joint (29.98 of 30 deg); RCM on at
frame E, a 20 deg wrist turn plus 5 mm lateral within 0.007 deg / 0.002 mm, deviation max 0.0003
mm; under the RCM a 110 deg target drives the pitch into its limit (78.6 of 80 deg) with the RCM
held; a stalled drive (fake instrument paused) holds the arm with the wrist command at zero, and
it resumes. On mock the roll check does not isolate the roll row (the posture term also pulls
the UR back). The whole 6-joint mock suite passes unchanged with the wrist off (smoke,
branch/singularity, RCM, shake sweep, tip frame, follower, gamepad). Gains are still ARTS's
(tracking.gain 5, rcm.gain 10), not yet A's; that and the real arm come with E9.

**Follower orientation, mock (2026-11-03).** `teleop_follower.py orientation:=true`: A's mapping
and gate as above. The right Inverse3's measured axis map is a mirror (det -1, the Hub's
handedness), which A's own matrices do not have; a mirror cannot multiply an orientation, so
one stylus axis is mirrored with it (D = diag(1, 1, det M)) to keep the stylus a proper
rotation, and a per-hand `stylus_calibration` (identity until set at the bench) stands where
A's hand-tuned matrices stood. The align assist runs from a pedal mapped to `align` (none is,
yet: the middle pedal is `camera`, the right `record_toggle`; the PI decides). No angular
feedforward yet. `arts_teleop/test/follower_orientation_mock_test.py` on the 9-joint mock arm
(screen = base, no camera frame), 9 checks pass: not engaged 30 deg off (the arm holds,
alignment 30.0 deg reported); clutch down + align pedal turns the instrument to 0.02 deg with the
position held to 0.08 mm (since 2026-11-03 on VerseGrip button b, not a pedal); engaged through the gate; a 20 deg stylus turn gives 19.99 deg at the
tip (0.017 deg from the stylus); 40 mm of handle gives 9.99 mm; clutch down, nothing moves. The
position-only follower test passes unchanged. Not yet tested: the camera frame (tf from the
ECM) and the GOOVIS tilt.

**MIS / open switch in the GUI (2026-11-03).** `arts_gui` tab "Surgery mode": per PSM the live
state (RCM on/off, deviation, insertion, teleop engaged) and "MIS: RCM on here" / "Open
surgery: RCM off", both through `<arm>/set_rcm`; off is refused in the GUI while that arm's
teleop is engaged and asks for the cannula-clear confirmation; on keeps the controller's 2 mm
guard. One background rclpy node in the GUI (reads state, calls the service on a click only).
Checked on mock through its ROS link (state read, on, off, an absent arm reported) and the GUI
constructed off-screen; not yet clicked at the supervisor station.

**Alignment overlay (2026-11-03).** The follower publishes, while an arm waits at the gate, the
instrument's and the stylus's orientations in the screen frame (`<arm>/teleop_alignment_frames`,
50 Hz). `arts_display` draws A's two triads per PSM (stylus thick and light, instrument thin,
x red / y green / z blue, at 1/4 and 3/4 of the width, mid height, orthographic and identical
in both eyes so they read at screen depth) and a mode line per PSM (MIS / OPEN from
`rcm_state`, the angle or ENGAGED from `teleop_state`), before the HMD squeeze, like the status
text (`align_overlay`, default on). Rendered with synthetic frames and topics and checked by
screenshot; not yet seen in the GOOVIS.

## Amendment 2026-11-05 — first orientation teleop on the real arm (E9)

The 2026-11-03 design ran on psm_left with the needle driver. What the bench overturned, each
found on the real arm and fixed the same day (E9 report:
`docs/benchmarks/e9_full_pose_teleop_20261105.md`):

- **Yaw sign.** The wrist model's yaw axis is -y, not +y. FLIR frames at single-joint poses
  (pitch +40, yaw -35, roll +36 deg, each from straight) matched the model for pitch and roll
  and mirrored it for yaw; the 5-deg sign checks of 2026-11-04 could not show it by eye. The
  drive's joint sign (A's coupling) is kept; the model changed.
- **Display rotation.** The camera-frame screen is the optical frame turned about x AND z when
  the display shows the image rotated 180 deg (`display_rotate_180`); without z, hand
  left/right and up/down were both reversed. Checked by projecting the modelled shaft into the
  image.
- **Pen axis.** The VerseGrip's tip is its body +y and its buttons +z (held poses after the
  pen's calibration button, which zeroes it pointing ahead, buttons up). A's chain used the
  body frame as the tip frame, which put the shaft out of the buttons on this pen and
  firmware; `stylus_calibration` now maps tool z = pen +y, tool y = pen +z. The pen drifts
  (~15 deg before a calibration): calibrate each session, as A did.
- **Room frame for E9.** The operator found camera-frame orientation (pen up = image up)
  hard to reason about with a camera looking 76 deg down and chose the room frame:
  `operator_to_base` from gravity (world +z) and the ECM's heading on the floor. It stops
  matching the view if the ECM moves; camera frame stays available (`camera_frame`).
- **Rate.** The 15 deg/s tip angular cap (v1.4, set for the UR wrist alone) is 90 deg/s with
  the wrist (operator decision), and the follower adds A's angular feedforward (the stylus
  differenced, median + low-pass, zero when the pen goes quiet).
- **UR share.** Instrument joint weights 0.2 -> 0.01. At MOPS's 0.2 the least-norm split gave
  the UR a visible share of every wrist rotation, which the posture term pulled back after the
  hand stopped (elbow sweeping at 10-26 deg/s, tip on target); at 0.01 UR joint speed p99 fell
  from 9.0 to 1.6 deg/s.
- **Bend limit (RCM off).** The VerseGrip's ball joint turns without end; once the wrist
  (+-80 deg) cannot make a rotation the QP makes it with the UR. The follower keeps the
  target's jaw axis within 60 deg of the shaft, roll free; with the RCM on the port already
  stops the UR from taking over. A bench safety setting reported with the experiments, like
  the +-50 mm box, not a surgical workspace.

Operator observations from the E9 runs, not acted on before the freeze: (1) a suturing-like
tip rotation still brings in visible UR elbow motion (UR spans up to 16 deg in run 2), which
the operator accepts where the rotation needs it; (2) a surgical bend limit should not be
symmetric about the shaft: pitching the tip toward the camera (more of the tip visible) is
the more useful direction, so a future limit should be shaped by the view, not a cone.

### Amendment 2026-11-07: ECM on servoj, set_rcm with the wrist, collision hold

- **ECM command interface.** Under the RCM the camera arm (UR5 CB3, 125 Hz) rang at ~9.5 Hz,
  0.35-0.45 mm at the scope tip, while the commanded targets were smooth; it was smooth without
  the RCM and the same motion on mock (no delay) showed no ringing. With `speedj` the
  controller restarts every cycle from the MEASURED joints, so the RCM correction acts on the
  CB3's delayed measurement. On `servoj` (the controller integrates its own command) the same
  teleop showed 0.03-0.08 mm of fast motion, no peak, and the RCM held 0.043 mm median. The
  ECM runs `servoj` (`ur_arms.yaml` `command_interface: position`); the tool arms (500 Hz)
  stay on `speedj`, where RCM teleop shows no ringing (0.024-0.026 mm). Tuning the gains for
  `speedj` on the CB3 is not done.
- **set_rcm with the wrist** placed the point along the jaw axis (the tip frame) while every
  cycle measures it against the shaft (frame E): at a wrist bend theta the two differ by
  depth * sin(theta), and the fault latch tripped in the same cycle (12.45 mm at ~8 deg on the
  real arm). The point is now placed from frame E (`rcm_bent_wrist_mock_test.py`: 20 deg bend,
  point on the shaft 0.000 mm, the old placement 34.6 mm off).
- **Collision hold.** `/arts/collision_hold` (latched Bool, `arts_safety collision_guard`)
  makes every servo controller hold like a stale reference; the teleop follower and
  camera_teleop disengage until a fresh pedal press (`collision_hold_mock_test.py`).
