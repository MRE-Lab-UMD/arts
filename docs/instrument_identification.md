# Instrument identification protocol

How ARTS obtains the kinematics, coupling, and geometry of a wristed da Vinci X instrument
from the physical instrument, without reusing anyone else's model files.

This exists for two reasons and it is worth being clear that the second is the better one.
The narrow reason is licensing: the geometry published by dVRK and MOPS is unlicensed and
cannot be redistributed (the full verdict is in the catalog's `parts/4_tools/NOTES.md`). The
real reason is that **the published numbers have no stated uncertainty.** dVRK's tool files
are transcribed from a vendor document; MOPS's shaft length is a comment in a xacro. Neither
reports how the value was obtained, how repeatable it is, or how well it predicts where the
tip actually goes. A measured parameter set with an error budget and a replicable procedure
is a better artifact than a copied one, and it is the thing this document is really for.

The protocol is the contribution. It ships as `arts_calibration/arts_instrument_id` with the
raw data and the reports, so a reader with a different instrument — a generation we do not
own, a tool Intuitive releases next year — can run it and get their own numbers rather than
waiting for ours.

## Nominal geometry in use until this runs (updated 2026-11-02)

The controller's da Vinci X tip preset (`arts_driver/config/tool_tips.yaml`,
`da_vinci_x_wrist`): the shaft LINE comes from the adapter designer's CAD
(`arts_description/scripts/import_tool_step.py`, 2026-11-02): perpendicular to wrist_3
along tool0 +y, crossing the flange axis, **88.388 mm** beyond the flange face (A had 88.5 mm,
B 88.8 mm). The distance ALONG the shaft is still collaborator A's: wrist pitch axis 600.0 mm,
yaw axis 609.0 mm; no jaw length (the "tip" is the wrist centre). B's ~593.3 mm to the first
wrist joint disagrees, and the length is per instrument, so it is measured per instrument
(rule below). The kinematic RCM error depends on the shaft line (direction and offset), not on
the length along it.

## ARTS's instruments (part numbers read off the instruments, 2026-11-02)

| Instrument | Ref | Shaft | Joints beyond roll/pitch/yaw |
|---|---|---|---|
| Large needle driver | 471006 | 8 mm | jaw |
| Monopolar curved scissors | 470179 | 8 mm | jaw |
| Fenestrated bipolar forceps | 471205 | 8 mm | jaw |
| Permanent cautery hook | 470183 | 8 mm | none |
| SureForm 45 stapler (optional) | 480445 | 12 mm | jaw + blade |
| Stapler 45 (optional) | 470298 | 12 mm | jaw + blade |

Instrument files are keyed by this ref; every recording names the fitted instrument and its
ref (`tool_info`). The staplers enter the paper only if the adapter drives them.

**Does the geometry differ by tool? (dVRK read 2026-11-02, nothing transcribed.)** In dVRK's
55 tool definitions the shaft length is per GENERATION (one value for all Classic 8 mm wristed
tools, one 50.8 mm longer for all S/Si), while the wrist pitch-to-yaw offset differs by family
(9.1 mm needle drivers and scissors, 9.3 mm cautery tools, 10.7 mm fenestrated bipolar and
ProGrasp, 11.2 mm Mega needle driver) and the jaw length beyond the yaw axis is 10–29 mm. dVRK
has no X/Xi (47xxxx/48xxxx) tool, so ARTS's values come from collaborator A and then
measurement. X/Xi instruments are longer than Classic and S/Si ones (PI, 2026-11-02).
A's model uses 9.0 mm for every instrument, which the dVRK pattern says is ~1.7 mm short for
the forceps family — a sim2real error source for learned policies.

**Rule (PI, 2026-11-02): every geometric value is measured PER INSTRUMENT** — shaft length,
pitch-to-yaw offset and jaw length, on each instrument by its ref, never a generic or family
value. A's numbers are the bring-up placeholder only; an instrument file says which of its
values are still A's and which are measured.

## STATUS: deferred. Read this before starting any of it.

**This is a TODO, not current work.** The full protocol is weeks and nothing that has to work
first needs it. Nothing below Stage A should start until the tool drive is teleoperating.

CORRECTED 2026-08-19 — THE DEADLINE THIS SECTION USED TO CITE DOES NOT APPLY TO ARTS, and the
conclusion survives anyway. Every "before 5 October" below was written believing the funder
demo ran on ARTS; it runs on the legacy Ubuntu 22.04 + Humble stack (see AGENTS.md, "Two
separate deliverables"), so ARTS owes nothing on that date and the ordering here is driven by
dependency rather than by a calendar. Read the staging notes as "before first teleoperation"
and "after it", not as October.

> **Amended 2026-10-29 — collaborator A's values go IN the tree.** Collaborator A, a co-author
> on the ISMR system paper, agreed to ARTS publishing A's instrument DH parameters and coupling
> values (measured by A on a bipolar forceps through A's Dynamixel adapter, other instruments
> hand-tuned from it; `$ARTS_LOCAL/code_intake/ctrl_velocity/NOTES.md`). They become the
> bring-up configuration, in a file stating that provenance and each instrument's PART NUMBER,
> and are credited in the paper. Three of A's disc-to-joint coefficients equal dVRK's published
> values exactly (item 2 below), so the file cites dVRK for those as well. The two paragraphs
> below (dVRK's values, kept out of the tree) are superseded for bring-up; the protocol still
> applies as the cross-check that gives the numbers an uncertainty.
>
> **Joint sets differ by instrument.** The controller's pose joints are always roll, wrist
> pitch, wrist yaw (UR 6 + 3, decision 17). Beyond those: graspers, scissors and forceps add a
> jaw; the cautery spatula and hook have NO jaw; the stapler adds a fifth driven disc that
> pushes the blade along the closed jaw. So an instrument file declares which of {jaw, blade}
> it has, and the coupling maps that instrument's joints to its discs (up to 5) in the
> hardware interface. The IK never sees a disc.

**To bring the tool drive up, use dVRK's published DH parameters and coupling matrices.** They
are the only validated numbers in existence for these instruments, so starting from them means
a wrong joint angle is a bug in ARTS's drive, disc ordering, sign convention or homing rather
than two unknowns compounding. Do not substitute a rushed version of the protocol below: a
half-measured parameter set is worse than a borrowed one, because it looks like ours.

**Using them is not the thing the licensing section prohibits.** The boundary is
redistribution, not use. Keep the values in a file marked clearly as third-party with its
source attributed, hold it in `$ARTS_LOCAL` or gitignored rather than in the published tree,
and it never reaches the capsule. Recorded bags are unaffected — poses computed with
someone's parameters are data, not a copy of the parameter file. Before publication, either
replace them with measured values or cite dVRK for the ones used; citing and using are normal
practice and were never the concern.

**But "validated" means validated for dVRK's hardware, not for ours, so bench-check before
trusting them.** This is the same principle as not debuting an unvalidated method — third-party
numbers on a different adapter are also unvalidated until shown otherwise. Four things to
confirm, and they are an afternoon, not a campaign:

1. **Part numbers must match.** dVRK covers `MONOPOLAR_CURVED_SCISSORS_400179`/`420179` and
   `FENESTRATED_BIPOLAR_FORCEPS_400205`/`420205`. Xi variants are `470xxx` and have no dVRK
   file at all. Read the number off the instrument first; if it is a generation dVRK does not
   cover, this plan does not apply to that instrument and you need to know that on the first
   day, not after the drive is built around an assumption.
2. **Whether the published values transfer to a non-ISI adapter — ANSWERED, 2026-08-12, and
   the answer is yes.** The worry was that dVRK's `ActuatorToJointPosition` might fold in the
   PSM's own motor and gear ratios rather than being a property of the instrument, in which
   case using it unmodified through our adapter would be silently wrong.

   MOPS settles it. Their coupling lives in `eua_control/eua_control/__init__.py`, not in the
   URDF, which is why an earlier survey of this repo concluded they had no coupling matrix —
   that was wrong. What matters is *how* they factor it:

   ```python
   class EUA1Transmission:
       t0 = 1.5     # motor-to-disc transmission (for all motors)
       t1 = 1.5632  # roll disc-to-joint transmission
       t2 = 1.0186  # wrist disc-to-joint transmission
       t3 = 1.2177  # grasper jaw disc-to-joint transmission (same for both jaws)
       w  = 0.6089  # coupling between wrist and grasper jaws (same for both jaws)

       K = t0 * np.array([[t1, 0,     0,   0 ],
                          [0,  t2,    0,   0 ],
                          [0,  t2*w, -t3,  0 ],
                          [0, -t2*w,  0,   t3]])
   ```

   The motor-to-disc factor `t0` is pulled out and multiplied in separately from the
   disc-to-joint terms. And `1.5632`, `1.0186` and `0.6089` are dVRK's coefficients exactly.
   So an independent group, driving through their own Dynamixel adapter, treated dVRK's
   numbers as the *instrument-internal* disc-to-joint transmission and supplied their own
   motor-to-disc ratio on top. That is the confirmation, and it is cleaner than a bench test
   because it shows the factorisation rather than just a working result.

   **What this means for us, concretely.** Take `t1`, `t2`, `t3`, `w` as the instrument —
   they transfer. Replace `t0` with the motor-to-disc ratio of the in-house adapter, which is a
   design parameter of the thing he built rather than anything to be measured or inherited.
   If the Dynamixels drive the discs directly, `t0` is 1.

   **The Xi numbers used in-house are almost certainly `EUA2Transmission`** in the same file:
   `t1 = 1.0`, `t2 = 1.0`, `t3 = 1.15`, `w = 0.75`, with no `t0` at all and the sign on `t3`
   flipped relative to EUA1. That answers where they came from — dVRK has no `470xxx` file, so
   this is MOPS's own Xi characterisation. Two things to confirm with the adapter's designer, because EUA2 having
   no `t0` is ambiguous: whether their Xi adapter is genuinely direct-drive, or whether the
   motor-to-disc ratio is folded into `t1`/`t2`/`t3`. If it is folded in, those numbers are
   adapter-specific and only worked for us by coincidence of similar gearing.

   Also worth noting from the same file: MOPS runs the EUA controller at 100 Hz with Reflexxes
   and MX28 servos, and carries a four-element `servo_calibration_offset` parameter. That last
   one is their equivalent of our Stage A zero, and it being a per-installation parameter
   rather than a published constant is independent confirmation that homing is adapter-specific
   and ours to do.

   **Licensing is unchanged by any of this.** MOPS has no LICENSE file, so nothing there may be
   copied wholesale either — but coefficients are facts, using them is not redistribution, and
   the plan remains to derive our own before publication or cite the source for the ones used.
3. **Disc ordering and signs will differ, and the forceps are unchecked.** The in-house adapter is
   not the ISI sterile adapter and has no reason to number or orient the discs the same way;
   note that EUA1 and EUA2 already disagree with each other on the sign of `t3`. Whatever he
   resolved for the scissors has to be repeated for the bipolar fenestrated forceps, which is
   the other reference-run instrument. Command each disc slowly and watch which joint moves, in which
   direction, before trusting any row of the matrix.
4. **End-to-end articulation check.** Command a known joint angle, measure what the wrist
   actually does with a protractor or a photograph. Agreement within a few degrees is enough
   for teleoperation; disagreement means one of the three points above is wrong.

**Homing is still ours and is still required.** dVRK publishes no homing procedure for these
instruments, and its engagement angles assume the PSM's engagement mechanism, which we do not
have. Stage A below is on the bring-up critical path — you cannot apply anyone's coupling
matrix without knowing where each disc's zero is. Conservative joint limits, likewise, are a
safety requirement rather than an accuracy one.

So the bring-up path is: Stage A homing, dVRK's published values bench-validated per the four
points above, conservative limits. Days of work. Everything else in this document is for the
paper's error budget and comes after the arm is teleoperating.

## What gets produced

One YAML per instrument, consumed by `arts_description` and `arts_dv_tool_driver`:

| Field | Stage | Units |
|---|---|---|
| actuator zero, per disc | A | rad |
| actuator travel limits | A | rad |
| actuator-to-joint coupling matrix | B | dimensionless |
| backlash band, per joint | B | rad |
| link geometry (DH or explicit frames) | C | m, rad |
| joint limits | C | rad |
| tip offset | C | m |
| visual mesh | D | m |
| collision primitives | D | m |

Every number carries an uncertainty and a pointer to the run that produced it.

---

## Stage A — actuator zero and travel limits

Everything downstream is referenced to the actuator zero, so its repeatability is the noise
floor on every parameter identified later. Establish it first and characterize it honestly.

**Method (current-based hard-stop bisection).** Drive one disc at constant low velocity until
motor current exceeds a threshold, sustained; record the position; reverse until the opposite
stop; take the midpoint. The existing in-house implementation on the monopolar curved scissors is
the reference. Note that the da Vinci X instruments have no published homing procedure at
all, so this is new regardless of the licensing question.

**Parameters to fix and report, because each of them moves the answer:**

- *Velocity.* Detection lags the true stop by an amount that grows with approach speed, since
  the disc has to decelerate. Use one slow velocity for every disc and every instrument, and
  report it. 5°/s at the disc is a reasonable starting point.
- *Threshold.* Set it as a multiple of the measured free-running current for that disc, not as
  an absolute mA figure — free-running current differs between discs and rises with cable
  tension. Require it sustained over a dwell window so a transient does not trigger.
- *Temperature.* Dynamixel current at fixed load drifts as the winding warms. Warm up with a
  fixed number of cycles before recording, and check for drift across the session.
- *Direction asymmetry.* Friction is not symmetric, so the two detected stops are not
  symmetric about the mechanical center. Report the asymmetry rather than hiding it in the
  midpoint.

**Deliverable.** N = 20 home cycles per disc, reporting mean, standard deviation, and
peak-to-peak. Then re-seat the instrument on the adapter and repeat, because *re-seating*
repeatability is the number that matters operationally and it will be worse than
back-to-back repeatability. Both go in the report.

**Zero conventions are per joint, not universal.** The hard-stop midpoint is right for roll,
pitch, and yaw. It is wrong for the jaw: both jaw stops are meaningful (closed-and-loaded
against itself, and full open), and the midpoint is a physically meaningless half-open pose.
Define jaw zero at closed-just-touching, detected as the current knee rather than the current
peak, and write the convention into the YAML so nobody has to re-derive it.

---

## Stage B — coupling matrix

The wrist is cable-driven, so disc rotations do not map one-to-one onto joint angles. dVRK
models this as a constant 4×4 `ActuatorToJointPosition`. Identifying it is a linear
regression, which makes this the easiest stage to do rigorously.

### The problem that rules out the obvious approaches

A da Vinci disc does not drive a joint. The wrist is a coupled cable transmission, which is
exactly why dVRK publishes a coupling matrix per instrument alongside the DH table. Driving
one disc moves several joints at once.

That kills two methods that look attractive on paper.

It kills **circle-point analysis as a primary method** — an earlier draft of this document
proposed it, and it was wrong. The idea was that actuating one joint sweeps the tip through a
circle whose fitted plane and centre recover the joint axis. It only works if single-disc
excitation produces single-joint motion, which is precisely what the coupling says is false.
The tip traces a compound path, the circle fit degrades, and you cannot design a
single-joint excitation without already knowing the coupling you are trying to measure. That
circularity has no clean escape.

It also kills **any method that assumes a known marker position at the tip.** Worth separating
this from the above, though, because it is a smaller problem than it looks: a tracker marker
never needs ideal placement. Its offset is an unknown you *solve for*, not a quantity you set.
The marker only has to be rigid and to move when the joints move. So "I cannot mount a sphere
at the exact tip frame" is not an obstacle — but it is not a rescue for circle-point analysis
either, because that method fails on the coupling, not on the mounting.

### What to do instead: structure first, then a direct linear fit

Three steps, each independently checkable, none depending on knowing the answer in advance.

**Step 1 — tear down an expired instrument.** Instruments are life-limited, so a used-up one
is available and costs nothing to sacrifice. Opening it settles by direct observation what no
amount of external fitting can: which disc drives which cable, where the cables route, which
pair is the differential, and the capstan and pulley radii. That last point matters more than
it sounds — **the coupling coefficients are pulley radius ratios**, so calipers on a torn-down
wrist give you the coupling matrix's structure exactly and its coefficients approximately,
before any experiment runs. It also gives link lengths measured on the inside rather than
inferred from outside, and it is completely clean: measuring an object we own.

This single step is what breaks the circularity. Once the structure is known — that discs 3
and 4 are a differential pair, say — you can design excitations that *are* single-joint
(move them together for one joint, oppositely for the other) without having fitted anything.

**Step 2 — measure joint angles optically at static poses, then regress.** Command a grid of
disc positions, photograph the wrist at each, and measure the joint angles in the images
against a scale reference. Pitch and yaw on a da Vinci wrist are large, visually obvious
articulations; roughly a degree of measurement accuracy is achievable from a photograph, which
over an 80-degree range is about one percent on a coupling ratio. That is well inside what
teleoperation needs, and averaging over a few hundred poses does better.

Then the coupling is an ordinary linear regression of measured joint angles against commanded
disc angles — `q_joint = C · q_actuator`, solved once, with no chicken-and-egg and no
iteration. This is the workhorse. It needs no tracker booking, no marker mounting, and no
assumption that any joint moves alone.

**Step 3 — validate against the tracker.** With C and the link geometry in hand, predict the
tip pose and compare against a fusionTrack measurement over a held-out trajectory. Mount a
multi-sphere geometry proximally on the shaft or housing, where there is room, and solve for
the marker-to-tip transform as free parameters in the fit. This produces the end-to-end
accuracy number for the paper. It is validation, not identification, which is why its rig
constraints no longer gate the whole protocol.

**Where circle-point analysis survives.** As a cross-check, after Step 1. Shaft roll driven by
its own disc is genuinely single-joint, and post-teardown the differential combinations are
too, so those axes can be fitted independently and compared against the regression. The code
in `arts_instrument_id/axis_fit.py` is kept for that. Do not use it before the teardown.

### Design of experiment

- *Structural excitations, post-teardown.* Combinations known from the mechanism to move one
  joint. These catch sign and wiring errors immediately, before any fitting.
- *Random multi-disc poses.* 100–200 poses spanning the reachable set, which is what the
  regression is actually fitted on, and which tests whether superposition — a constant matrix —
  is the right model at all.
- *Bidirectional sweeps.* Approach each point from both directions. The gap is backlash.

**Fit.** Stack the samples and solve `Q_joint = C · Q_actuator` by least squares. Report the
residual RMS, per joint, and the condition number of the actuator matrix — a poorly
conditioned excitation gives a confident-looking matrix that is mostly noise.

**Report backlash separately, do not fold it into C.** The bidirectional sweeps give a
hysteresis band in degrees at each joint. This is a number nobody in this space publishes,
it directly bounds the achievable tip accuracy, and it is exactly what a reviewer asking
"how good can an open adapter really be?" wants to see.

**Check whether C is actually constant.** Cable-driven wrists commonly show roll-dependent
coupling, because rolling the shaft winds the pitch and yaw cables. Repeat the pitch/yaw
sweeps at several shaft roll angles. If C varies with roll, a constant matrix is an
approximation — and quantifying that approximation is a finding worth a paragraph, not a
problem.

---

## Interlude — where is "the tip"?

This has to be settled before Stage C, because pivot calibration measures whatever point you
put in the divot, and "the tip" is genuinely different per instrument. Scissors cut along a
blade. Fenestrated forceps grasp across a fenestration. A vessel sealer seals a zone. A
stapler fires a line. None of those is a point, and picking a different one per instrument
would make the numbers incomparable.

Resolve it with one universal frame plus named per-instrument offsets.

**The universal tool frame, identical for every instrument.** Origin at the distal-most point
of the closed jaws, with the jaw at its zero (closed, just touching — the Stage A convention).
Axes: **z** distal along the closed-jaw centerline, **x** along the jaw pivot axis, **y**
completing right-handed.

This convention is chosen because it is exactly what pivot calibration physically yields: seat
the closed jaws in a divot, pivot the instrument through many orientations, and the stationary
point is the distal jaw point. Nothing to interpret. It is also unambiguous across every
instrument that closes, which is all of them, and both its axes are anchored to physical
features rather than to a modelling choice.

**Per-instrument functional frames**, published as fixed offsets from the tool frame in the
same YAML, so a task planner can ask for the right one by name:

| Instrument | Named frames |
|---|---|
| Monopolar curved scissors | `cut_point` (blade midpoint, where tissue actually parts), `electrode_extent` |
| Bipolar fenestrated forceps | `grasp_center` (fenestration center), `electrode_extent` |
| Vessel sealer | `seal_center`, `seal_extent` (the jaw length that seals) |
| Stapler | `staple_line_start`, `staple_line_end` |

**The control frame** used by teleop and by the RCM constraint defaults to the universal tool
frame, and is overridable live through the tool-tip offset that decision 11 exists to protect
— parameter into an RT-safe buffer, offset inside the pinocchio model so the Jacobian is
computed to the real tip. Naming the functional frames here is what makes that override a
documented choice at calibration time rather than a magic number typed during an experiment.

## Stage C — link geometry

**Coarse pass, calipers.** Shaft diameter, shaft length, clevis pitch-to-yaw offset, yaw-to-tip
length, jaw length. A digital caliper resolves 0.01 mm and is accurate to roughly 0.02 mm,
which is well inside what the QP needs. Do this first: it gives the initial guess for the fit
and an independent check on it. Two people measure independently and compare.

**Refined pass, kinematic calibration.** Mount a fusionTrack marker rigidly at the tip,
command M poses spanning the joint space, measure the tip pose at each, and fit the link
parameters by nonlinear least squares (Levenberg–Marquardt) minimizing pose error. This is
standard robot kinematic calibration; nothing here is novel except applying it to an
instrument whose parameters have only ever been transcribed.

Three details decide whether the result is trustworthy:

- **Solve for the marker offset too.** The transform from the true tip to the marker is
  unknown, and if you fix it at a nominal value that error lands in the link parameters
  instead. Either include it as free parameters in the fit or pivot-calibrate it first.
- **Do an observability analysis, and drop what is not observable.** Build the identification
  Jacobian, take its SVD, and report the singular values. Parameters in the near-null space
  are not identifiable from the poses you collected; fitting them anyway produces
  precise-looking numbers that are fitting noise. Fix them at the caliper value and say so.
  (Borm and Menq's observability index is the conventional citation.)
- **Hold out data.** Fit on 70% of the poses, report error on the remaining 30%. An
  identification paper that reports only training residual is not reporting anything.

**On disagreeing with dVRK — expect it, and compare the right thing.** DH parameter sets are
not unique: two different tables can produce identical kinematics, so comparing tables
directly will manufacture disagreements that do not exist. **Compare forward-kinematic tip
poses over a common joint trajectory**, and report the pose difference in mm and degrees.

Also expect genuine differences. dVRK's numbers are mostly for `400xxx` (S/Si) and `420xxx`
parts; the Xi variants are `470xxx`. These are different instrument generations. Record the
exact part number off every instrument before measuring it — an unlabelled parameter set is
worth much less than a labelled one, and a mismatch traced to a generation difference is a
result rather than an error.

---

## Stage D — meshes

**Visual, by photogrammetry.** Polished stainless is the worst possible photogrammetry
subject, so the matting step is not optional: a vanishing scanning spray (AESUB Blue
sublimates in a few hours and leaves no residue, which matters for an instrument that goes
back into use) or, at a pinch, dry shampoo. Then a turntable, diffuse lighting, and 60–100
images per revolution at three elevations. COLMAP is the reconstruction tool of choice here —
BSD-licensed, which keeps the pipeline's own license story clean, whereas Meshroom is MPL.

Photogrammetry produces no absolute scale. Set it from a caliper-measured feature; the shaft
diameter is ideal because it is a precisely manufactured cylinder. Record which feature was
used and its measured value, because that single number scales the entire mesh.

Decimate to roughly 5–20k triangles. The mesh is ours: we photographed our own instrument.

**Collision, by primitives.** Capsules for the shaft and links, boxes for the housing, from
the Stage C caliper numbers. Same reasoning as the arms in `parts/2_arms/NOTES.md` — closed-form
capsule distance is what the 500 Hz QP wants, and a decomposed mesh of a tube is just a worse
capsule. MOPS reached the same arrangement independently: tip meshes, box housing, cylinder
shaft.

**Scope honestly.** Scissors and forceps get scanned tips because they are the reference-run
instruments and the jaw geometry matters for the task. The vessel sealer and stapler get
primitives until their own papers need better.

---

## Stage E — validation

Four numbers, in increasing order of what a reviewer will care about:

1. **Held-out pose error.** RMS and max, from the Stage C split.
2. **Agreement with published values.** FK tip pose against dVRK's parameters over a common
   trajectory. Agreement corroborates; disagreement gets attributed to a generation
   difference, a frame convention, or our uncertainty — and we should be able to say which.
3. **Cross-instrument variance.** Identify two instruments of the same type and compare. This
   is manufacturing variance in a consumable surgical instrument and, as far as the survey
   found, nobody has published it. It also bounds how much a single-instrument parameter set
   can be trusted by someone else.
4. **End-to-end tip accuracy.** UR arm plus adapter plus instrument, tracked tip against
   commanded tip. This is the number that goes in the paper and the one that feeds the
   evaluation table in `AGENTS.md`.

---

## Stage F — packaging

`arts_ros2_ws/src/arts_calibration/arts_instrument_id`, following the same contract as the
benchmark harness in decision 10 — capture, offline analyze, commit the report next to the
raw data:

```
arts_instrument_id/
├─ capture/      launch files: homing repeatability, single-disc sweeps,
│                random poses, tracker recording
├─ identify/     coupling least squares, DH Levenberg-Marquardt,
│                observability analysis, backlash extraction
├─ report/       markdown + json, same shape as docs/benchmarks/
└─ config/       per-instrument capture parameters
```

Outputs land in `arts_description/config/instruments/<instrument>.yaml`. Raw data is committed
— it is pose streams, not images, so it is small — which is what makes the numbers auditable
rather than merely asserted.

---

## Effort and staging

First instrument is the expensive one; the marker jig and the scripts amortize.

| Stage | First instrument | Each after |
|---|---|---|
| Teardown (Step 1, once) | 1 day, shared across instruments of the same family | — |
| A — homing | 1–2 days (in-house method exists, needs stats) | hours |
| B — coupling | 2–3 days (optical measurement, then regression) | ~1 day |
| C — geometry | 3–5 days | 1–2 days |
| D — meshes | ~1 day | ~1 day |
| E — validation | ~2 days | ~1 day |

Roughly two weeks for the first instrument, three to four days for the second.

**Staging.** Before first teleoperation: teardown, Stage A complete, structural coupling with
approximate coefficients, conservative joint limits. Nothing else. After that and before the
ISMR submission: Stage B regression in full, Stage C with observability, Stage D
photogrammetry, Stage E validation.

The paper needs the error budget. Bring-up needs the robot to work. Those are different jobs,
and getting the second one wrong makes the first one impossible to interpret — which is the
ordering argument, independent of any deadline.
