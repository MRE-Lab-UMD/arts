# arts_instrument_id

Measures a da Vinci X instrument's kinematics, actuator coupling, backlash and geometry from
the physical instrument, and emits the YAML that `arts_description` and `arts_dv_tool_driver`
consume.

Protocol, rationale and effort estimates: **`docs/instrument_identification.md`**.
Why nothing is reused from dVRK or MOPS: catalog `parts/4_tools/NOTES.md`.

## Why this package exists

Every published da Vinci instrument parameter set in this field is a transcription with no
stated uncertainty. dVRK's tool files say "values from the dVRK user guide"; MOPS's shaft
length is a comment in a xacro. Neither reports how the number was obtained, how repeatable
it is, or how well it predicts where the tip actually goes.

The licensing problem (both projects' instrument geometry is unlicensed and cannot be
redistributed) is what forced the question. The answer turned out to be better than the thing
it replaced, which is why the protocol ships as a contribution rather than as a workaround: a
reader with an instrument we do not own can run it and get their own numbers.

## Structure

```
arts_instrument_id/    analysis, hardware-free and unit-tested
  homing.py            Stage A — repeatability statistics over hard-stop cycles
  axis_fit.py          circle-point analysis: joint axis and angle from a swept point
  coupling.py          Stage B — coupling matrix, and backlash reported separately
  frames.py            pivot calibration and the universal tool frame
  geometry.py          Stage C — observability; the LM fit is a documented stub
  report.py            markdown + json + the instrument YAML
capture/               launch files; STUBS, blocked on arts_dv_tool_driver
config/                per-instrument capture setup and frame conventions
data/                  committed raw captures
test/                  synthetic-data checks, no hardware
```

Capture and analysis are deliberately separate processes. Analysis reads committed data, so
any published number can be re-derived by a reviewer with no robot on the bench. Same contract
as the benchmark harness in `AGENTS.md` decision 10.

## Running the tests

```bash
cd arts_ros2_ws/src/arts_calibration/arts_instrument_id
python3 -m pytest test/ -q
```

No ROS environment needed — numpy and pytest only.

## Status — DEFERRED until the tool drive is teleoperating

Do not start this work now. It is weeks, and nothing that has to work first needs it: a human
watching the tool through the endoscope corrects for modest kinematic error, so teleoperation
needs a *safe and honest* model rather than an accurate one.

(This section used to say "deferred until after the 5 October demo". That deadline was never
ARTS's — the funder demo runs on the legacy Ubuntu 22.04 + Humble stack, per AGENTS.md, "Two
separate deliverables" — so the ordering below is a dependency, not a calendar.)

**What bring-up actually needs** — days, not weeks:

1. **dVRK's published DH parameters and coupling matrices.** They are the only validated
   numbers for these instruments, so starting from them keeps a misbehaving wrist attributable
   to ARTS's drive rather than to two unknowns at once. Using them is fine — the licensing
   boundary is redistribution, not use —
   provided they live in a clearly-attributed third-party file held in `$ARTS_LOCAL` or
   gitignored, never shipped in the published tree under ARTS's own license.
2. **A bench check of those values against our adapter**, because they are validated for
   dVRK's hardware and not ours. Match part numbers (`470xxx` Xi variants have no dVRK file);
   confirm whether dVRK's actuator space is disc rotation or PSM motor rotation; verify disc
   ordering and signs against the in-house adapter; then command a known joint angle and measure
   what the wrist does. An afternoon.
3. **Stage A homing.** Still ours and still required — dVRK publishes no homing procedure for
   these instruments and its engagement angles assume the PSM's mechanism. You cannot apply
   anyone's coupling matrix without knowing where each disc's zero is.
4. **Conservative joint limits.** A safety requirement, not an accuracy one.

**Everything else, after that:**

| Stage | State |
|---|---|
| Teardown | not started; unblocks everything else, needs one expired instrument |
| A — homing | analysis done; capture blocked on `arts_dv_tool_driver` |
| B — coupling | analysis done; capture blocked on `arts_dv_tool_driver` |
| C — geometry | observability done; refinement blocked on the pinocchio model (Phase 2) |
| D — meshes | deferred; primitives until then |
| E — validation | not started |

## A method that was tried and rejected

An earlier draft made circle-point analysis the primary identification method: actuate one
joint, fit a circle to the swept tip, recover the axis. It does not work here. A da Vinci disc
does not drive a joint — the transmission is coupled, which is the entire reason a coupling
matrix exists — so single-disc excitation moves several joints and the path is not a circle.
You also cannot design a single-joint excitation without already knowing the coupling you are
trying to measure.

The replacement is structure-first: tear down an expired instrument to establish cable routing
and pulley ratios by direct observation, then measure joint angles optically at static poses
and fit the coupling by ordinary linear regression. No circularity, no tracker, no marker
mounting problem. `axis_fit.py` survives as a post-teardown cross-check only.

## Two things that are easy to get wrong

**Do not transcribe dVRK's tool JSONs**, even as a shortcut or a starting guess. Their values
come from Intuitive's account-gated user guide, and the CISST license they ship under states
it "does not grant any rights with respect to third party software." Derive independently;
independent derivation is the defense, and the numbers coming out close is corroboration.

**Compare tip poses, not parameter tables.** DH parameter sets are not unique — two different
tables describe identical kinematics — so diffing tables manufactures disagreements that do
not exist. Compare forward-kinematic tip pose over a common joint trajectory and report the
difference in mm and degrees.
