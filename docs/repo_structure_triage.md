# Repo structure review — ARTS triage and verdicts

**Doc layer 1 of 4 — decision record.** Companion to `AGENTS.md`, which stays authoritative
for locked decisions and status; this file records what ARTS accepts, scopes, defers or
rejects from an external structural review, and why. Verdicts that change a locked decision
are also written into `AGENTS.md`. Layer 2 = `docs/modules/<module>.md`, layer 3 = bench
docs, layer 4 = `docs/benchmarks/`.

**Date of triage:** 2026-08-20. **Review received:** 2026-08-19.

## Where the reviewed document is, and why it is not in this repo

The review was produced by an independent model against this tree and `mops_ws` (a working
prototype of the same three-arm platform, on another machine). It is staged, unedited, at
`$ARTS_LOCAL/code_intake/repo_review/0_incoming/repo_structure_review_20260819.md`, reachable
through the gitignored `reference/repo_review` symlink.

It was briefly tracked as `docs/repo_structure_review.md` and has been removed, for the
reason the review itself raises: **the document reporting the identity leak contained 15
instances of the maintainer's name, email, GitHub handle and lab affiliation, and was
therefore itself adding the exact contamination its own item 1 is about.** It also cites
`~/mops_ws` paths that no reader of this repo can resolve, which is its own item 5. Handling
it as intake material rather than as a repo document is the consistent application of the
rule already used for `esu_tap`: received material stays machine-local, only the verdict is
committed. Every item is restated below, so nothing is lost to a reader without the symlink.

## Conflict of interest, stated up front

Most of the structure being reviewed was authored in this agent session. That makes this
triage a case of marking one's own homework, which is why each verdict below carries its
evidence and its reason rather than a bare judgement, and why the three items involving real
design tension (6, 9, 10) are called out as wanting a second opinion rather than settled
here. An independent re-review is *not* recommended — the finding work is done, and its value
came from an independence that cannot be reproduced by re-running it here.

## Verdicts

Every factual claim below was checked against the tree on 2026-08-20. Two of the review's
claims needed correction; both are noted at their item.

| # | Item | Verdict | When |
|---|---|---|---|
| 1 | Scrub identity leaks | **ACCEPT** | now |
| 2 | `LICENSE`, `CONTRIBUTING.md`, CI | **ACCEPT** (CI already a Phase 0 checklist item) | now / Phase 0 |
| 3 | One overlay-build mechanism | **ACCEPT** — sharpest find in the review | before the new-PC install |
| 4 | Make "module" mean one thing | **ACCEPT (scoped)** — table, not parsed YAML | with item 3 |
| 5 | Documented layout matches disk | **ACCEPT in part, REJECT `deploy/udev/`** | now |
| 6 | Split `AGENTS.md` | **ACCEPT (scoped)** — extract narrative, not decisions | after FLIR exit review |
| 7 | Halve the README | **ACCEPT** | now |
| 8 | Prune stubs; fix name collision | **ACCEPT collision, REJECT deletion** | now / at intake |
| 9 | De-duplicate description/driver | **ACCEPT (scoped)** — must not collapse decision 13 | Phase 2 |
| 10 | Single `system_map.yaml` | **ACCEPT (scoped)** — must not absorb calibration | Phase 2/4 |
| 11 | OEM stubs fail loudly | **ACCEPT** | now |
| 12 | Real `example_app` | **ACCEPT, DEFER** | Phase 6 |
| 13 | `launch/` everywhere, one `setup.py` | **ACCEPT** | now |
| 14 | Sweep the small lies | **ACCEPT** | now |
| — | Appendix: `mops_ws` migration map | **ACCEPT — highest value in the document** | via `code_intake` |

## The items that needed a judgement rather than a yes

### 3 — one overlay-build mechanism (the sharpest find)

Verified: five of six `build.sh` scripts re-implement the `env -i` / `vcs import` / `rosdep` /
`colcon` sequence independently (`ur` 54 lines, `oem` 28, `atracsys` 23, `isaac` 22,
`slicer` 22); only `flir` (13 lines) delegates to its module function. And `arts_install.sh`
contains no call to any `*_overlay_build` — the only occurrence of the string is a **comment
asserting that Docker runs the same functions**.

This outranks the rest because of what it is, not its size: decision 4's no-drift property is
the justification for the whole module contract, and it currently holds for exactly one
module while a comment claims it holds generally. A false comment is worse than a missing
feature, because it stops the next person from checking.

Accepted as: extract the shared sequence into `deploy/modules/_common.sh` as
`overlay_build_generic <ws>`, reduce every `build.sh` to a delegate, and make
`arts_install.sh` call the module function when one exists. **Sequenced before the
production-workstation install**, because that install is the only real test of the written
recipe and it is worth spending on the fixed version rather than discovering this again.

**DONE 2026-09-08**, as accepted: `_common.sh` (`overlay_build_generic`, `overlay_build_dispatch`,
`overlay_owner_module` — the latter is the "table" item 4 asked for), six one-call `build.sh`
delegates, `flir_overlay_build` / `camera_sync_overlay_build` delegating, `arts_install.sh`
dispatching, `build_flir.sh` + `Dockerfile.flir` staging the shared file. Verified from a
scratch tree for `flir_ros2_ws` (full build via the installer) and `ur_ros2_ws` (full build,
6 packages, MoveIt excluded; the rosdep stage needs interactive sudo for `liburdfdom-tools`
exactly as before). Record in AGENTS.md "Build order".

### 5 — `deploy/udev/` is rejected, and the diagram is what changes

Verified absent: `deploy/udev/`, `deploy/ci/`. Accepted for `deploy/ci/` (item 2) and for
tracking the `code_intake` protocol as a doc rather than as `AGENTS.md` prose pointing at a
gitignored directory.

`deploy/udev/` is rejected on design grounds. Rules are currently written by the module that
needs them, inside `<module>_host_deps` — `flir.sh` writes the camera rules, `camera_sync.sh`
writes the one that creates `/dev/arts_camera_sync`. A directory of loose rule files would
compete with that for ownership, and the FLIR bring-up already produced the cautionary
version of this: rules that `dpkg-deb -x` bypassed were merely *warned* about until ARTS took
over writing them, which is what made a fresh bare-metal install have root-only cameras while
the privileged Docker path hid it. One owner per rule is the property worth keeping. The
AGENTS.md tree diagram is corrected instead.

### 6 — split `AGENTS.md`, but along a different seam than proposed

Verified: 2265 lines, 185 KB, edited in essentially every commit. The complaint is fair and
the observation that it violates the repo's own documentation contract is fair.

The proposed remedy — one file per decision in `docs/decisions/` plus a root `STATUS.md` — is
not accepted as stated, for a reason specific to what this file is *for*. It is the agent
onboarding file, and its value is that one read delivers every locked decision and the current
status together; a fresh agent handed an index of fifteen ADRs reads the index. The failure
this repo has actually suffered is not "the file is long", it is status drifting across three
internally-consistent documents — which is what produced the documentation contract in the
first place, and which splitting the status *out* would risk re-creating.

The seam that does work is genre, not decision count: what has grown without bound is the
**dated amendment narrative**, and layers 2 to 4 already exist to hold it. Decision 5, 14 and
15 carry roughly 1500 words of amendment each, and the FLIR Stage 5 checklist is ~600 lines of
bring-up log inside a status list, while `docs/modules/pilot_flir_camera.md` exists precisely
for that. Accepted as: move per-module narrative to the module doc, leave the decision and a
one-line result headline beside the checkbox — which is what the documentation contract
already mandates and which nothing but habit has prevented. Realistic target is ~1000-1200
lines, not the 500 proposed.

**Deferred until after the FLIR exit review.** Rewriting the status list in the middle of the
module it is tracking is how the drift happened before.

### 8 — the collision is fixed, the stubs stay, and the review miscounted

Verified: `arts_calibration/arts_calibration/` is a package sharing its group directory's
name. Accepted; flatten to `src/arts_instrument_id/` and drop the group, which holds two
packages.

Deleting the stubs is rejected. Eight packages are 10-line CMake stubs, and that is a real
smell, but they are the anchors that keep `colcon list` aligned with the phase checklist; the
alternative is that whoever implements one scaffolds it ad hoc, which is the convention drift
the review is otherwise arguing against. The review's own middle ground is adopted: a 3-line
README per stub naming the phase that fills it.

**Correction to the review:** `arts_record` is not a stub. It carries the entire benchmark
harness (`bench/run_flir_bench.sh`, `flir_bench.launch.py`, `analyze_bench.py`,
`mcap_image_config.yaml`) that produced every committed FLIR baseline. Its actual defect is
the opposite of the one reported — the content exists and the 10-line `CMakeLists.txt`
installs none of it, so the harness can only be run from the source tree. That is worth
fixing and would not have been found by counting lines.

### 9 — de-duplicate the configs, do not merge the descriptions

Verified: two `initial_positions` files (`arts_description/config/initial_positions.yaml` and
`arts_driver/config/ur5e_initial_positions.yaml`) and two xacro entry points.

The duplicated joint values are a genuine defect and are accepted for fixing — one source,
consumed with prefix substitution.

The two xacro entry points are **not** a defect to merge away, and the scoping matters. A
`controller_manager` consumes exactly one `robot_description`, and the tool arms run at 500 Hz
while the camera arm runs at 125 Hz, so per-arm control descriptions and a combined
viewing-only assembly are what decision 13 requires; a single document behind a
`hardware:=none|mock|real` argument would re-merge precisely what that decision separates.
What *is* accepted is narrower and already implied by decision 13: `arts_driver`'s
`arts_ur.urdf.xacro` is a Phase-1 stopgap that exists only to inject initial positions
(neither UR launch file forwards `initial_positions_file`), and it should be retired into
`arts_description`'s per-arm assemblies when those are authored — composed, not duplicated.

### 10 — one machine map, but calibration does not go in it

The `device_map.yaml` pattern is the strongest idea the review brings across from the
prototype, and the single-bringup-input direction is accepted.

One hard constraint on scope: it must **not** hold calibrated mount transforms. Decision 12
requires CAD-nominal and measured values to live in separate files, so that the
CAD-versus-measured discrepancy survives as a number the paper can report and so that a fresh
clone with no calibration still brings up a sane model. A single file holding both destroys
that the first time a calibration run writes to it. So `system_map.yaml` carries identity and
topology — serials, IPs, roles, USB paths, instrument class, homing thresholds — and measured
geometry stays in `arts_calibration/config/<frame>_correction.yaml`. The review's own
repo-default-plus-machine-local-override framing is compatible with this; the naive reading is
not.

Deferred to Phase 2/4, when there are three arms and a tool drive to configure. Building the
single source of truth before there is a second consumer would be guessing at the schema.

### Appendix — the `mops_ws` migration map is the most valuable part

More useful to ARTS than items 1 to 14 combined, because it is a concrete intake queue rather
than a cleanup list, and because it identifies first-party tested code that fills an empty
ARTS stub (`mops_collision_guard`, with `test_capsule.cpp`, into `arts_safety`).

It cannot be acted on from here: **`~/mops_ws` does not exist on this machine** (nor does the
`~/arts/arts` path the review used), so that review ran elsewhere. Acting on it means staging
the prototype through `code_intake` exactly as `esu_tap` was, with per-part verdicts in a
`NOTES.md`, and honouring the licensing split the review already flags — SDU's `mops_core`
ships no LICENSE, so it can only be re-implemented from behaviour and measured constants,
while first-party prototype code can be ported outright. The map also names a lab or company
in several rows, so whatever lands in the tree is scrubbed on the way in.

## What the review missed

Two defects found while checking its claims, neither of which line-counting would surface:

1. **`arts_record` installs nothing.** See item 8. The benchmark harness behind every
   committed FLIR baseline is not installed by its `CMakeLists.txt`.
2. **Nothing runs the tests that exist.** `arts_controllers/test/run_servo_smoke_test.sh`
   encodes six regressions that actually happened, and no automation invokes it. The review
   correctly notes there is no CI; the sharper point is that CI has a ready-made first job
   waiting for it.
