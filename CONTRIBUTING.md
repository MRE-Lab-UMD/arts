# Contributing to ARTS

Read `AGENTS.md` first. It carries the locked architecture decisions, the two-tree layout, and
the current phase status, and it is authoritative over anything written here.

This file records only the rules a contributor has to follow that are *not* obvious from the
code, plus the two rules that are easiest to break by accident.

## Before you write anything

1. **`AGENTS.md` is the onboarding file.** Locked decisions are not re-litigated; if you think
   one is wrong, say why in a new dated amendment rather than quietly working around it.
2. **Work starts at the first unchecked item in the phase order.** Each phase must leave a
   working system.
3. **Pre-existing code from another project does not go straight into `arts_ros2_ws/src`.** It
   is staged in `$ARTS_LOCAL/code_intake/<module>/0_incoming/`, read, given a per-part verdict
   in a `NOTES.md` (PORT / REWRITE / REFERENCE-ONLY / ADOPT / DISCARD), and then re-authored
   against the locked decisions. The job is evaluation, not transcription: a port that
   faithfully reproduces a design nobody re-examined inherits its assumptions too, and those
   were made for a different distro, different hardware and a different deadline.

## Two rules that are easy to break by accident

### Anonymity

This repo becomes a double-anonymous code capsule for peer review, so **no tracked file may
contain a personal name, email address, username, hostname, institution, collaborator or
private repository URL.** That includes places you would not think of: `package.xml`
maintainer fields, `setup.py`, generated benchmark filenames, and documents received from
elsewhere (a structural review was itself removed from `docs/` in Aug 2026 for exactly this).

Published prior art is a deliberate exception and is *encouraged*: cite MOPS, dVRK, the open
laparoscopic platform, and any other published system by name. The rule bans identifying **the
authors of this work**, not the literature. The prior in-house system is referred to as "the
prior system" or "the v1.4 reference", never by its acronym.

Machines are named by class, never by hostname — `devlaptop-6c12t`, `ws-16c32t`. Benchmark
harnesses must derive that from topology or from `ARTS_MACHINE_CLASS`, never from
`hostname`, or the leak regenerates itself on the next run.

Check before you commit:

```bash
./deploy/ci/check_anonymity.sh
```

### Pinning

Every dependency is pinned, and each class has exactly one home — third-party ROS packages in
`middle_overlay_ros2_wss/<ws>/<ws>.repos`, non-ROS libraries in
`open_source_software/<lib>/VERSION`, OEM SDKs in `oem_drivers/<sdk>/VERSION`, apt packages
with `=version` in `deploy/modules/<module>.sh`, Python with `==` in a `requirements.txt` next
to its consumer. Never a branch: a tag, preferably a full commit hash.

**Bumping a pin is a benchmarked act**, not an edit: change the pin, rebuild, re-run that
module's benchmark, and commit the new baseline in `docs/benchmarks/` beside the old one.

Two ways a pin gets defeated that are worth knowing before they cost you a day. An apt package
you never asked for can install a `<lib>Config.cmake` into `/opt/ros/jazzy` and win
`find_package` silently (this happened with Ruckig). And an upstream `CMakeLists.txt` can
download its own dependency when it cannot find yours — `spinnaker_camera_driver` fetches a
*different* Spinnaker version from a third-party host if the OEM install was skipped, so "it
built fine" is not evidence that the pin held. Where a mismatch would be silent, assert the
version at configure time and fail the build loudly.

## The module contract

Every module owns one `deploy/modules/<module>.sh` exposing `<module>_host_deps`,
`<module>_oem_install` and `<module>_overlay_build`. Both the bare-metal installer and the
Dockerfiles consume the same functions, which is the only reason the two install paths cannot
drift — so **a function used by both paths must contain no container housekeeping.** `flir.sh`
once ended `oem_install` with `rm -rf /var/lib/apt/lists/*`, correct in a Dockerfile and
destructive on a host, and the resulting failure surfaced two functions later in a completely
unrelated package.

Both paths are first-class. A contribution that works only in Docker, or only bare metal, is
not finished.

## Sourcing

Source **only the immediate underlay**; ROS 2 `setup.bash` files chain downward on their own.
Sourcing `/opt/ros/jazzy` *and* an overlay works by accident in simple cases and breaks when
paths conflict. After building any overlay, check that `/opt/ros/jazzy` appears exactly once
in `AMENT_PREFIX_PATH`, and last. It is a cheap check that has caught real bugs.

## Documentation

Four layers, one job each; putting a fact in two layers is how they end up disagreeing.

| Layer | Where | Its only job |
|---|---|---|
| 1 | `AGENTS.md` | why the architecture is what it is, and where work stands (**authoritative for status**) |
| 2 | `docs/modules/<module>.md` | what a module is: artifacts, stages, per-stage results |
| 3 | `docs/modules/<substep>.md` | do this at the bench, in time order |
| 4 | `docs/benchmarks/<name>_<date>.md` | one measurement session, immutable once committed |

A result is written in exactly two places: the benchmark report in full, and a one-line
headline beside its checkbox in `AGENTS.md`. Tick the checkbox in the **same edit** that
records the result.

Never cite a machine-local path (`$ARTS_LOCAL/...`, `reference/...`) from a tracked document
without saying that it is machine-local — a reader of the repo cannot resolve it.

## Commits

Explain **why**, not what. Where a change corrects an earlier assumption in this repo, say so
and say what the assumption was: the failures worth documenting here are overwhelmingly the
ones that produced a plausible wrong number rather than an error message, and the reasoning is
the only thing that stops them recurring.

Do not explain a change in a code comment. Comments state constraints the code cannot show.

## License

ARTS is released under Apache-2.0 (`LICENSE`, verbatim). Copyright is held by the ARTS
authors collectively, as recorded in `NOTICE`; no institution is named as the holder.
Contributions are accepted under Apache-2.0 on that understanding.
