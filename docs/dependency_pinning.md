# Dependency pinning policy — full text

> **Role: reference.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.


An unpinned open-source dependency WILL break the build between machines or overnight; this
happened in the prior system when coal's API changed between two builds a day apart. With >100
libraries in the chain, chasing HEAD is not viable. Pin a verified release and stay on it
until a concrete problem forces a bump.

Five rules, one home per dependency class:

| Class | Pinned in | Example |
|---|---|---|
| Third-party ROS packages | `middle_overlay_ros2_wss/<ws>/<ws>.repos` | `Universal_Robots_ROS2_Driver` @ tag 2.2.15 |
| Open-source non-ROS libs | `open_source_software/<lib>/VERSION` | `coal` @ full commit hash |
| OEM SDKs | `oem_drivers/<sdk>/VERSION` | `spinnaker` @ 4.2.0.88 |
| apt packages | `deploy/modules/<module>.sh` | `libeigen3-dev=3.4.0-2ubuntu2` |
| Python packages | `requirements.txt` next to the consuming package | `torch==2.3.1` |

1. **Never a branch.** `.repos` entries and VERSION files use a tag or, preferably, a full
   commit hash. `version: main` / `devel` is banned. Tags are acceptable only because most
   projects do not retag; commit hashes are bulletproof.
2. **Non-ROS C++ libraries are pinned too, not just ROS packages.** coal, pinocchio and
   proxsuite each get `open_source_software/<lib>/{build.sh, VERSION}`. (Ruckig is the
   exception and keeps a VERSION with no build.sh — it comes from apt; see the resolved
   note below.) Watch for
   transitive `FetchContent(... GIT_TAG main)` inside a dependency's CMakeLists — override
   it explicitly in `build.sh` rather than inheriting upstream's floating pin.
3. **apt gets `=version`** in the module script, so the bare-metal and Docker paths resolve
   identically months later.
4. **Python gets `==`**, generated with `pip freeze` from a working environment.
5. **Anything that cannot be pinned by git/apt/pip goes to `oem_drivers/` with its archive
   mirrored under `~/arts_local/`** and the exact version recorded — the prior system's `Software/`
   pattern, which existed for precisely this reason.

Bumping a pin is a deliberate, benchmarked act: change the VERSION/`.repos` entry, rebuild,
re-run that module's benchmark, and commit the new baseline in `docs/benchmarks/` next to
the old one.

**A pinned version can still be defeated by an apt package you never asked for.** On
2026-08-10, building `ur_moveit_config` installed `ros-jazzy-ruckig` 0.9.2 into
`/opt/ros/jazzy` as a `moveit_core` dependency. ARTS pins Ruckig v0.14.0 to a separate
prefix under `arts_local`, so nothing was overwritten — but `find_package(ruckig)` from
`arts_controllers` would have resolved to 0.9.2, five minor versions old, in the 500 Hz
trajectory smoother, with no error. Removing MoveIt (decision 11) eliminated it; a full
audit afterwards confirmed exactly one Ruckig on the machine and no vendored, pip, or
`/usr/local` copies. Two rules follow. First, when activating any module, audit what its
apt dependencies install into `/opt/ros/jazzy` against the `open_source_software/*/VERSION`
list, not just what it builds. Second, `arts_controllers` must assert the Ruckig version at
CMake configure time so a mismatched copy fails the build loudly — the coal incident and
this one are the same failure shape, and an assertion is the only thing that makes it loud.
DONE 2026-08-12: that assertion is implemented in `arts_controllers/CMakeLists.txt`.
MoveIt also brought FCL 0.7.0 and OSQP, functional twins of ARTS's coal and proxsuite;
those are not name collisions (`find_package(coal)` cannot match `fcl-config.cmake`) so
they were redundant rather than dangerous, but note coal IS the renamed hpp-fcl, an FCL
fork, so the overlap is real.

**coal and pinocchio are the OPPOSITE call from Ruckig, and deliberately so (2026-08-12).**
Both are built from source into `$ARTS_LOCAL` at their pinned tags (coal v3.0.4, pinocchio
v3.9.0), even though apt now offers `ros-jazzy-pinocchio` 4.0.0, `ros-jazzy-coal` 3.0.3 and
`ros-jazzy-proxsuite` 0.6.5. Ruckig went to apt because it is frozen at 0.9.2 across every
distro, there was nothing to gain from a newer version, and the apt copy REMOVED a
second-copy hazard. None of that holds here: apt is already a major version ahead of the
pinocchio pin with breaking API changes, so these are moving rather than frozen; they
produce numbers we publish; and apt's pinocchio would drag apt's coal into `/opt/ros/jazzy`
where it would shadow the pinned coal — creating the hazard rather than removing it.
`arts_controllers/CMakeLists.txt` asserts all three versions at configure time.
Three integration details that cost time and will cost it again if forgotten:
- pinocchio v3.9.0 still calls `find_package(hpp-fcl)`, so coal must be built with
 `COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON` (off by default) or pinocchio's configure
 fails claiming hpp-fcl is missing while coal — the very library it wants — is installed.
 Consequence: the coal prefix now answers to BOTH names, so apt's `ros-jazzy-hpp-fcl`
 could shadow it, which is why coal is asserted too.
- pinocchio must be built with `CMAKE_INSTALL_RPATH_USE_LINK_PATH=ON`. The linker emits
 RUNPATH, and RUNPATH is NOT consulted for a dependency's own dependencies, so a consumer
 listing coal's directory in its own RUNPATH still fails to load pinocchio. The symptom
 is a library that links cleanly and then cannot be dlopen'd — and because pluginlib does
 the loading, it surfaces as "controller type not found", which sends you looking in
 entirely the wrong place.
- These prefixes are outside every ament prefix, so sourcing overlays does not reveal
 them. `build_arts_ws()` in `arts_install.sh` prepends them to `CMAKE_PREFIX_PATH`.

**RESOLVED 2026-08-12 — Ruckig comes from apt at 0.9.2, and is NOT built from source.**
`open_source_software/ruckig/build.sh` was deleted so nobody can recreate the hazard; the
VERSION file in that directory is now a decision record carrying the full evidence. Four
findings decided it. (1) There is nothing to buy: `position_third_step2.cpp`, the solver
where the reported failures live, is unchanged modulo renames between 0.9.2 and 0.14.0,
and the two versions plus 0.15.3 give bit-identical results over 1.4M randomized trials.
Upstream says plainly that numerical robustness is a Pro-edition feature and not to expect
Community fixes. (2) 0.14.0 was the WORST of the three: a 2023 refactor put a
`std::function` in `Trajectory::at_time()`, which `update()` calls every cycle, so it
heap-allocates once per control cycle — measured 0 / 1 / 0 allocations for
0.9.2 / 0.14.0 / 0.15.3, fixed in 0.15.3. (3) Using the apt copy makes the shadowing
failure structurally impossible, since `/opt/ros/jazzy` is the underlay and there is then
no second copy to prefer. (4) ros-jazzy-ruckig has been 0.9.2 in every distro since 2023
and the bump request is still open, so it is frozen in practice.
Do NOT pin it with apt `=`: the Debian revision changes on rebuilds while upstream stays
0.9.2, which is the same rolling-repo trap as `ur_description`. The guard is instead a
hard version assertion at CMake configure time in `arts_controllers`, now implemented —
that is the "make it loud" rule from the incident above, discharged. Verified on the
machine 2026-08-12: `ros-jazzy-ruckig` 0.9.2-5noble installed, `arts_interfaces` and
`arts_controllers` build clean against the `ur_ros2_ws` underlay, `ruckig_DIR` resolves to
`/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/ruckig`, and `/opt/ros/jazzy` appears exactly
once in `AMENT_PREFIX_PATH`. The assertion itself was exercised against synthetic 0.9.2
and 0.15.3 config packages, so the failure path is known to fire, not just the pass path.
If it ever goes back to source, use 0.15.3, never 0.14.0, and never 0.17.3+ (those make
`cxx_std_20` a PUBLIC compile feature that would propagate C++20 into `arts_controllers`).
Two things learned that shape Phase 1: Ruckig's tracking interface is Pro-only, so
continuous re-targeting is the plain `update()` loop, and upstream warns it always lags
the signal. MoveIt's `ruckig_filter.cpp` on Jazzy compensates by forward-extrapolating
`target_velocity` from the current state each cycle. DO NOT COPY THAT — tried 2026-08-12
and it drove the arm backwards at the speed limit; the reason, and what ARTS does
instead, are under Phase 1 in the status section below.

**Re-measured 2026-11-01 (apt versions move; the reasoning above stands).** In a fresh
`ros:jazzy-ros-base-noble` image, `ros-jazzy-pinocchio` is now 4.1.0 (was 4.0.0),
`ros-jazzy-coal` still 3.0.3, and `ros-jazzy-proxsuite` is 0.7.3 (was 0.6.5), i.e. the SAME
version as the source pin. So the proxsuite configure-time assertion can no longer tell the apt
copy from the source build; the prefix order in `arts_ws_build` (`deploy/modules/arts.sh`) is
what selects the source build. The pinocchio assertion was seen firing against 4.1.0.
