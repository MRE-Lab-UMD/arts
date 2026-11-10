# Repo and on-disk layout — full text

> **Role: reference.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.


There are two trees. The git repo holds only text: recipes, manifests, and first-party
source. Everything heavy (OEM SDKs, third-party source, build artifacts — ~142 GB on the
current PC) lives in a machine-local tree that is reconstructed from those recipes and is
never committed.

**Tree 1 — the git repo (`~/arts`, tens of MB):**

```
arts/
├─ oem_drivers/              manufacturer drivers/SDKs FOR PHYSICAL HARDWARE — recipes only.
│  │                         Criterion is "a physical component exists", not license: Intel's
│  │                         librealsense is open source but belongs here; Slicer does not.
│  ├─ spinnaker/             install.sh + VERSION   (FLIR cameras)
│  ├─ haply/                 install.sh + VERSION   (Inverse3 service, closed binary)
│  ├─ fusiontrack/           install.sh + VERSION   (Atracsys optical tracker)
│  ├─ zivid/  clarius_cast/  librealsense/          (as needed)
│  └─ COLCON_IGNORE
├─ open_source_software/     open-source NON-ROS software built from source — recipes only.
│  ├─ coal/                  build.sh + VERSION (pinned commit)
│  ├─ pinocchio/  proxsuite/                         (control math stack)
│  ├─ ruckig/                VERSION only, no build.sh — Ruckig comes from apt;
│  │                         the file is the decision record for why
│  ├─ slicer/  plus_toolkit/                         (imaging)
│  └─ COLCON_IGNORE
├─ middle_overlay_ros2_wss/  third-party ROS 2 workspaces — manifests + build scripts only.
│  │                         Named "middle overlays" deliberately: in ROS 2 "underlay" means
│  │                         the base (/opt/ros/jazzy). Each of these is an overlay of the
│  │                         base and an underlay for arts_ros2_ws. Each depends ONLY on the
│  │                         base, never on a sibling, and each builds in its own `env -i`
│  │                         shell.
│  ├─ ur_ros2_ws/            ur_ros2.repos + build.sh     (UR driver + description, pinned)
│  ├─ flir_ros2_ws/          flir_ros2.repos + build.sh   (FLIR spinnaker driver, image_common)
│  ├─ slicer_ros2_ws/        slicer_ros2.repos + build.sh (SlicerROS2)
│  ├─ atracsys_ros2_ws/      atracsys_ros2.repos + build.sh (cisst-saw + crtk, optical tracker)
│  ├─ isaac_ros2_ws/         isaac_ros2.repos + build.sh  (Newton ROS 2 bridge)
│  ├─ oem_ros2_ws/           oem_ros2.repos + build.sh    (loose OEM wrappers with no ws of
│  │                                                       their own: haply_inverse3_ros2,
│  │                                                       zivid-ros, robotiq, realsense-ros,
│  │                                                       micro-ROS agent)
│  └─ COLCON_IGNORE
├─ arts_ros2_ws/             THE top overlay. Same shape as a middle overlay; builds in place
│  │                         (build/ install/ log/ are gitignored). Users source
│  │                         `~/arts/arts_ros2_ws/install/setup.bash` and nothing else.
│  └─ src/
│     ├─ arts_interfaces/    msgs + srvs + actions (single package, ROS 2 convention)
│     ├─ arts_description/   URDF+SRDF, tools, trainer box, USD export
│     ├─ arts_calibration/   GROUP dir (like arts_utilities), feeds arts_description:
│     │                      arts_calibration/ (cannula/RCM + other procedures)
│     │                      arts_instrument_id/ (da Vinci X instrument kinematics,
│     │                      coupling, backlash, geometry; see
│     │                      docs/instrument_identification.md)
│     ├─ arts_driver/        bringup launch + configs, per-component toggles (see below)
│     ├─ arts_controllers/   ros2_control chainable controllers
│     ├─ arts_embedded/      FIRST-PARTY embedded bridges — custom PCB + ESP32/micro-ROS
│     │    arts_dv_tool_driver/ (da Vinci X disc drive, 4-motor ESP32 micro-ROS)
│     │    arts_esu/           (ESU-TAP PCB interposer: footswitch + software energy
│     │                        activation via ESP32 micro-ROS; sense-then-forward)
│     │    arts_camera_sync/   (ESP32 stereo trigger generator + ExposureActive capture;
│     │                        decision 14. Firmware + micro-ROS pulse messages)
│     ├─ arts_utilities/     arts_kinematics/  arts_safety/  arts_teleop/
│     │                      arts_record/ (MCAP profiles)  arts_datasets/ (→ LeRobot)
│     ├─ arts_sim/           ARTS-specific sim configs (Newton bridge in middle overlay)
│     └─ arts_slicer/        ARTS Slicer modules
├─ example_app_ros2_ws/  ONE example application overlay showing how to build on
│                        arts_ros2_ws without touching any middle overlay. COLCON_IGNORE.
├─ hardware/            first-party MECHANICAL designs, published with the code so the
│                        hardware is reproducible from the repo alone (PI, 2026-11-02):
│                        tool_adapter/ and laparoscope/ — assembly STEP (AP214), printed
│                        STLs, drawings, a README with the bought parts. Meshes for RViz are
│                        GENERATED from these (import_tool_step.py), never committed. About
│                        43 MB, rarely changed; no Git LFS (it complicates the anonymous
│                        capsule). PCBs stay with their firmware in arts_embedded/. COLCON_IGNORE.
├─ deploy/               COLCON_IGNORE
│  ├─ modules/<module>.sh   single source of truth per module: host_deps() /
│  │                        oem_install() / overlay_build(); consumed by BOTH
│  │                        arts_install.sh and the Dockerfiles so they cannot drift
│  ├─ docker/               Dockerfile per cart + compose profiles (one cached layer per
│  │                        middle overlay)
│  ├─ host_setup.sh         what a container cannot do: PTP daemons, NIC/MTU tuning, udev,
│  │                        CPU isolation, kernel/GRUB entries
│  ├─ bench/               rt_latency.sh and other host benchmarks
│  └─ ci/                   check_anonymity.sh today; the full-chain build is still to come
├─ docs/                 build guide, BOM, calibration, ADRs, benchmarks, proposal.
│                        COLCON_IGNORE.
├─ reference/            gitignored machine-local symlinks (v1.4 reference, install README,
│                        staged code_intake material). COLCON_IGNORE is MANDATORY — without
│                        it colcon walks the symlink and tries to build the reference system.
│                        DELETE before open-sourcing.
├─ LICENSE               verbatim Apache-2.0. The appendix's copyright-holder line is
│                        deliberately UNFILLED: naming the holder would break anonymity, so
│                        it is filled at de-anonymization. See CONTRIBUTING.md.
├─ CONTRIBUTING.md       the rules that are not visible from the code, and the two that are
│                        easiest to break by accident (anonymity, pinning)
└─ arts_install.sh       thin orchestrator over deploy/modules/*.sh
```

**`deploy/udev/` WAS IN THIS DIAGRAM AND HAS BEEN REMOVED RATHER THAN CREATED (2026-08-20),
which is a decision and not a tidy-up.** A structural review flagged it as documented-but-
absent, correctly; the fix is to delete the claim, because a directory of loose rule files
would compete for ownership with `<module>_host_deps`, where the rules actually live and
belong — `flir.sh` writes the camera rules, `camera_sync.sh` writes the one that creates
`/dev/arts_camera_sync`. The FLIR pilot already produced the cautionary version: rules that
`dpkg-deb -x` silently bypasses were merely WARNED about until ARTS took over writing them,
which is what left a fresh bare-metal install with root-only cameras while the privileged
Docker path hid the problem. One owner per rule is the property to keep. Full triage of that
review, item by item, is in `docs/repo_structure_triage.md`.

**Tree 2 — machine-local (`~/arts_local`, ~142 GB, never in git):**

```
~/arts_local/
├─ oem_drivers/                spinnaker-4.2.0.88/  haply/  fusiontrack_v4.7.1/  zivid-2.14.0/
├─ open_source_software/       coal/  pinocchio/  proxsuite/  Slicer-Superbuild-Release/
├─ middle_overlay_ros2_wss/    ur_ros2_ws/{src,build,install,log}   flir_ros2_ws/{...}
│                              slicer_ros2_ws/{...}   atracsys_ros2_ws/{...}
│                              isaac_ros2_ws/{...}   oem_ros2_ws/{...}
├─ description_catalog/        CAD/mesh staging, one dir per part group (see arts_description)
└─ code_intake/                PRE-EXISTING CODE STAGED FOR REVIEW, one dir per module
```

**`code_intake/` is where work from earlier projects lands before anyone decides its fate,
and the rule is that it is READ, never built or sourced.** Several later modules (teleop,
instrument drive, ESU, Slicer, sim) have working predecessors, and dropping that source
straight into `arts_ros2_ws/src` would defeat every property this repo is built for: it
arrives unpinned, it carries another project's naming, and it would be tracked by git before
the double-anonymous review of its contents has happened. "Code" is the common case but not
the limit — EDA projects, firmware and measured hardware documentation come in the same way
and are staged the same way. Same pattern as `description_catalog/`, same discipline:

```
code_intake/<module>/            e.g. esu_tap/, teleop/, dv_tool_driver/, slicer/
├─ 0_incoming/                   as received (a git clone counts), never edited
└─ NOTES.md                      REQUIRED before anything moves: where it came from, what it
                                 targeted (distro, middleware, hardware), license/authorship,
                                 and a verdict PER PART — PORT / REWRITE / REFERENCE-ONLY /
                                 ADOPT / DISCARD. One verdict for a whole repo is almost
                                 always wrong: `esu_tap` came out ADOPT for the KiCad v2
                                 project, REWRITE for the PlatformIO firmware, PORT for the
                                 ROS 2 nodes and REFERENCE-ONLY for the v1 board, in one repo.
```

Add a gitignored symlink under `reference/` so staged material is reachable from the repo
without being in it — `reference/esu_tap` alongside `reference/v1.4_ref`.

Nothing leaves `0_incoming/` by copy. Material is PORTED into a first-party `arts_*` package
the same way the v1.4 reference is: read it, take the parts that are right, write them fresh
against the locked decisions. The verdict in `NOTES.md` is what gets committed to the repo as
a docs entry; the source itself stays machine-local and never enters the review capsule.

**THE INTAKE JOB IS EVALUATION, NOT TRANSCRIPTION — three questions in this order, and skipping
the middle one is the failure mode.** (1) What is the logic, and is it *right*? (2) Is there a
better solution, or one that fits ARTS's structure better — chainable ros2_control instead of a
standalone node, `arts_interfaces` types instead of `std_msgs`, pinned ESP-IDF instead of a
second ESP32 build system? (3) Only then, migrate. A port that faithfully reproduces a design
nobody re-examined inherits its assumptions along with its code, and those assumptions were
made against a different distro, different hardware and a different deadline.
EXPECT HUMBLE, AND DO NOT TREAT IT AS A BUG. Collaborators' modules target Humble because the
Oct 5 funder demo does (see "Two separate deliverables"). Their code being on Humble is CORRECT for its
purpose; the Humble→Jazzy migration is ARTS's work, done at intake, not a change requested
upstream. Same for a design still under active revision — stage it, learn the pattern from it,
and adopt only once it settles.
`esu_tap` is the worked example: staged 2026-08-19 to answer "how do we do a PCB", which it
answered completely (KiCad 9, self-contained revisions, release-only fab exports), while the
board itself is still being revised upstream and is therefore NOT adopted yet. The next
expected intake is the da Vinci tool adapter.
THREE THINGS TO SETTLE BEFORE ANY STAGED FILE IS COMMITTED, and they are the reason this
staging area exists rather than a direct copy. Who owns the copyright, when the work was done
by a student or an RA — a permissive licence answers redistribution, not ownership. Whether
the notice, the upstream project name, or a repo owner's username would leak identity into the
review capsule; a git SUBMODULE is the worst available option here, since it records the
owner's username and points at something no reader can fetch. And whether the hardware it was
written against is the hardware ARTS actually has.

The two trees mirror each other by folder name: `~/arts/oem_drivers/spinnaker/install.sh`
MIRRORS ITS ARCHIVE to `~/arts_local/oem_drivers/spinnaker-4.2.0.88/`;
`~/arts/middle_overlay_ros2_wss/ur_ros2_ws/ur_ros2.repos` is fetched and built into
`~/arts_local/middle_overlay_ros2_wss/ur_ros2_ws/`. Root overridable via `$ARTS_LOCAL`
(default `~/arts_local`).

**"Mirrors its archive" is not the same as "installs to", and conflating the two caused real
confusion on 2026-08-18 — read this before moving an OEM install path.** `$ARTS_LOCAL` owns
DOWNLOADS AND BUILD ARTIFACTS. It does not own vendor system integration. An OEM package that
is a device driver installs where its vendor puts it, and Spinnaker is the worked example:
the account-walled tarball is mirrored under `$ARTS_LOCAL/oem_drivers/spinnaker-4.2.0.88/`
(that is the part the two-tree rule exists for — an un-redistributable blob recorded in one
auditable place, with its version pinned in `oem_drivers/spinnaker/VERSION`), while the debs
extract to `/opt/spinnaker` plus `/etc/ld.so.conf.d/libspinnaker.conf` plus udev rules.
Three reasons that is correct rather than lazy, and the first is decisive: VERIFIED
2026-08-18, the vendor binaries and `libSpinnaker.so.4` carry NO RUNPATH AT ALL, so they
resolve the GenICam libraries purely through that `ld.so.conf.d` entry pointing at
`/opt/spinnaker/lib` — relocating the tree means every consumer forever (the ROS driver, the
CLI samples, anything a user writes) must carry `LD_LIBRARY_PATH`. Second, the udev rules and
the usbfs setting are system-level whatever we do. Third, the Docker path must extract to `/`
regardless, and decision 4 wants the two paths byte-identical.
CONTRAST WITH ESP-IDF, where relocation out of the default WAS right, so the two decisions do
not conflict: that is a BUILD TOOLCHAIN, self-contained, honouring a documented
`IDF_TOOLS_PATH`, with no system integration, and its default `~/.espressif` was a hidden
directory recorded by nothing. `/opt/spinnaker` is documented, vendor-standard, auditable via
`ldconfig -p`, and is where a reader following FLIR's own instructions would already have it —
which helps replication rather than hurting it. The test to apply to the next OEM SDK is
therefore not "is it inside `$ARTS_LOCAL`" but "is it recorded, pinned, and removable":
Spinnaker's uninstaller ships in the mirrored archive as `remove_spinnaker.sh`.

`COLCON_IGNORE` is NOT `.gitignore`. Every directory above marked COLCON_IGNORE is fully
git-tracked; the marker only stops a `colcon build` run from `arts_ros2_ws` from descending
into workspaces that must be built separately.

**Build order** (from the legacy installer; each step in a fresh `env -i` shell with a
minimal PATH so no stale environment leaks in):

```
/opt/ros/jazzy   →  ur_ros2_ws  →  flir_ros2_ws  →  slicer_ros2_ws
                 →  atracsys_ros2_ws  →  isaac_ros2_ws  →  oem_ros2_ws
                    (each sources ONLY /opt/ros/jazzy)
                 →  arts_ros2_ws        (sources ALL middle overlays only — see rule below)
                 →  example_app_ros2_ws (sources arts_ros2_ws only)
```

**HOW THAT ORDER IS EXECUTED — ONE SHARED SEQUENCE SINCE 2026-09-08, fixing a defect that
undercut decision 4 (found by an external review 2026-08-19, verified 2026-08-20).** The module
contract says each `deploy/modules/<module>.sh` exposes `host_deps` / `oem_install` /
`overlay_build` and that BOTH `arts_install.sh` and the Dockerfiles consume them "so the two
paths cannot drift". For overlay builds that was true of FLIR ONLY: `arts_install.sh` called
`middle_overlay_ros2_wss/<ws>/build.sh`, never `<mod>_overlay_build` — the string appeared in
that script exactly once, in a COMMENT asserting the property — and five of the six `build.sh`
files re-implemented the same `env -i` / `vcs import` / `rosdep` / `colcon` sequence
independently (`ur` 54 lines, `oem` 28, `atracsys` 23, `isaac` 22, `slicer` 22) while `flir`
(13 lines) delegated.
It outranked its size because of WHAT it was: no-drift is the entire justification for the
module contract, and a comment claiming a property the code does not implement is worse than
the missing feature, since it stops the next person from checking. The FLIR pilot already
showed what drift costs — `flir_oem_install` carried a Dockerfile idiom (`rm -rf
/var/lib/apt/lists/*`) that was destructive on a host and surfaced two functions later as an
unrelated package "having no installation candidate" — and the `set -u`-after-source fix
learned in flir.sh never reached the other five copies, so the first bare-metal `ur_ros2_ws`
build (2026-08-10) died on it cold.
THE FIX, AS BUILT: `deploy/modules/_common.sh` holds `overlay_build_generic <ws> [--ws
PATH] [--repos FILE] [--exclude REL_PATH]… [--skip-key K]… [--cmake-arg A]… [--no-rosdep]`
— the ONE copy of the sequence, with the two field-tested fixes in it (`set -eo` → source
Jazzy → `set -u`; purge empty clone dirs before `vcs import --skip-existing`) and the
audible `$SUDO apt-get update`. `--exclude` applies all three MoveIt-exclusion markers at
once (COLCON_IGNORE + AMENT_IGNORE + rosdep `--skip-keys`), so decision 11's exclusion is
now an argument, not a script. `overlay_build_dispatch <ws>` is what `arts_install.sh`
calls: the owning module's `<mod>_overlay_build` when `overlay_owner_module` names one
(flir, so its Spinnaker preflight is on the installer's path), else the overlay's
`build.sh`, itself a one-call delegate. `flir_overlay_build` and `camera_sync_overlay_build`
delegate too; `build_flir.sh` stages `_common.sh` beside `flir.sh` so the image runs the
identical file. Per-overlay differences are arguments in the delegate (`ur`: the MoveIt
exclude; `slicer`: `--no-rosdep` + `-DSlicer_DIR`), never a second copy. Image-only
housekeeping (dropping `build/` and `log/`) is gated on `/.dockerenv`, per the lesson above.
`oem_ros2_ws` has deliberately NO owning module — it is shared by every loose OEM wrapper.
VERIFIED 2026-09-08 against an empty scratch `$ARTS_LOCAL` on this laptop: `./arts_install.sh
--overlay flir` → dispatch → `flir_overlay_build` (preflight fires) → generic, 4 packages in
54 s, `/opt/ros/jazzy` once and last in the resulting chain, `build/` retained on bare metal;
`--overlay ur` writes both IGNORE markers and passes `--skip-keys`, then stops at rosdep
wanting sudo for `liburdfdom-tools` — the same interactive requirement the old script had;
re-run with `--no-rosdep` on the same tree, the colcon stage built all 6 packages clean in
5 min 33 s with `ur_moveit_config` excluded, so the whole `--exclude` path is exercised.
The Docker image was NOT rebuilt (daemon not running); `Dockerfile.flir` gained one COPY
line and is otherwise unchanged. Still to do: a CI check that the functions named in
`deploy/docker/Dockerfile.*` are the ones `overlay_build_dispatch` resolves to — that is the
mechanical form of the no-drift property (Phase 0 CI item). The production-workstation
install remains the only real test of the recipe end to end. Full triage:
`docs/repo_structure_triage.md`, item 3.

**ROS 2 overlay sourcing rules (critical — do NOT violate):**

ROS 2 overlay `install/setup.bash` files **automatically chain to their underlay.** When a
workspace is built after sourcing an underlay, the resulting `setup.bash` records that chain
internally. Therefore you NEVER manually source multiple layers — you source ONLY the
immediate underlay, and it chains down for you. Sourcing multiple layers manually can cause
duplicate or conflicting entries in `AMENT_PREFIX_PATH` and break package resolution.

```
TO BUILD a middle overlay (e.g. flir_ros2_ws):
    source /opt/ros/jazzy/setup.bash        ← the ONLY source
    cd $ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws
    colcon build

TO BUILD arts_ros2_ws (full — needs all middle overlays):
    source $ARTS_LOCAL/middle_overlay_ros2_wss/ur_ros2_ws/install/setup.bash
    source $ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws/install/setup.bash
    source $ARTS_LOCAL/middle_overlay_ros2_wss/slicer_ros2_ws/install/setup.bash
    ...                                     ← each middle overlay (they chain to Jazzy
                                              automatically; do NOT also source Jazzy)
    cd ~/arts/arts_ros2_ws
    colcon build

TO BUILD arts_ros2_ws (modular test — e.g. only FLIR):
    source $ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws/install/setup.bash
    cd ~/arts/arts_ros2_ws                  ← only FLIR packages will resolve; that's fine
    colcon build                              for isolated module testing

TO BUILD example_app_ros2_ws (or any application overlay):
    source ~/arts/arts_ros2_ws/install/setup.bash   ← the ONLY source
    cd ~/arts/example_app_ros2_ws
    colcon build

TO RUN arts_ros2_ws packages directly:
    source ~/arts/arts_ros2_ws/install/setup.bash   ← the ONLY source, period
    ros2 launch arts_driver arts_bringup.launch.py cameras:=true

TO RUN packages from an application overlay (e.g. example_app_ros2_ws):
    source ~/arts/example_app_ros2_ws/install/setup.bash   ← NOT arts_ros2_ws
    ros2 launch example_app ...                             (chains to arts_ros2_ws
                                                             → middle overlays → Jazzy)
```

The rule is always the same: **source only the immediate underlay, never the full chain.**
Each `setup.bash` chains downward automatically. Sourcing both
`/opt/ros/jazzy/setup.bash` AND a middle overlay is a common mistake — it works by
accident in simple cases but breaks when paths conflict. `build_arts_ws()` in
`arts_install.sh` violated exactly this until 2026-08-10; a correct chain shows
`/opt/ros/jazzy` in `AMENT_PREFIX_PATH` exactly once, at the end. That is a cheap thing to
check after building any overlay.

Corollary worth internalizing: because `/opt/ros/jazzy` is always on `CMAKE_PREFIX_PATH` as
the underlay, ANY apt package that installs a `<lib>Config.cmake` there will win
`find_package(<lib>)` over an ARTS-built copy unless ARTS's prefix is explicitly prepended.
This is silent — no warning, wrong version, and it compiles. It is the mechanism behind the
Ruckig incident described below, and it is why an apt dependency pulled in by some package
you do not even use can change the library your control loop links against.

**`arts_driver` bringup contract** — one package, per-component toggles, so the decision-10
benchmark ladder is the same launch file at every rung:

```
ros2 launch arts_driver arts_bringup.launch.py \
    psm_left:=true  psm_left_tool:=large_needle_driver \
    psm_right:=true psm_right_tool:=monopolar_curved_scissors \
    ecm:=true ecm_stereo:=true ecm_stereo_sync:=true \
    mtm_left:=true mtm_right:=true footswitch_teleop:=true head_sensor:=true gamepad:=true \
    use_fake_hardware:=false rviz:=true
```

Module solo = one flag true; contention pair = two flags; full cart = all true.

Carts are deployment profiles (`cart-patient`, `cart-surgeon`, `cart-vision`), not code
boundaries. Procedure-specific autonomy (perception, resection planners, task FSMs) belongs
in application overlays, NOT in `arts_ros2_ws` — the prior system mixing `ismr2024` and
`iros_2024_nephrectomy` into the main workspace is the mistake this avoids.
