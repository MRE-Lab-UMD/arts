# Phase 0 — Scaffolding — status narrative

> **Role: status narrative archive.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Checkboxes here are a SNAPSHOT and are NOT authoritative — status lives only in
> `AGENTS.md`. Narrative that duplicates a module doc is still to be de-duplicated
> into that module doc (see `docs/repo_structure_triage.md`). Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

- [x] Create the `arts` git repo (private; the remote URL is deliberately NOT recorded in any
      tracked file — it names the owner's account, and this tree becomes the anonymous review
      capsule. `git remote -v` is where that belongs); reference symlink + gitignore set up
- [x] Host prep, Ubuntu 22.04 (superseded — kept as the record behind the cyclictest
      numbers below): docker 29.1.3; PREEMPT_RT 5.15.0-1081-realtime as a second GRUB entry
      (generic 6.8.0-136 default); NVIDIA 595.84 proprietary built on RT via the
      `IGNORE_PREEMPT_RT_PRESENCE=1` DKMS bypass, made permanent in
      `/etc/dkms/framework.conf` + `apt-mark hold`; nvidia-container-toolkit 1.13.5.
- [x] Host prep, Ubuntu 24.04 (2026-08-10, via `./arts_install.sh --prereqs`): ROS 2 Jazzy;
      NVIDIA 595.84 via `nvidia-driver-595-open` (open kernel modules — Ubuntu's recommended
      package on noble); Docker CE 29.7.2; nvidia-container-toolkit 1.19.1-1;
      `isolcpus/nohz_full/rcu_nocbs = 4,5,10,11`; realtime-kernel 6.8.1-1057-realtime
      (note the generic kernel here is the NEWER 7.0.0-28).
      VERIFIED ON THE FIRST RT BOOT: `uname -v` = `#58-Ubuntu SMP PREEMPT_RT`;
      `/sys/devices/system/cpu/isolated` = `4-5,10-11`; `nvidia-smi` works with Xorg
      actually running on the discrete GPU; `docker run --gpus all` reaches it.
      Secure Boot is disabled on this machine, so the DKMS modules' unenrolled MOK key
      is moot — with Secure Boot ON, enroll it or the modules will not load.
      Three 24.04-specific gotchas, all documented in docs/host_prerequisites.md: enabling
      realtime-kernel DISABLES Canonical Livepatch (mutually exclusive); it writes
      `GRUB_FLAVOUR_ORDER="realtime"` so the RT kernel becomes the DEFAULT boot entry rather
      than an extra one; and the `-open` driver's PREBUILT modules exist for the generic
      flavour ONLY (no `linux-modules-nvidia-*-realtime` package exists), so the prebuilt
      route leaves the RT kernel with no NVIDIA driver at all and never pulls in DKMS.
      CORRECTION to an earlier assumption: `-open` does NOT avoid the PREEMPT_RT build
      refusal — verified on this machine, `nvidia-dkms-595-open` fails the same
      `preempt_rt_sanity_check` as the proprietary module, so `IGNORE_PREEMPT_RT_PRESENCE=1`
      is required for BOTH flavours (it goes in `/etc/dkms/nvidia.conf`, NOT
      `framework.conf`, which noble's dkms ignores — see the workstation entry below).
      Also, the nvidia-dkms
      postinst builds only for the RUNNING kernel, so the RT kernel needs an explicit
      `dkms install nvidia/<ver> -k <rt-kernel>`. All three steps must happen BEFORE the
      first RT boot; `deploy/prereqs.sh` now does them.
- [~] Host prep, production workstation (started 2026-10-21; Core Ultra 9 285K, 8P+16E, no
 SMT; RTX 5080, not the 5090 assumed earlier).
 HEADLINES (2026-10-26): **full install `--path both` from a clean `$ARTS_LOCAL` and an
 empty Docker build cache, 7 min 46 s to `ALL DONE`** (images `arts-base:jazzy`
 `c4d89e583cc5`, `arts-flir:jazzy` `9caca2ccfab3`); **RT latency 19 us worst case on
 isolated P-cores under CPU + NVIDIA load**
 (`docs/benchmarks/rt_latency_ultra9-285k-8p16e_20261026.md`). The 7 min 46 s is NOT a
 bare-OS figure and must not be quoted as one: host apt packages from the three earlier
 runs persisted, the `ros:jazzy-ros-base-noble` pull was cached, Spinnaker was
 pre-mirrored, and `--prereqs` (14 min 31 s) failed and needed the manual recovery below.
 It took FOUR full runs, each failing on a recipe defect now fixed. Still `[~]` for the
 `prereqs.sh` rework below; a reinstall-from-bare-OS run is what would turn the install
 time into a paper number.
 RECIPE DEFECT FOUND BEFORE THE TIMED
 INSTALL: `recommend_isolcpus` took the highest-numbered `lscpu` CORE and picked
 `22,23`, two E-cores sharing one L2 with the unisolated 20 and 21 — the sibling hazard
 the rule existed to prevent, through a cache grouping `lscpu -p=CPU,CORE` does not
 show. Fixed to select whole L2 groups, P-cores only, never CPU 0, highest-boost cores
 left to the scheduler; the laptop result (`4,5,10,11`) is unchanged. The NVIDIA driver
 was installed by hand before the recipe ran (615-open); it is being purged so the timed
 run installs the `ubuntu-drivers` recommendation itself.
 TIMED `--prereqs` RUN 2026-10-21 23:25, 14 min 31 s, FAILED WITHOUT SAYING SO — it
 ended "Done." with no NVIDIA module for ANY kernel and the RT kernel half-installed.
 Exact chain, from `/var/log/apt/term.log` and the DKMS `make.log`:
 (1) `nvidia-driver-595-open` pulled `nvidia-dkms-595-open` (not prebuilt modules, as
 615-open and the laptop had), whose postinst built for the running 7.0.0-38-generic and
 skipped it: "kernel headers for this kernel do not seem to be installed". That kernel
 image sits outside its meta package (`linux-*-generic-hwe-24.04` are still 6.17.0-14),
 so nothing ever installed its headers. Machine state before the recipe, but the recipe
 never checks headers for the RUNNING kernel.
 (2) `pro enable realtime-kernel` installed `linux-headers-6.8.1-1062-realtime`, whose
 `header_postinst.d/dkms` hook built nvidia AUTOMATICALLY — before the script's bypass
 step — and hit "Failed PREEMPT_RT sanity check". The hook failure left
 `linux-headers-` and `linux-image-6.8.1-1062-realtime` half-configured (`iF`) and
 `ubuntu-realtime` unpacked-only (`iU`), so `99-realtime.cfg` stayed as `.dpkg-new` and
 the RT kernel is NOT the default entry. `pro` retried apt 4× at 23:37 and the script
 printed nothing on its failure. `/etc/dkms/framework.conf` has no bypass.
 ROOT CAUSE IN THE RECIPE: the bypass is applied REACTIVELY, after a manual `-k` build
 fails, but once DKMS is installed the RT kernel's own header hook builds first. The
 laptop never hit this because its driver was prebuilt, so no DKMS existed when
 `pro enable` ran. The bypass must be written BEFORE `pro enable` whenever DKMS is
 present. Also: `nvidia_pkg` is read before step 1 installs the driver, so it is empty
 by step 2.
 RECOVERY, AND A SECOND TRAP IT HIT: bypass written to `framework.conf`, then
 `apt install linux-headers-7.0.0-38-generic`. Its DKMS hook built the generic module
 fine, but `nvidia-cdi-refresh.service` (nvidia-container-toolkit 1.20.1, installed by
 step 4) then ran `nvidia-smi` and LOADED `nvidia` into the live session while `nouveau`
 drove the display; gnome-shell tried to hot-plug the GPU and the desktop froze (no
 kernel reboot — boot 0 continued). Same failure as the "never modprobe into a live
 session" rule, reached through a toolkit service, so any driver (re)build after the
 toolkit is installed must be followed by a reboot, not by continuing in the session.
 THIRD FINDING, AND IT FALSIFIES A DOCUMENTED CLAIM: `sudo dpkg --configure -a` with
 the bypass in `framework.conf` failed the identical PREEMPT_RT sanity check. dkms
 3.0.11 (noble) reads `framework.conf` through `safe_source`, which sources it in a
 subshell and re-exports ONLY an allowlist (`source_tree dkms_tree install_tree
 tmp_location verbose symlink_modules autoinstall_all_kernels modprobe_on_install` +
 signing keys), so `export IGNORE_PREEMPT_RT_PRESENCE=1` there is SILENTLY DROPPED.
 `host_prerequisites.md`'s "framework.conf survives apt upgrades" is therefore false on
 24.04; the laptop worked only because `prereqs.sh`'s retry passes the variable on the
 `sudo` command line. Side effect of the failed configure: the RT kernel has
 `vmlinuz` but NO initrd (the dkms postinst hook aborts `run-parts` before
 initramfs-tools), so it must not be booted until configure succeeds.
 RECOVERED 2026-10-22 00:01 with `sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg --configure -a`
 (module built for both kernels, initrd generated, `99-realtime.cfg` installed), then
 `apt-mark hold nvidia-driver-595-open nvidia-dkms-595-open`. VERIFIED ON FIRST RT BOOT:
 `uname -v` = `#63-Ubuntu SMP PREEMPT_RT` (6.8.1-1062-realtime);
 `/sys/devices/system/cpu/isolated` = `2,4,6-7`; `nvidia-smi` = 595.99.02, RTX 5080,
 Xorg on the discrete GPU. DURABLE FIX, `prereqs.sh` rework 2026-10-26: NOT the driver's
 own `dkms.conf` (package-owned, re-extracted on every configure) but the per-module
 override `/etc/dkms/nvidia.conf`, which dkms 3 sources after `dkms.conf` with `MAKE` on
 its allowlist, so `MAKE[0]="export IGNORE_PREEMPT_RT_PRESENCE=1; ${MAKE[0]}"` prefixes
 the driver's own command for any version. The same rework writes it BEFORE
 `pro enable`, installs running-kernel headers before the driver, re-reads the driver
 package after installing, stops for a reboot after a driver install, and exits non-zero
 listing failed steps, half-configured packages and initrd-less RT kernels.
 VERIFIED 2026-10-26: `dkms unbuild` then `sudo env -u IGNORE_PREEMPT_RT_PRESENCE dkms
 install nvidia/595.99.02 -k 6.8.1-1062-realtime` compiled afresh and passed, and dkms
 printed its build command as `export IGNORE_PREEMPT_RT_PRESENCE=1; unset ARCH; …`, so the
 override is what let it through; the 2026-10-22 configure without it is the failing
 control. TRAP while testing: `dkms install --force` on an already-built module only
 re-copies the old `.ko` files (seconds, no compile), so it proves nothing — `unbuild`
 first. After a successful build the log is at `<kver>/x86_64/log/make.log`, not
 `build/make.log`.
 FULL-INSTALL SCOPE DECIDED 2026-10-22, BEFORE ITS FIRST RUN, and it is a scope decision,
 not a workaround: the full install had hardwired all six overlays, so it would have
 died at `slicer_ros2_ws` — whose Slicer superbuild `arts_install.sh` never builds, and
 whose pins (Slicer v5.8.1, `slicer_ros2_module` v1.2) were never verified on 24.04 or
 Jazzy — and it never built coal or pinocchio, although their `build.sh` headers claim
 "Called by: arts_install.sh" and `arts_controllers` asserts both (plus apt Ruckig).
 Default is now what the ISMR paper uses (ur, flir, oem overlays → coal → pinocchio →
 `arts_ros2_ws` → example app); slicer, atracsys, isaac and proxsuite are opt-in via
 `ARTS_ENABLE_<NAME>=1` until verified on Jazzy (Slicer: ask its developers which
 release builds on 24.04 and which `slicer_ros2_module` revision supports Jazzy).
 The paper's install-time figure must state this module set. Known gap, not fixed: the
 control libraries' apt dependencies (Eigen, Boost, assimp, qhull) are declared nowhere;
 they are present here only because `ros-jazzy-desktop` pulls them in.
 FIRST FULL RUN 2026-10-22 00:31, 6 min 22 s, FAILED AT proxsuite, which the 429f3a8
 scoping wrongly included (no `arts_ros2_ws` package uses it; only the cut Phase 2 QP
 does): "Could NOT find Simde (missing: Simde_INCLUDE_DIR) ... Set
 BUILD_WITH_VECTORIZATION_SUPPORT=OFF or install Simde". Everything before it built
 clean — ur 6 pkgs 52.5 s, flir 4 pkgs 18.7 s, oem 2 pkgs 19.6 s, coal v3.0.4, pinocchio
 v3.9.0 with collision support. Proxsuite dropped from the default. FOR PHASE 2, two
 things the opt-in path must settle: declare `libsimde-dev` (noble: 0.7.2-6) or decide
 the QP runs unvectorized, and pin jrl-cmakemodules, which proxsuite FETCHES FROM THE
 NETWORK at configure time when its submodule is absent (it got v2.3.0, commit
 `7017548`) — the transitive-FetchContent hazard of pinning rule 2.
 SECOND FULL RUN 2026-10-22 00:42, CLEAN `$ARTS_LOCAL` (overlays, open_source_software,
 install_path and `arts_ros2_ws/{build,install,log}` deleted; Spinnaker mirror kept),
 4 min 4 s, FAILED IN `arts_ros2_ws`: `arts_controllers` "Could not find a package
 configuration file provided by "ruckig"" at CMakeLists.txt:36; `arts_datasets` and
 `arts_instrument_id` aborted with it, 12 packages finished, 1 not processed; example
 app and Docker images NOT REACHED. Cause: `package.xml` declares `<depend>ruckig</depend>`
 (→ rosdep → `ros-jazzy-ruckig`), but `build_arts_ws` never runs `rosdep install` — only
 the middle overlays do, via `overlay_build_generic` — and `ros-jazzy-ruckig` is not
 installed. The laptop had it by hand (see the Ruckig decision record), which hid this.
 FIXED 2026-10-26: `build_arts_ws` and `build_example_app` now run `rosdep install`.
 Adding it exposed a SECOND latent defect: `arts_driver`, `arts_datasets` and
 `arts_instrument_id` declared `<buildtool_depend>ament_python</buildtool_depend>`, which
 is not a rosdep key (a ROS 2 Python package declares only `<build_type>`); colcon
 ignores it, rosdep aborts on it. Removed; `rosdep install --simulate` now asks for
 exactly `ros-jazzy-ruckig`.
 THIRD FULL RUN 2026-10-26 09:42, clean, 5 min 31 s: **THE BARE-METAL PATH COMPLETED** —
 ur 6 pkgs, flir 4, oem 2, coal, pinocchio, `arts_ros2_ws` 16 pkgs in 28.2 s — then
 FAILED AT THE FIRST DOCKER STEP: `COPY .base-staging/deploy/cyclonedds.xml ... not
 found`. `build_docker_images` ran `docker build` on `Dockerfile.base` directly,
 bypassing `deploy/docker/build_base.sh`, the script that stages `.base-staging/`
 (`build_flir.sh` was already called correctly). Fixed to call `build_base.sh`.
 SAME RUN, A SILENT DEFECT: `example_app_ros2_ws` "built" 0 packages and printed
 Finished, because its root `COLCON_IGNORE` (there so a repo-root colcon skips it) is
 honoured when colcon runs INSIDE it too. Fixed with `--base-paths src` plus a guard that
 fails if `colcon list` is empty; verified `example_app` builds and `/opt/ros/jazzy`
 appears once and last.
 NEXT: wipe `$ARTS_LOCAL` as above, re-run the timed full install with
 `--path both`, then `deploy/bench/rt_latency.sh` with `ARTS_MACHINE_CLASS` set to a CPU
 description. Also pending: the `prereqs.sh` rework for the defects above.
- [x] RT + GPU compatibility test: PASSED on BOTH stacks, so decision 8 holds across the
      migration. Ubuntu 22.04 / 5.15.0-1081-realtime / NVIDIA 595.84 proprietary (2026-08-05):
      worst-case 190 us, avg 4–5 us. Ubuntu 24.04 / 6.8.1-1057-realtime / `nvidia-driver-595-open`
      (2026-08-10): worst-case **73 us**, avg 5 us = 3.6% of a 500 Hz period, with the discrete
      GPU verified at 33% mean / 41% peak utilization during the run. Averages identical, tail
      shorter — the kernel, not the driver flavour. Quote the 24.04 number in the paper.
      Harness: `deploy/bench/rt_latency.sh` (derives the core map from the kernel, verifies
      the GPU was actually loaded, writes a committed report). Baseline:
      `docs/benchmarks/rt_latency_devlaptop-6c12t_20260810.{md,json}`.
      Caveat for the paper: glmark2 is a moderate GPU load (~33%), not the ARTS workload.
      The definitive contention numbers come from the decision-10 integration ladder.
- [x] Base dev image (`deploy/docker/Dockerfile.base`: Jazzy + CycloneDDS + MCAP +
      ros2_control; `deploy/docker/compose.yaml` with per-module compose profiles;
      `deploy/cyclonedds.xml` tuned for GigE camera traffic). Verified: RMW =
      rmw_cyclonedds_cpp, GPU passthrough confirmed, MCAP plugin loads. CycloneDDS is a
      deliberate departure from the prior system's implicit Fast DDS — chosen for
      shared-memory simplicity and large-data throughput.
      GAP FOUND AND CLOSED 2026-08-12. The above had been true INSIDE THE CONTAINER ONLY:
      bare metal had only `rmw_fastrtps_cpp` installed with `RMW_IMPLEMENTATION` unset, so
      everything run natively silently used Fast DDS. Decision 4 makes both paths
      first-class and the decision-10 ladder compares across them, so two middlewares is
      not a cosmetic inconsistency — it invalidates the comparison.
      The fix is `deploy/modules/dds.sh`, which is the single source of truth for the
      middleware layer and is consumed by BOTH `arts_install.sh` and `Dockerfile.base`, the
      same non-drift contract decision 4 requires of every module. `cyclonedds.xml` moved
      `deploy/docker/` → `deploy/` because it is no longer container-only; its settings are
      unchanged so the committed FLIR baselines still describe the running config.
      `dds_host_deps` installs the RMW package, writes `net.core.rmem_max/rmem_default`
      (25 MiB) to `/etc/sysctl.d/10-cyclonedds.conf` AND applies them live, installs the
      config to `/etc/cyclonedds/cyclonedds.xml`, and exports `RMW_IMPLEMENTATION` +
      `CYCLONEDDS_URI` from `/etc/profile.d/arts-dds.sh`. Deliberately NOT apt-pinned with
      `=version`, for the rolling-repo reason in the Ruckig decision record.
      Verified on this machine 2026-08-12: native `rclpy` reports
      `rmw_cyclonedds_cpp`; runtime `rmem_max` = 26214400 against the config's 10 MB
      request; and — the check that matters — pointing `CYCLONEDDS_URI` at a deliberately
      malformed file makes participant creation FAIL quoting that path, which proves the
      ARTS config is genuinely parsed rather than silently ignored. The failure path was
      exercised, not just the pass path, same discipline as the Ruckig assertion.
      Image rebuilt and verified the same way (`arts-base:jazzy`, digest
      `sha256:46a2a223ee94…`): in-container RMW is `rmw_cyclonedds_cpp` and
      `/etc/cyclonedds/cyclonedds.xml` is byte-identical to `deploy/cyclonedds.xml`, so both
      paths demonstrably run the same middleware off the same file.
      THREE TRAPS worth remembering. `/etc/profile.d` is read by LOGIN shells only, so any
      non-login caller (service unit, CI step, `env -i` build shell, an editor terminal)
      gets the RMW default and silently tests the wrong stack;
      `arts_controllers/test/run_servo_smoke_test.sh` now sources the file itself when
      `RMW_IMPLEMENTATION` is unset, and any new harness must do the same. A container
      cannot set `net.core.*` — it shares the host kernel — so `dds_host_deps` runs on the
      host in the DOCKER path too, not only the bare-metal one. And Docker's default `RUN`
      shell is `/bin/sh` = dash, which has no `set -o pipefail`: since decision 4 makes the
      Dockerfiles RUN the same `deploy/modules/*.sh` functions the installer does, and every
      one of those opens with `set -euo pipefail`, sourcing one under the default shell dies
      on line 1 with "Illegal option -o pipefail" before defining any function. That killed
      the first rebuild. `Dockerfile.base` now sets `SHELL ["/bin/bash", "-c"]` once; every
      future module Dockerfile needs the same line.
      STILL OPEN: the FLIR camera baseline has NOT been re-run bare-metal under CycloneDDS.
      It is camera-gated, not blocked on anything here — the committed baselines were taken
      in Docker, which already used CycloneDDS, so they remain valid for the Docker path.
      Re-run `docs/benchmarks/flir_*` natively when the cameras are next connected.
- [x] Benchmark harness template: `arts_record/bench/` — bench launch file
      (`flir_bench.launch.py`), MCAP storage config (`mcap_image_config.yaml`), offline analyzer
      (`analyze_bench.py`: rate, drops, CPU, memory, disk throughput from `rosbag2_py` +
      `docker stats`), orchestration script (`run_flir_bench.sh`).  First baseline committed:
      `docs/benchmarks/flir_bfs-u3-63s4c_20260806.md` (image digest `sha256:7365f6bd9642…`).
- [x] Create both trees: repo skeleton (see "Repo layout") with COLCON_IGNORE markers in
      every non-package directory, plus `~/arts_local/` with matching folder names.
      Package stubs (`package.xml` / CMakeLists or setup.py) placed for all arts_* packages.
- [x] Write `middle_overlay_ros2_wss/<ws>/<ws>.repos` manifests with real commit hashes or
      tags. All 5 active overlays pinned (UR Driver 3.8.0 — the 3.x series is the Jazzy
      branch; the earlier 2.13.2 was a Humble-only tag — FLIR camera driver 3.0.5,
      slicer_ros2_module v1.2, cisst-saw/crtk individual repos pinned to commit hashes,
      newton_ros pinned to commit hash). `oem_ros2.repos` entries remain commented until each
      module is activated. All 6 `open_source_software/*/VERSION` files filled (coal v3.0.4,
      pinocchio v3.9.0, proxsuite v0.7.3, ruckig — superseded 2026-08-12, now apt 0.9.2 —
      Slicer v5.8.1, PlusLib Plus-2.8.0).
      All 6 `oem_drivers/*/VERSION` files filled from verified versions (Spinnaker
      4.2.0.88, Haply 3.4.16, fusionTrack 4.7.1, Zivid 2.14.0, Clarius Cast 11.0.0,
      librealsense 2.51.1).
- [x] `arts_install.sh` skeleton: fetch + build the chain in order (base → each middle overlay
      in its own `env -i` shell → arts_ros2_ws), orchestrating `deploy/modules/*.sh`.
      Full vision (in progress): on a fresh Ubuntu 24.04 machine the user runs ONLY
      `./arts_install.sh` and it handles everything — detects/installs ROS 2 Jazzy,
      optionally installs Docker + nvidia-container-toolkit (if user wants Docker path),
      sets up PREEMPT_RT (guides user to register Ubuntu Pro, installs the RT kernel),
      runs host_deps/oem_install/overlay_build for each module, builds arts_ros2_ws.
      The script is idempotent: re-running skips already-completed steps. OEM SDKs that
      require vendor login (Spinnaker, etc.) are detected-or-prompted, not auto-downloaded.
- [x] `arts_interfaces` initial msg/srv/action set: 5 messages (InstrumentState,
      EnergyCommand, EnergyState, EpisodeMark, TeleopState) + 1 service (SetEnergyMode).
      Designed for the reference-run critical path; the prior system's OROCOS command/feedback messages
      are replaced by ros2_control standard interfaces — `arts_interfaces` covers only what
      ros2_control does not (instrument identity, energy activation, recording marks, teleop
      state). Nephrectomy-specific interfaces (the prior system's `nephrectomy_interfaces`) belong in
      application overlays, not here.
- [x] Jazzy migration: repo side DONE (Dockerfiles, module scripts, overlay build.sh,
      README, bench harness on Jazzy; `arts_install.sh` now asks bare / docker / both and
      remembers the answer in `$ARTS_LOCAL/install_path`). Two pins were re-verified
      against upstream: UR Driver moved 2.13.2 → 3.8.0 (2.x is the Humble branch, 3.x the
      Jazzy branch); FLIR 3.0.5 is distro-shared and unchanged. Pins for slicer, atracsys
      and isaac were NOT re-verified — do that when each module is activated.
      DONE 2026-08-10: host setup on 24.04 (see Phase 0 above); both images rebuilt from
      scratch on Jazzy (`arts-base:jazzy`, `arts-flir:jazzy` digest `sha256:350845b4b901…`);
      FLIR benchmark re-run and the new baseline committed (identical 39.99 Hz / 0 drops).
      DONE 2026-08-10 (bare metal): `ur_ros2_ws` is the first middle overlay ever compiled
      against `/opt/ros/jazzy` outside Docker, and it builds clean — 5 packages, ~4–7 min
      depending on machine load, only stderr being an upstream `tl_expected` deprecation
      that Jazzy's own parameter_traits emits (removal is scheduled for a distro well past
      Jazzy's 2029 EOL; ignore it). The UR 3.8.0 pin is therefore verified, not just
      claimed: tag exists, is the newest on the 3.x series, and its commit matches the hash
      recorded in `ur_ros2.repos`.
      Three defects were found and fixed doing it, all invisible until a real bare-metal run:
      (1) EVERY overlay `build.sh` and both build functions in `arts_install.sh` did
      `set -euo pipefail` before `source /opt/ros/jazzy/setup.bash`. That setup file reads
      `$AMENT_TRACE_SETUP_FILES` without a `:-` default, and `env -i` guarantees it is unset,
      so `-u` killed every overlay build instantly. The fix (`set -eo` → source → `set -u`)
      already existed in `deploy/modules/flir.sh` from the FLIR pilot but had never been
      back-ported, because FLIR is the only module built so far and it is the one that goes
      through flir.sh rather than a generic build.sh. Check the other build.sh files first
      whenever a new overlay is activated.
      (2) `build_arts_ws()` sourced `/opt/ros/jazzy/setup.bash` AND every middle overlay —
      the exact mistake the "source only the immediate underlay" rule below names. It now
      sources Jazzy only as a fallback when no middle overlay exists.
      (3) `ur_moveit_config` pulled the MoveIt stack and a conflicting Ruckig; excluded per
      decision 11. Note `ur_description` comes from apt (`ros-jazzy-ur-description` 3.5.1),
      NOT from the .repos manifest, so it is currently UNPINNED — it defines the link
      geometry and joint limits `arts_description` will build on, so pin it with `=` when
      `deploy/modules/ur.sh` is written (that script does not exist yet; flir.sh is the
      template).
      Known regression: SpinView_QT/libSpinVideo do not run on Noble
      (jammy ffmpeg ABI) — CLI sample tools replace them; see docs/modules/pilot_flir_camera.md.
- [~] `arts_description`: URDF for pedestal + 3 arms + adapters + tools + trainer box.
 DONE 2026-08-12 (Stage 1 placeholder): `urdf/arts.urdf.xacro` instantiates a primitive
 pedestal box plus three prefixed arms from the pinned `ur_description` (`tool_left_` /
 `tool_right_` UR5e, `camera_` UR5 CB3). Mount poses are named properties in
 `config/mount_transforms.yaml` and are PLACEHOLDERS — not measured from the Vention
 STEP. View with `ros2 launch arts_description view_arts.launch.py`. ros2_control
 hardware stays in the per-arm driver xacro until Phase 2. Collision capsules, real
 mount measurements, adapters, tools, laparoscope, and the trainer box are still open.
 CORRECTED 2026-08-17 by decisions 12 and 13, which supersede the shape of this stage —
 do not extend `arts.urdf.xacro` toward being the control description. Four things
 changed and each is a rework item, not a nice-to-have. (1) `arts.urdf.xacro` is the
 VIEWING/sim assembly only; the control path needs `tool_arm.urdf.xacro` and
 `camera_arm.urdf.xacro` rooted at their own mount frames, in their own namespaces,
 because a `controller_manager` takes one description and the arms are rate-grouped.
 (2) `mount_transforms.yaml` is CAD-NOMINAL FOREVER; measured corrections go in
 `arts_calibration/config/`, so that the CAD-versus-measured discrepancy survives as a
 reportable number. (3) The pedestal origin is the STEP file's own origin tied to a
 physical feature — NOT the mount-derived intersection point, which is non-observable
 and self-referential (full reasoning in decision 12). (4) The tool mounts tilt 45°
 nose-down, so per-arm initial positions must be re-derived per mount.
 Assets are migrated from a second PC ONE PART AT A TIME through the machine-local
 staging area `$ARTS_LOCAL/description_catalog/`, which is the ONLY source — do not go
 hunting for meshes elsewhere on the machine; other 3-D model folders are unreviewed
 leftovers from earlier projects and are not authoritative for anything.
 Six part groups in physical-chain order: `1_pedestal` (Vention, INCLUDES the arm mounts,
 so it alone defines all three arm base transforms), `2_arms` (apt-supplied, nothing to
 stage), `3_tool_adapter`, `4_tools` (one subfolder per instrument), `5_laparoscope`
 (mount + head + FLIR bodies + optics as one part), `6_sample_setup` (trainer box,
 cannula, table, CT-scanned organ phantoms). Each part flows `0_incoming/` (as-received,
 duplicates fine) → `1_cad/` (chosen master) → `2_mesh/{visual,collision}`, with a
 `NOTES.md` recording units, origin, provenance and license. A part enters this repo
 only once its NOTES.md is resolved.
 Three things that shape the work:
 - Arm VISUAL geometry is closed: `ur_description` 3.5.1 supplies meshes and config for
 `ur5` (CB3) and `ur5e`, and is now pinned from SOURCE in `ur_ros2.repos` (tag 3.5.1 =
 commit 710505166d0399…) rather than arriving unpinned from apt. `arts_description`
 wraps its `ur_robot` macro with a prefix and vendors nothing. Arm COLLISION geometry
 is ARTS's own work and is NOT yet authored — see the two entries below.
 - An apt `=version` pin does NOT solve this and was rejected: packages.ros.org is a
 rolling repo, not an archive, so the exact version string stops being installable once
 a newer build lands and the pin turns into a fresh-machine build failure. Prefer a git
 tag for anything whose numbers we publish. Verified 2026-08-11: 6 packages build clean,
 the overlay `ur_description` wins over apt, `/opt/ros/jazzy` still appears once in
 `AMENT_PREFIX_PATH`. Note the build needs `liburdfdom-tools` via rosdep (sudo).
 - Ask for STEP, not STL. An STL carries no units, so a millimetre export silently
 becomes a 1000x error; STEP states units in its header. Use AP214: AP242 superseded
 it, but SolidWorks (checked on 2026) exports only AP203 and AP214 and has no setting
 that enables AP242. AP214 and AP242 keep colour and assembly structure; AP203 drops
 them. SolidWorks masters must arrive as Pack-and-Go zips, since a bare `.SLDASM`
 arrives with broken references.
 - VENDOR GEOMETRY IS USABLE BUT NOT PUBLISHABLE — checked for both vendors 2026-08-11,
 and both landed in the same place, so assume it for any vendor part until proven
 otherwise. `ur_description` is DUAL-licensed: BSD-3-Clause code, but the meshes fall
 under UR's "Terms and Conditions for Use of Graphical Documentation", which permit our
 exact use (simulation, path planning, collision avoidance, non-commercial research) and
 even permit redistribution if the T&Cs travel along — but §2.2 assigns UR the rights to
 any DERIVATIVE work, so a decimated or convex-hulled UR mesh would not be ours to
 license, and §5.2/§6 let UR change or revoke the terms. Vention is stricter: Terms of
 Sale §8.2 grants a "non-transferable, non-sublicensable, revocable" license limited to
 installation, operation and maintenance, so its part geometry cannot be redistributed
 at all. The arrangement we designed is ours (Terms of Use, User Content); the component
 geometry is not.
 The escape is the same for both and costs little: AUTHOR OUR OWN PRIMITIVE COLLISION
 GEOMETRY from published or measured dimensions — boxes for extrusion, capsules for arm
 links. Legally clean, what the 500 Hz QP wants anyway, and the numbers that matter
 (mount transforms, link lengths, RCM positions) are measurements, not vendor IP. For
 visual fidelity, depend on vendor distribution instead of copying: UR meshes arrive via
 the pinned `.repos`, and the Vention design can be linked by public design ID so a
 reader pulls it themselves. Same expectation for da Vinci instruments — kinematic
 parameters ours, Intuitive's geometry almost certainly not.
 - KNOWN DEFECT TO REPRODUCE AND FIX IN PHASE 2: on the prior Humble system, coal
 reported UR links colliding with THEMSELVES when fed the official collision meshes.
 Partly adjacent-link pairs that need SRDF `disable_collisions`, partly convex-hull
 inflation that no SRDF entry fixes. This is a third, independent reason to author
 capsules rather than reuse vendor meshes. Reproduce it on Jazzy first so the baseline
 is documented, then verify zero self-collisions over a joint-space sweep.
 - Stage 1 can proceed against a parameterized placeholder pedestal with the mount
 transforms as named xacro properties, so modelling is not blocked on the fetch.
 If mesh volume ever exceeds what GitHub is comfortable with (100 MB/file hard limit,
 ~1 GB/repo advisory), decimate harder first, then consider Git LFS, a release tarball,
 or Hugging Face — decide when it is a real problem, not before.
- [x] `arts_driver` bringup skeleton with per-component toggle arguments
      (`arts_bringup.launch.py` — logs enabled flags; includes TBD)
- [~] CI: full chain builds clean on Jazzy from one script. STARTED 2026-08-20 with
      `deploy/ci/check_anonymity.sh`, which is the piece that could not wait: an external
      structural review found the maintainer's name and email in 17 `package.xml` files, a
      username-derived hostname baked into a COMMITTED benchmark filename, and — the reason a
      script exists rather than a rule in a doc — none of those were written by hand. ROS
      tooling generates the maintainer field, and `hostname -s` regenerated the filename on
      every benchmark run, so `deploy/bench/rt_latency.sh` now keys reports to a machine
      CLASS (`ARTS_MACHINE_CLASS`, default derived from CPU topology) and the 2026-08-10
      baseline was renamed accordingly. The check's FAILURE path was exercised, not just its
      pass path, same discipline as the Ruckig assertion and the CycloneDDS malformed-config
      test. Still to come: the colcon build of the full chain and `ament_lint`. Note there is
      a ready-made first job waiting — `arts_controllers/test/run_servo_smoke_test.sh` encodes
      six regressions that actually happened and nothing currently runs it.
- [x] Repo-level basics: `LICENSE` (verbatim Apache-2.0, holder line unfilled by design — see
      CONTRIBUTING.md) and `CONTRIBUTING.md`. Both were missing while every `package.xml`
      declared Apache-2.0, which for a project whose stated purpose is "any research
      institution can reproduce this" is not optional.
- [x] Structural review triaged (2026-08-20): `docs/repo_structure_triage.md` records an
      accept/scope/defer/reject verdict per item with the evidence for each, and the received
      review is staged machine-local at `$ARTS_LOCAL/code_intake/repo_review/` rather than
      tracked — it contained 15 instances of the identity its own first item was about. The
      verdicts that change something structural are recorded at the relevant place in this
      file rather than only there. TWO REMAIN OPEN AND ARE THE REAL WORK: one overlay-build
      mechanism (see the note under "Build order" below), and extracting per-module narrative
      out of this file into the module docs the documentation contract already defines.
