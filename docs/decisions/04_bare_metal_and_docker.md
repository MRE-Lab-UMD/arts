# Decision 4 — Bare metal and Docker, both first-class

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

4. **Bare-metal AND Docker — both are first-class, neither is optional.** The system
   must work as a native Ubuntu 24.04 + ROS 2 Jazzy installation AND as Docker
   containers. Both paths are tested, both are documented, both are release
   deliverables. This is a hard requirement: a user who cannot or will not run Docker
   must be able to `./arts_install.sh` on a fresh Ubuntu 24.04 machine and get a
   working system; a user who prefers containers must be able to `docker compose up`
   and get the same result. The two paths share one source of truth: every module
   ships `deploy/modules/<module>.sh` (host_deps / oem_install / overlay_build
   functions); `arts_install.sh` orchestrates these scripts on bare metal, and the
   Dockerfiles RUN the same functions in layers — so neither path can drift.
   Thin `host_setup.sh` handles what containers cannot: PTP daemons, NIC/MTU tuning,
   udev rules, CPU isolation, kernel/GRUB entries. Each middle overlay = one cached
   Docker layer. OEM installers (Spinnaker, Haply service, Zivid) are build arguments,
   not baked into public images.

## Amendment 2026-11-01 — the Docker path now covers the whole chain (mock-verified only)

Until this date only `Dockerfile.base` and `Dockerfile.flir` existed; `compose.yaml` named
`Dockerfile.ur`, `Dockerfile.haply` and `Dockerfile.arts`, none of which had been written, and
no image built coal, pinocchio, proxsuite or `arts_ros2_ws`. Now:

- **`Dockerfile.arts` → `arts:jazzy`**, built `FROM arts-flir:jazzy` by
  `deploy/docker/build_arts.sh`, so the chain matches bare metal: Jazzy → `ur_ros2_ws`,
  `flir_ros2_ws`, `oem_ros2_ws` → coal → pinocchio → proxsuite → `arts_ros2_ws`. Every step
  RUNs the installer's own function: `arts_build_deps`, `arts_control_libs_build` and
  `arts_ws_build` (new `deploy/modules/arts.sh`; `arts_install.sh`'s `build_control_libs` and
  `build_arts_ws` now call the same functions instead of carrying inline copies) and
  `overlay_build_dispatch` for the two overlays. `ur_ros2_ws` got its owner (`ur`) in
  `overlay_owner_module`. The control libraries' apt build dependencies (Eigen, Boost
  components, assimp, qhull, OpenCV) are pinned in `arts_build_deps`, closing the Phase 0
  "declared nowhere" gap for both paths; on the workstation the pinned set is already
  installed (`apt-get install -s`: 0 newly installed).
- **One image, many services**, not one image per module: every module node is a package of
  the single `arts_ros2_ws` build, and per-module images would be differently-built copies of
  it (`Dockerfile.arts` header). `compose.yaml` services: `psm_left`, `psm_right`, `ecm` (RT:
  `CAP_SYS_NICE`, rtprio 99, memlock unlimited; no `cpuset:` because `ur_arm.launch.py` pins
  each loop to its `cpu_core` and Docker's default cpuset includes the isolated cores),
  `camera_sync`, `disc_drive`, `haply` (talks to the host's Inverse Service over its
  WebSocket; nothing Haply in the image), `footswitch`, `head_sensor`, and an `arts` shell.
  Devices are reached through the host `/dev` plus `device_cgroup_rules` by major, so udev
  symlinks and hotplug work without `--privileged`.
- `arts_install.sh --path docker|both` builds `arts:jazzy` last, after `arts-flir:jazzy`.

Verified 2026-11-01 in the container (image `sha256:52f1e5c7fd98…`, 8.59 GB on disk, from
`e3e7ef0`): all 18 `arts_*` packages listed; `/opt/ros/jazzy` once in `AMENT_PREFIX_PATH`, last;
`task_qp_test` 8/8 passed; `run_servo_smoke_test.sh` PASS inside the `psm_left` service with
"Successful set up FIFO RT scheduling policy with priority 50". Failure paths seen: the same
smoke test run without the compose RT settings logs "Could not enable FIFO RT scheduling
policy ... Operation not permitted" (and still passes, so the smoke test does not guard RT);
with apt `ros-jazzy-pinocchio` installed and the source prefixes off `CMAKE_PREFIX_PATH`,
`arts_controllers` configure stops at "Found pinocchio 4.1.0 ... but ARTS pins 3.9.0".

Not yet covered: no container has driven a real arm, camera, sync box, pedal, Haply or
Dynamixel bus; no benchmark has been run in `arts:jazzy`, so no paper number comes from it;
the image is not slimmed (it keeps the `arts_ros2_ws` build tree that `--symlink-install`
needs); `host_setup.sh` is still unwritten; `Dockerfile.flir` and its image were not rebuilt.

### Amendment 2026-11-08: build parallelism bounded by memory

Install failure, recorded as it happened: rebuilding `arts:jazzy` on the production
workstation (24 cores, 20 usable after `isolcpus`, 31 GB) compiled pinocchio with one job per
core (`JOBS=$(nproc)`); the templated compile units need several GB each, the machine ran out of
memory, and the kernel OOM killer took VS Code and the desktop session down mid-build (no image
was produced). The same recipe runs on bare metal, so a fresh install on a large-core machine
would fail the same way. Fix, in the recipes both paths share: `open_source_software/pinocchio`
and `coal` `build.sh` use min(cores, available GB / 4 resp. / 2) jobs, and `arts_ws_build` builds
one package at a time with min(cores, available GB / 3) jobs; `ARTS_BUILD_JOBS` overrides. On the
workstation that is 6 / 13 / 9 jobs; the incremental bare-metal workspace build is unchanged
(7.8 s).
