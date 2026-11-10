# Module pilot: FLIR camera

> **Doc layer 2 of 4 — module overview.** What this module is, how it was built, and what
> remains. Status shown here MIRRORS AGENTS.md and is never independent of it.
>
> | | Where | For |
> |---|---|---|
> | Decisions + **authoritative status** | AGENTS.md decisions 5 / 14 / 15, "Module pilot — FLIR camera" checklist | why, and what is done |
> | **You are here** | this file | the module: artifacts, pins, stages, results, what's left |
> | Bench procedure | [`camera_sync_wiring.md`](camera_sync_wiring.md) | wiring and rungs 5a–5e, step by step |
> | Measured numbers | [`../benchmarks/`](../benchmarks/) — `flir_*` and `camera_sync_*` | figures plus the rig and pins they came from |

**Camera:** 2× FLIR Blackfly S BFS-U3-63S4C (USB3, color, Sony IMX178, 3072×2048 @ 57 fps)
**Serials:** 24057131 (left stereo), 24057132 (right stereo)
**Interface:** USB3 (no PTP — stereo sync via hardware trigger only)

First module developed under the module-first methodology (AGENTS.md decision 10). Besides
delivering the camera module itself, this pilot validates the whole development pattern —
if the pattern feels wrong here, fix the pattern before applying it to the next module.

## Versions in play, and which one is which

Four different things in this module get called a "version" or a "pin", and they are easy to
confuse because three of them are numbers near each other. This table is the answer to "which
version is actually used here":

| What | Value in use | Pinned in | Notes |
|---|---|---|---|
| Spinnaker SDK | **4.2.0.88** | `oem_drivers/spinnaker/VERSION` | Installs to `/opt/spinnaker`. Archive mirrored to `$ARTS_LOCAL/oem_drivers/spinnaker-4.2.0.88/`. Account-walled download. |
| ROS driver repo (`flir_camera_driver`) | **3.0.5** | `middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos` | Distro-shared branch, so the Jazzy migration did not move it. Ships both the single and the synchronized driver. |
| Camera firmware | **1801.0.9.0** | not pinnable — it is on the device | Read with `/opt/spinnaker/bin/Enumeration`. Several measured behaviours are properties of THIS firmware (see Stage 5). |
| Which of the two ROS drivers | **`spinnaker_camera_driver`** (single) | `arts_driver/launch/cameras.launch.py` | NOT `spinnaker_synchronized_camera_driver`. Decided 2026-08-19; reasoning in AGENTS.md decision 5 amendment. |

**The one open version question, and the answer for a fresh machine.** FLIR ships **4.4.0.99**
for Ubuntu 24.04; ARTS pins 4.2.0.88, which officially supports 20.04 and 22.04 only and is
why `SpinView_QT` and `libSpinVideo` do not run on noble (7 debs deliberately skipped). On a
new host, **install the pin as recorded first** and reproduce the committed FLIR baseline —
that is the only clean test of the written recipe — and treat the bump to 4.4.0.99 as a
separate, benchmarked act afterwards. Bumping during the first install changes two variables
at once and makes any regression unattributable.

**When the pin cannot be downloaded (2026-11-09).** An outside tester found the flir.com
download link in every installer message dead and only a 4.4.0 release on offer. The link now
points at Teledyne's Spinnaker page (`DOWNLOAD_URL` in `oem_drivers/spinnaker/VERSION`), the
debs are found by name anywhere under `spinnaker-<ver>/` (24.04 tarballs nest differently), and
`ARTS_SPINNAKER_VERSION=<ver>` installs another release as an explicit, loudly printed
deviation from the pin, not a bump. `ARTS_ENABLE_FLIR=0` drops the module entirely for installs
without cameras (`docs/host_prerequisites.md` §4). No ARTS run on 4.4.0 exists yet.

**A pinned SDK can be silently bypassed, so verify rather than assume.**
`spinnaker_camera_driver`'s CMake falls back to `DownloadSpinnaker.cmake`, which maps noble →
4.4.0.99 and fetches it from a third-party host. A machine that skips `flir_oem_install`
therefore builds successfully against an SDK nobody chose. `flir_overlay_build` now preflights
for `/opt/spinnaker/lib/libSpinnaker.so`, so "the driver built fine" is not evidence the OEM
install worked — `ldconfig -p | grep Spinnaker` is.

## Camera GPIO lines — which physical pin does what

Separate from the software versions above, and the other thing "which pin" can mean. The
6-pin Hirose pigtail, as ARTS uses it (full pinout, colours and resistor values in
[`camera_sync_wiring.md`](camera_sync_wiring.md)):

| Camera line | Hirose pin | Direction | ARTS use | Why not the alternative |
|---|---|---|---|---|
| **Line 0** | 2 (+ 5 = OPTO_GND) | opto-isolated input | **trigger in**, from the ESP32 through 560 Ω | Upstream's own examples use `Line3` (pin 1), whose absolute maximum input-high is 3.6 V — a 5 V drive there destroys the camera. Line 0's isolation is also the only barrier against the cart's monopolar ESU. |
| **Line 1** | 4 | opto-isolated output | **`ExposureActive` strobe out**, with a 1 kΩ pull-up | The pull-up is 1 kΩ, not 10 kΩ: the pin idles LOW and RISES at exposure start, so the pull-up governs the edge that dates the photons. At 10 kΩ, 4–5 % of strobes were missed outright. |
| Line 2 | 3 | switchable 3.3 V rail | diagnostic only, default off | Energizing it is a static way to confirm an unlabelled pigtail's colour-to-pin map. |
| Line 3 | 1 | non-isolated I/O | **unused** | See Line 0 above. |
| — | 6 | camera power ground | **never connected to ARTS ground** | Tying it across shorts the optocoupler and silently discards the isolation. |

`560 Ω`, not the obvious 1 kΩ, on the trigger input: FLIR's table gives 3.1 mA at 3.3 V
through 1 kΩ, below the 3.5 mA minimum input current, so 1 kΩ there is an intermittent
trigger that looks like a software bug.

> **Stages 0–4 below were completed on Ubuntu 22.04 / ROS 2 Humble and are kept as the
> historical record.** They are not re-marked for Jazzy. What the migration changed, and
> what has to be re-run before the numbers count again, is in
> [Jazzy migration](#jazzy-migration-what-changed) at the end of this document.

**Why FLIR first:** it exercises the four riskiest mechanisms at once — Docker↔hardware
access (the classic container failure mode), the vendor-SDK vendoring pattern (Spinnaker
requires a FLIR account to download, like the prior system's `Software/` folder), the benchmark harness,
and the highest-bandwidth data path in the system.

## The dual-path install contract (applies to every module)

Bare-metal install must remain a first-class option ("Docker works in sim but sometimes fails
to connect to hardware" — keep the door open). To guarantee the two paths never drift:

```
deploy/modules/flir.sh          ← single source of truth, three functions:
   flir_host_deps()               sysctls, udev, NIC tuning   (bare-metal / host only)
   flir_oem_install()             Spinnaker SDK debs; recipe in oem_drivers/spinnaker/,
                                  archive mirrored at $ARTS_LOCAL/oem_drivers/spinnaker-<ver>/
   flir_overlay_build()           Spinnaker preflight, then the SHARED sequence:
deploy/modules/_common.sh       ← overlay_build_generic <ws>: env -i, source Jazzy, vcs import,
                                  rosdep, colcon — the ONE copy every overlay uses (since
                                  2026-09-08; before that five build.sh files had their own)
                                  overlay_build_dispatch <ws>: module function if the overlay
                                  has an owner, else its build.sh (itself a delegate)

arts_install.sh                 ← thin orchestrator: asks bare / docker / both, then runs
                                  deploy/modules/*.sh and overlay_build_dispatch per overlay.
                                  host_deps() runs on the host in EVERY path.
Dockerfile.flir                 ← RUNs flir_oem_install + flir_overlay_build in layers
                                  (build_flir.sh stages _common.sh beside flir.sh);
                                  host_deps stays on the host (kernel/sysctls/udev are not
                                  containerizable anyway)
```

One script, two consumers. `arts_install.sh` is therefore not written by hand as a monolith
like the legacy installer was — it grows automatically as modules are added.

## Artifacts this pilot must produce (all checked into the arts repo)

| Artifact | Path |
|---|---|
| Module script (single source of truth) | `deploy/modules/flir.sh` |
| OEM SDK recipe (install.sh + pinned VERSION) | `oem_drivers/spinnaker/` |
| Container image + compose profile `flir` | `deploy/docker/Dockerfile.flir`, `deploy/docker/compose.yaml` |
| Pinned camera middle-overlay manifest | `middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos` |
| Camera bringup: launch + per-serial params + calib placeholder | `arts_ros2_ws/src/arts_driver/` |
| Benchmark harness (first instantiation = the template) | `arts_ros2_ws/src/arts_utilities/arts_record/bench/` |
| Committed baseline report | `docs/benchmarks/flir_<model>_<date>.md` |
| Orchestrator v0 (ROS section + flir module wired) | `arts_install.sh` |

## Stage 0 — Host readiness (~0.5 day)

> Assumes the Phase 0 host prep in AGENTS.md (docker + nvidia-container-toolkit install) is
> already complete — this stage only **verifies** it and adds the camera-specific host
> configuration. Do not re-install anything that Phase 0 already set up.

- [x] NVIDIA check: `nvidia-smi` works; driver version 595.84. Verified 2026-08-05 (see
      README.md host setup log).
- [x] Docker + nvidia-container-toolkit; verified via both
      `docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi` and
      `docker run --rm --gpus all arts-base:humble nvidia-smi`. Base dev image
      (`arts-base:humble`) built with CycloneDDS + MCAP + ros2_control.
- [x] Camera identified: 2× FLIR Blackfly S BFS-U3-63S4C, USB3, serials 24057131 (L) /
      24057132 (R). **Interface is USB3** (not GigE): no PTP, no NIC/MTU/sysctl tuning
      needed. Stereo sync will use hardware trigger only (Stage 5). Host-side USB3 plumbing
      already in place:
  - udev rules: `/etc/udev/rules.d/40-flir-spinnaker.rules`,
    `/etc/udev/rules.d/99-spinnaker-usb.rules` (installed by Spinnaker SDK)
  - `usbcore.usbfs_memory_mb=1000` (confirmed; default 16 MB is too small for USB3 Vision)
  - Container will mount `/dev/bus/usb` (not `network_mode: host` — that's GigE-only)
- [x] Model/serial/interface recorded at the top of this file. Verified 2026-08-05:
      Spinnaker 4.2.0.88 enumerates both cameras; SpinView_QT streams successfully.

## Stage 1 — OEM driver layer in a container (~0.5–1 day)

> Spinnaker SDK 4.2.0.88 is already installed on the dev PC (dpkg-confirmed, same version
> as the prior system). VERSION file pinned. flir_camera_driver 3.0.5 is pinned in flir_ros2.repos.

- [x] Spinnaker SDK debs mirrored to `$ARTS_LOCAL/oem_drivers/spinnaker-4.2.0.88/`.
      `oem_drivers/spinnaker/install.sh` implemented with clear download instructions if
      SDK is missing. `oem_drivers/spinnaker/VERSION` pinned to 4.2.0.88.
- [x] `Dockerfile.flir` built: `FROM arts-base:humble` + Spinnaker debs extracted via
      `dpkg-deb -x` (bypasses EULA debconf prompt that blocks in non-interactive Docker).
      Build script `deploy/docker/build_flir.sh` stages debs into build context temporarily
      (Docker cannot access paths outside its context), builds, cleans up via trap.
- [x] **SpinView_QT streams live from inside the container** (verified 2026-08-06).
      Docker run flags: `--privileged -v /dev/bus/usb:/dev/bus/usb` + X11 forwarding.
      Single camera (24057132) enumerates and streams at full resolution.
      Second camera not visible while first is streaming — USB3 bandwidth saturation on
      shared controller (expected; Stage 5 concern, not Stage 1).
      SpinView_QT path: `/opt/spinnaker/bin/SpinView_QT` (PATH added in Dockerfile).
      PC slowdown observed during full-res streaming — expected at 6.3 MP.
- [x] **Acceptance: PASSED.** Docker↔USB3 hardware access works with `--privileged` +
      `/dev/bus/usb` mount. No udev or kernel-level changes needed beyond what Spinnaker's
      bare-metal install already provides.

## Stage 2 — ROS 2 wrapper (~1 day)

- [x] `middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos`: pinned to
      `ros-drivers/flir_camera_driver` tag 3.0.5 (latest release, 2026-05-08; it sits on
      the shared `ros2-release` branch, so the same tag serves Humble and Jazzy).
      Ships `spinnaker_camera_driver` and `spinnaker_synchronized_camera_driver`. ARTS uses
      the FORMER; the synchronized driver was evaluated for Stage 5 and rejected (see the
      versions table above and AGENTS.md decision 5 amendment).
- [x] `flir_ros2_ws` built as a Docker layer in `Dockerfile.flir` (sources only
      `/opt/ros/humble/setup.bash` per overlay rule; rosdep resolves deps including xacro).
      Entrypoint overridden to source `/opt/flir_ros2_ws/install/setup.bash`.
- [x] Camera bringup: `arts_driver/launch/cameras.launch.py` launches left/right camera
      nodes via `spinnaker_camera_driver` (serials 24057131/24057132). Settings: BayerRG8,
      40 fps (~70% of 57 fps ceiling), `adjust_timestamp: True`, chunk data ON (FrameID,
      ExposureTime, Gain, Timestamp), trigger_mode Off (free-running; Stage 5 switches to
      hardware trigger). Wired into `arts_bringup.launch.py` via `cameras:=true`.
- [x] ROS driver verified (2026-08-06): `spinnaker_camera_driver` streams at **40.00 Hz,
      0% drops** inside Docker container. Required host fix: `net.core.rmem_max=26214400`
      (CycloneDDS needs 10 MB socket buffer; kernel default 212992 is too small — causes
      `rmw_create_node` failure). Docker flags: `--privileged --net=host
      -v /dev/bus/usb:/dev/bus/usb`.
- [x] `header.stamp` verified (2026-08-06): device clock timestamps confirmed
      (sec: 1785997191, distinct from host wallclock). `adjust_timestamp: True` maps the
      camera's internal clock to monotonic ROS time.

## Stage 3 — Benchmark harness (~1 day; becomes the template for every module)

- [x] Bench launch: `arts_record/bench/flir_bench.launch.py` starts one camera node +
      rosbag2 MCAP recorder. `mcap_image_config.yaml` configures Zstd compression.
      `run_flir_bench.sh` orchestrates: Docker container start → duration-controlled
      recording via `timeout --signal=INT` (ensures clean bag shutdown with `metadata.yaml`)
      → docker-stats sampling every 5 s → offline analysis. (On Humble this needed
      `ros-humble-rosbag2-storage-mcap` plus an explicit `-s mcap`; on Jazzy MCAP is the
      default storage plugin and the flag has been removed.)
- [x] Offline analyzer: `arts_record/bench/analyze_bench.py` reads the MCAP bag via
      `rosbag2_py`, deserializes Image messages, computes per-topic rate mean ± std from
      `header.stamp` deltas, drop count (gaps > 1.5× nominal period), bag size, disk
      throughput. Parses `docker_stats.csv` for CPU/memory. Outputs JSON + markdown table.
- [x] 10-minute baseline committed: `docs/benchmarks/flir_bfs-u3-63s4c_20260806.md` with
      image digest `sha256:7365f6bd9642...`.

      | Metric | Result |
      |--------|--------|
      | Rate | 39.99 ± 0.00 Hz (100.0% of 40 Hz nominal) |
      | Period jitter | 25.004 ± 0.002 ms |
      | Drops | 0 over 599 s |
      | Bag size | 26.4 GB (Zstd, 44.0 MB/s) |
      | CPU | 125.9% mean, 152.9% peak |
      | Memory | 755 MiB peak (2.4%) |

- [x] **Acceptance: PASS.** 100% of nominal rate, 0 drops, resource usage well within laptop
      budget.  Note: full 30-minute acceptance run deferred to Stage 4 (run against the
      finalized dual-path image to serve as the from-scratch reproduction check).

## Stage 4 — Packaging: the dual-path contract (~0.5–1 day)

- [x] `deploy/modules/flir.sh` rewritten with three full implementations:
  - `flir_host_deps()`: usbfs_memory_mb, CycloneDDS rmem_max/default sysctl, udev check.
  - `flir_oem_install()`: searches `/tmp/spinnaker/` (Docker) then `$ARTS_LOCAL` (bare metal);
    `dpkg-deb -x` for non-interactive install in both contexts; apt deps inline.
  - `flir_overlay_build(ws_path)`: parameterized workspace root (bare metal defaults to
    `$ARTS_LOCAL/…`, Docker passes `/opt/flir_ros2_ws`). `env -i` shell sources only
    `/opt/ros/humble/setup.bash`. Key fix: `set +u` before sourcing setup.bash
    (`AMENT_TRACE_SETUP_FILES` unbound in clean env).
- [x] `oem_drivers/spinnaker/install.sh` and `middle_overlay_ros2_wss/flir_ros2_ws/build.sh`
      now delegate to the module script (thin wrappers, not duplicate logic).
- [x] `arts_install.sh` updated: full build discovers module scripts in `deploy/modules/`,
      calls `<module>_host_deps()` + `<module>_oem_install()` before building overlays.
- [x] `Dockerfile.flir` refactored: `SHELL ["/bin/bash", "-c"]`, stages module script +
      VERSION + repos + debs via `build_flir.sh` into `.flir-staging/`, then
      `RUN . flir.sh && flir_oem_install` and `RUN . flir.sh && flir_overlay_build /opt/…`.
      Same functions as bare-metal path — cannot drift.
- [x] Compose profile `flir` already in `deploy/docker/compose.yaml` (USB plumbing, privileged).
- [x] **Acceptance: PASS.** From-scratch image build (new digest `sha256:b7618fec…`) reproduces
      the Stage-3 baseline: 39.99 Hz, 0 drops over 60 s. Bare-metal path (host Spinnaker +
      `build.sh`) was validated during Stages 1–2 and is now wired through the same functions.

## Stage 5 — Synchronization: the ESP32 sync box

**Not gated on the laparoscope rig.** The trigger can be built and the skew measured on the
bare camera pair sitting on the bench; only the final stereo *optical* calibration needs the
scope. Scope and method were revised 2026-08-17 after reading the pinned driver source — see
AGENTS.md decisions 5 (amendment) and 14.

**Where things stand, 2026-09-08.** The bring-up ladder 5a–5e is complete on the breadboard
rig and the sync box works: skew measured two independent ways, agreeing, thirty times better
than the < 1 ms target. The PCB layout is approved and the JLCPCB order is queued (not yet
purchased or fabricated). AGENTS.md's Stage 5 checklist is authoritative for status; the
items below are what remains, in order.

1. ~~Compose the two cameras into ONE process.~~ **DONE 2026-08-20** in
   `cameras.launch.py`: both cameras are composable nodes in an `arts_camera_container`,
   keeping the single `spinnaker_camera_driver` (template: upstream
   `multiple_cameras.launch.py`). This replaced the former plan to switch to
   `spinnaker_synchronized_camera_driver`, which is cancelled — see AGENTS.md decision 5
   amendment, and "Why a first-party sync box" below. Two things worth carrying forward:
   composing renames NO topic (`/camera_left/meta` is unchanged, because `topicPrefix_` is
   empty in this driver — the prefixed layout is the synchronized driver's), and one absent
   or slow-to-enumerate camera now kills the container and therefore the other camera too,
   so check `lsusb` before launching. Details and the reasoning against `respawn` are in the
   launch file's own docstring and in AGENTS.md.
2. Fabricate the sync-box PCB. Decision 14 forbids a committed skew baseline from the
   breadboard, because contact resistance and lead length move the systematic term by the
   same order as the 2.81 µs measured.
   **DESIGN PACKAGE + LAYOUT APPROVED 2026-09-08.** Source is
   `arts_ros2_ws/src/arts_embedded/arts_camera_sync/hardware/arts-camera-sync-v2/`
   (KiCad 9 project `arts-camera-sync-v2.kicad_pro` plus `design_package/`). Pin-checked against
   `01` §2–§4; DRC 0/0; antenna keep-out clear; JLCPCB order queued with PCBA for R1–R6
   and D1–D6 only. Boards have not arrived. `production/` stays empty until they exist.
   The committed skew baseline is still this item's remaining work — it cannot come from
   the breadboard.
3. Take the committed baseline on the PCB, on the production workstation, with both cameras
   composed — which is also where the "both cameras, full resolution, full rate, zero drops"
   number gets settled.
   **TAKEN 2026-10-28 (Stage D, E3):** skew L − R **+2.793 ± 3.209 µs** (24057131 − 24057132;
   the report itself labels 24057132 LEFT, the role assignment corrected later that day — its
   numbers are per serial and stand), worst 10.825 µs over
   all 8991 pulses, 0 missed strobes; `camera_time` cross-check agreeing pulse by pulse
   (r = 0.990, residual 0.447 µs std). Report:
   `../benchmarks/camera_sync_skew_v2pcb_fix_20261028.md`. Two readings of it.
   (i) The systematic did not move with the board: per camera serial it is
   24057132 − 24057131 = −2.79 µs on the breadboard (where 24057131 was cam0) and −2.71 /
   −2.79 µs in the two PCB runs (where it is cam1), so it follows the cameras or their
   pigtails, not the capture channel or the resistors. The Stage C console run gave −1.61 µs
   for the same pair, so the term is not stable to better than ~1 µs between sessions.
   Camera-internal versus cable is not separated.
   (ii) **Missed strobes were a firmware defect, not missed exposures** — this overturns the
   explanation given for the breadboard's misses. The first PCB run
   (`../benchmarks/camera_sync_skew_v2pcb_20261028.md`) lost 16 of 8991 start edges while
   `camera_time` showed no frame gap. `CONFIG_ARTS_SYNC_EDGE_DIAG` then showed every loss to be
   a rising edge, at the right time and with the pin high, that the ESP-IDF driver's
   `cap_edge` (from the MCPWM `cap_status` register) reported as falling. That happens on ~0.1 %
   of edges, and why the register is wrong is not established. The firmware now classifies
   edges by pin level (`7859c33`), and the re-run lost none. The same diagnostic caught one
   extra rising edge on both strobe lines at the same tick, 81 µs into an exposure; the first
   start in a record now wins, and later starts are counted as glitches (SELFTEST only). **The A/B against the two-process layout was RUN 2026-08-21 and came
   out inconclusive in the only honest sense: the dropouts did not reproduce in EITHER
   layout** (5 min per arm, full-res 30 Hz triggered, split buses, every window 30.00 Hz).
   So the composed container is verified on real hardware at full rate — including the skew
   path, which matched the breadboard baseline through the container (mean 3.40 µs,
   systematic 2.63 µs; `../benchmarks/flir_composed_ab_20260821.md`) — but the shared-SDK
   dropout hypothesis remains untested, because the defect is currently non-reproducible.
   `composed:=false` stays for the session in which the dropouts reappear; re-run the A/B
   before anything else in that session.

### Why a first-party sync box and not just a pulse generator

Three facts, each verified rather than assumed, force this design:

1. **USB3 Blackfly S have no PTP.** `ptp4l`/`phc2sys` are not applicable at all. The shipped
   `blackfly_s.yaml` does expose `gev_ieee_1588*`, but those nodes are under
   `TransportLayerControl/GigEVision` and do not exist on a USB3 model.
2. **`spinnaker_synchronized_camera_driver` does not put the device clock in
   `header.stamp`.** `TimeKeeper::getTimeStamp()` takes the camera timestamp as an unnamed,
   unused argument; stamps come from host arrival time, chunk `FrameID`, and a
   Kalman-smoothed common-frame-time estimator. The README says outright that "the Timestamp
   is not used at the moment". The package deliberately assigns *identical* stamps to the
   frames of one pulse — so **L/R skew computed from `header.stamp` is identically zero and
   measures nothing**. (The single-camera `spinnaker_camera_driver` is the opposite: it
   honors `adjust_timestamp` and does carry the device clock, as Stage 2 confirmed. Same
   repo, opposite behaviour — this is the trap.)
3. **The device clock is still reachable**, on `~/<cam>/meta`
   (`flir_camera_msgs/ImageMetaData`, field `camera_time`). Negligible bandwidth, and the
   only software route to a real per-camera timestamp. Both drivers publish it through the
   same code, so it is available whichever one is used.

**Fact 2 is also why ARTS does not adopt the synchronized driver at all** (decided
2026-08-19). Its single benefit is the identical stamps described above, which ARTS has no use
for since stereo correspondence comes from the trigger and skew from the sync box. Worse, when
a synchronizer is attached the driver never reaches the `adjust_timestamp` path, so the real
per-camera device-clock stamp is replaced by the estimator's output — a measurement traded for
an estimate. Neither driver publishes chunk `FrameID`, so that is not a differentiator either
(and it forced an amendment to AGENTS.md decision 15, whose image↔pulse join assumed FrameID
was available).

So the ESP32 does two jobs, and the second one is the reason to build rather than buy: it
**generates** one hardware trigger edge for all cameras, and it **captures each camera's
`ExposureActive` strobe back** on an MCPWM capture channel clocked at 80 MHz (12.5 ns
resolution). Skew then becomes the difference of two capture timestamps on one counter —
a hardware measurement independent of ROS, Spinnaker, DDS, and any clock estimator. None of
the SOTA anchor platforms (dVRK-Si, MOPS, the open laparoscopic platform) reports a number
obtained this way.

### Hardware

- **MCU:** Espressif ESP32-DevKitC-32E (ESP32-WROOM-32E, dual-core LX6). Chosen because it
  has 2× MCPWM units × 3 capture channels and an 80 MHz capture clock.
- **Camera connector:** Hirose HR10A-7R-6PB on the camera; mating HR10A-7P-6S(73). We use
  FLIR's pre-made GPIO pigtail (colored wires, bare ends).
- **Passives:** 3× 560 Ω (trigger current limit, one per camera), 3× 1 kΩ (strobe pull-up —
  **not** the 10 kΩ originally specified; see the pull-up table in `camera_sync_wiring.md`,
  measured 2026-08-18, where 10 kΩ lost 4–5 % of strobes outright).

### Camera GPIO pinout (BFS cased models)

| Pin | Wire | Line | Function | ARTS use |
|---|---|---|---|---|
| 5 | blue | — | OPTO_GND (isolated, *not* camera ground) | ESP32 GND — **connect first, always** |
| 2 | black | Line 0 | OPTOIN, opto-isolated input. High 2.6–30 V, **input current 3.5–7 mA** | trigger in, via 560 Ω |
| 4 | white | Line 1 | OPTOOUT, opto-isolated **open-collector** output | `ExposureActive` out, 1 kΩ pull-up, **active-HIGH at the pin** (measured 2026-08-18 — idles low, rises while exposing, i.e. the opposite of what open-collector implies) |
| 1 | green | Line 3 | VAUX 8–24 V / non-isolated input, **max input-high 3.6 V** | **leave open** |
| 3 | red | Line 2 | 3.3 V out (120 mA) / non-isolated I/O | leave open |
| 6 | brown | — | camera power ground | leave open (powered over USB3) |

Verify the wire colors on your actual pigtail with a multimeter before trusting this table.

Three electrical points that are easy to get wrong:

- **The opto input is an LED, so you drive CURRENT, not voltage.** That is what the series
  resistor is for. **1 kΩ is the wrong obvious choice**: FLIR's own table gives 3.1 mA at
  3.3 V through 1 kΩ, *below* the 3.5 mA minimum, which yields an intermittent trigger that
  looks exactly like a software bug. 560 Ω gives ≈5.5 mA; 680 Ω also works; 330 Ω exceeds
  the 7 mA ceiling.
- **Line 1 is open-collector**, so it can only pull down. Without the external pull-up you
  see nothing. With it, the signal is *active-low*.
- **Never put 5 V on pin 1.** Its non-isolated input tops out at 3.6 V. The upstream
  driver's `follower_example.launch.py` uses `trigger_source: Line3` (= pin 1); ARTS uses
  `Line0` instead, both to remove that footgun and because a monopolar ESU shares this cart
  and opto-isolation is the only barrier between its ground currents and the camera front
  end. The opto path costs 18 µs of propagation delay, which is common-mode to every camera
  and therefore cancels out of the skew.

### ESP32 pin map — provisions THREE cameras although only two exist

| ESP32 pin | Direction | Peripheral | Wiring |
|---|---|---|---|
| GPIO 25 | out | MCPWM0 generator (same signal, fanned via GPIO matrix) | → 560 Ω → cam 1 pin 2 |
| GPIO 26 | out | same signal | → 560 Ω → cam 2 pin 2 |
| GPIO 27 | out | same signal | → 560 Ω → cam 3 pin 2 (reserved) |
| GPIO 34 | in (input-only) | MCPWM0 capture ch. | ← cam 1 pin 4, 1 kΩ to 3.3 V |
| GPIO 35 | in (input-only) | MCPWM0 capture ch. | ← cam 2 pin 4, 1 kΩ to 3.3 V |
| GPIO 36 (board label **`VP`**) | in (input-only) | MCPWM0 capture ch. | ← cam 3 pin 4 (reserved, mutually exclusive with the loopback) |
| GPIO 39 (board label **`VN`**) | in (input-only) | MCPWM0 capture ch. | trigger loopback, jumpered from GPIO 25 |
| 3V3, GND | — | — | pull-up rail; GND to every camera's pin 5 |

Routing ONE generator signal to three pads through the GPIO matrix (rather than toggling
three pins in software) means the three edges are simultaneous to within pad skew, and each
pad sources only ~5.5 mA.

**Every capture — strobes AND loopback — shares ONE capture timer in MCPWM group 0**, so
every number in a `CameraSyncPulse` is a difference of two reads of the same counter and no
clock-domain correction exists to get wrong. That group has exactly three capture channels,
which is the real constraint: two cameras plus the loopback fills it. The third camera is
provisioned in the pin map and in the message, but activating it means setting
`ARTS_SYNC_TRIGGER_LOOPBACK=n` and losing absolute trigger-to-exposure latency — camera 0's
strobe then becomes the reference edge. Inter-camera skew, which is the metric the paper
reports, is unaffected either way. The firmware refuses to compile if both are asked for
rather than silently dropping a channel. (Putting the loopback on MCPWM group 1 would free
the slot, but then absolute latency would be a difference across two independent counters,
which is exactly the kind of caveat this design exists to avoid.)

**ESP32 pins to keep clear:** GPIO 6–11 (SPI flash — the board will not boot), GPIO 0/2/5/12/15
(strapping pins, sampled at reset; 12 selects flash voltage), GPIO 1/3 (UART0 — USB serial and
the micro-ROS transport). GPIO 34–39 are input-only with no internal pull-ups, which is exactly
why they suit the strobe captures and why the pull-ups are external. Note that GPIO 36 and 39
are silkscreened `VP` and `VN` (SENSOR_VP / SENSOR_VN) on the DevKitC and on breakouts for it —
no terminal is marked 36 or 39, and the two aliases are adjacent to each other.

### Driver-side configuration

Start from upstream `follower_example.launch.py` (two externally triggered Blackfly S, master
+ follower exposure controllers) with `trigger_mode: On`, `trigger_source: Line0`,
`trigger_selector: FrameStart`, `trigger_activation: RisingEdge`, `trigger_overlap: ReadOut`,
`exposure_auto: Off`, `compute_brightness: True`, chunk mode on with `FrameID`/`ExposureTime`/
`Gain` enabled, and `max_exposure_time` safely under the trigger period.

**These are NOT global shutter sensors, and it changes what a timestamp means.** Verified
against the FLIR spec 2026-08-17: BFS-U3-63S4C is a Sony IMX178 running *rolling shutter with
global reset*. Global reset means every row starts integrating at one instant, so a single
well-defined exposure start exists and the `ExposureActive` rising edge marks it — the sync box
times the right thing. But only the start is common: each row's exposure *ends* staggered by
its readout, so exposure lengthens down the frame and one frame's light-collection span is the
exposure time plus the full sensor readout, ~17 ms at 3072×2048. That is set by the line rate,
so **lowering the frame rate does not shorten it**; shrinking the recorded ROI vertically does.
Two consequences: at the 60 mm/s tool-speed limit roughly 1 mm of tip travel is smeared across
the frame's vertical extent (a stated caveat, same order as the RCM budget), and there is a
top-to-bottom brightness gradient that reads as vignetting or bad white balance if you are not
expecting it. Full reasoning in AGENTS.md decision 15.

**`ExposureActive` mode cannot be set on this camera, so it must be measured.** On a
global-reset sensor the strobe can assert at first-row exposure start (*Any pixel*) or last-row
exposure start (*All pixels*), and those differ by the entire readout — choosing wrong adds a
silent ~17 ms to every absolute timestamp while leaving the stereo skew perfectly correct,
because the error is common-mode. The instruction used to be "assert *Any pixel* explicitly".
**Verified 2026-08-18 that this is impossible here:** the full GenICam node map of
BFS-U3-63S4C at firmware 1801.0.9.0 contains no `ExposureActiveMode` node. The only `Active`
features present are `Device Indicator Mode`, `Chunk Mode Active`, `SequencerSetActive` and
`Line Source: ExposureActive` itself.

That costs nothing, because rung 5d separates the two cases by three orders of magnitude:
hundreds of microseconds for first-row assertion against ~17 ms for last-row, with no
ambiguous middle ground.

**MEASURED 2026-08-18 on camera 24057131: 711.7 µs, so this firmware asserts at FIRST-ROW
exposure start (*Any pixel*).** Spread 8.5 µs, all 240 pulses captured, with the exposure
width simultaneously reading 4977.2 µs (spread 0.15 µs) against the camera's own reported
`ExposureTime` of 5004 µs — a 26.8 µs shortfall that is the strobe's rise delay, recoverable
per frame from the `meta` topic and common-mode across cameras.
The budget behind 711.7 µs is `TriggerDelay` 379 µs — the model's floor, present whether or
not anyone sets it — plus the optocoupler in and out plus the strobe's rise through the
pull-up. An earlier estimate here said ~50–60 µs by forgetting `TriggerDelay` entirely; the
conclusion is untouched, because the discriminator is three orders of magnitude wide.

**The polarity had to be measured too, and the expectation was wrong.** An opto-coupled
output described as open-collector suggests the line is pulled LOW when asserted, and the
firmware defaulted to capturing the falling edge on that basis. On this camera the pin idles
LOW against the external pull-up and RISES while exposing. The consequence is the reason
this whole box exists: capturing the wrong edge produced 5687.7 µs with an 8.5 µs spread —
stable, tight, and completely plausible — which was silently `TriggerDelay` + exposure +
slew, i.e. the exposure's END. It was caught only by noticing the number tracked the
exposure time. The firmware now captures BOTH edges and reports the exposure width beside
the delay, so the same mistake would announce itself: a mislabelled edge makes the width
come out as the trigger period minus the exposure.

### Camera defaults measured from the node map, and which ones lie in wait

Dumped from camera 24057131 with `NodeMapInfo` on 2026-08-18, immediately after a power cycle,
with `User Set Default: Default` — so these are factory defaults, not leftovers from the 22.04
work.

| Feature | Default | Action |
|---|---|---|
| `Line0` | `Input`, `OptoCoupled`, source `Off` | already right for the trigger input; set explicitly anyway |
| `Line1` | `Output`, `OptoCoupled`, source `ExposureActive` | already right for the strobe; set explicitly anyway |
| `TriggerDelay` | **379 µs**, on every selector — and this is the FLOOR, not a chosen default (the camera rejects 9 with `OutOfRangeException`, so upstream's `value >= 9` comment is wrong for this model) | **set explicitly and identically on both cameras.** Two untouched cameras agree because both sit at the floor, but it is settable above it and per-camera, and the 379 µs is a term in every trigger-to-exposure budget |
| `TriggerOverlap` (FrameStart) | `Off` | set `ReadOut` |
| `ExposureAuto` | `Continuous`, `ExposureTime` 14996 µs | set `Off` with a fixed time |
| `ChunkModeActive` | `0` | must be `1`, plus `chunk_enable_frame_id` |

**`TriggerDelay` is the dangerous one.** It is per-camera, and two cameras left on differing
values produce a skew that the hardware measures perfectly and that describes nothing physical
— precisely the failure this sync box exists to eliminate, arriving through a knob nobody
thought to look at. The driver's parameter file documents the floor as 9. Set it, and record
the value beside every skew figure.

**`ChunkModeActive: 0` is quietly load-bearing too.** Decision 15's first hop joins images to
pulses by integer-matching the driver's `ChunkData` `FrameID` against
`CameraSyncPulse.pulse_index`. With chunk mode off, that `FrameID` is not transmitted at all,
so the join has nothing to work with.

That `Line0` and `Line1` already default to exactly what ARTS needs is convenient, and is not a
reason to omit them from the parameter file: a vendor default is not a guarantee, and the point
of the file is that a fresh camera behaves identically.

**A camera booting from a saved UserSet broke that assumption (found 2026-10-27, Stage C of
the PCB test).** Camera 24057132 now boots with `User Set Default: UserSet1`; it was saved
by collaborator A with the Spinnaker GUI, after the August breadboard runs (which used the
factory set).
`UserSet1` has `ISP Enable: 1` (plus sharpening, gamma 0.4, fixed white balance), and the
on-camera ISP caps the sensor line rate: 40 099 lines/s, `Device Max Throughput` 119.6 MB/s,
17.25 fps at 3072×2048, against 128 700 lines/s and 59.6 fps with the factory `Default`
loaded. Under a 30 Hz trigger the symptom is `IN: 17.2 Hz` and a trigger→exposure delay
spread uniformly over 0–33 ms, because a trigger arriving mid-readout waits a variable
time. Neither parameter file wrote `IspEnable`, so the flash value won.
`blackfly_s_arts.yaml` now defines `isp_enable` and the triggered launch writes `False`;
verified from the failing state (`UserSet1` reloaded first): 30.00 Hz, delay 708.17 µs.
**Resolved 2026-10-28: both launch paths load the factory `Default` set into camera RAM
first.** `cameras.launch.py` sets `user_set_selector: Default` and fires `user_set_load`, and
both definition files list those two entries first, so every later write lands on the
factory set. Nothing is written to flash: both cameras still boot `UserSet1`
(`UserSetDefault` untouched, so collaborator A's stack keeps its tuning).

Why not the alternatives. A node-map diff (`Default` against `UserSet1`, all selectors
expanded) shows `UserSet1` is identical on both cameras and differs from factory in more
than ISP: frame-rate limiter on at 18.88 fps, gamma 0.40 (factory 0.80), white balance fixed
(factory auto), plus colour matrix, sharpening and saturation, which are active only with ISP
on. A saved `UserSet0` on 24057132 equals `UserSet1`. Neither path wrote gamma or white
balance, and both exist with ISP off, so forcing ISP alone (option a) left them inherited.
Collaborator A made the set to make two cameras and optical channels look alike. That is a
valid aim, but a replicating lab's cameras will not carry it, so the published configuration
must not depend on it. Per-camera matching, if needed later, goes in the launch file, explicit
and versioned. Setting `UserSetDefault` back to `Default` (option b) would have written
flash and changed A's setup; it was not done.

Verified from each camera's real boot state (`UserSet1` loaded into RAM before every launch):
- Triggered: both 30.00 Hz, no refused write. A dump after configuration differs from factory
  only in values the triggered path writes (trigger, exposure, gain, chunks) and quantities
  derived from them; the two cameras' states are identical.
- Free-running: both 40.00 Hz (with `UserSet1`'s ISP this path was capped at 17.25 fps).
  Differences from factory are the written frame rate and chunks, plus the auto loops'
  exposure (14 996 µs) and gain.
- Timing unchanged: 60 s triggered bag, skew L − R +2.752 ± 3.203 µs, 0 missed strobes
  (E3 baseline −2.793 ± 3.209 µs, taken before this change).

Dumps, logs and the check bag are in `$ARTS_LOCAL/camera_usersets_20261028/`.

**Sync box reconnects to a restarted agent (firmware `434879c`, 2026-10-28).** Over the serial
transport a best-effort publish succeeds with no agent listening, so the publish-failure reboot
never fired: after the agent was killed the board kept triggering, published nothing, and a new
agent never saw it until the board was reset by hand. The firmware now pings the agent every
2 s and reboots after 3 consecutive misses. Verified: agent SIGKILLed, a new agent started
3 s later, pulses back 3.0 s after that with `pulse_index` restarted (2), 30.003 Hz. Triggering
pauses for the reboot and `pulse_index` restarts, so the gap is visible in a recording.

**Acquisition stop before the factory load (triggered path, 2026-10-28).** A camera whose
driver was killed rather than shut down keeps acquiring, and then `UserSetLoad` and
`PixelFormat` are not writable: the next launch was stopped by strict config. The triggered
definition file now executes `AcquisitionStop` first. Verified: SIGKILL of the camera
container, then relaunch, configured cleanly at 30.00 Hz. The free-running path uses
upstream's file, which has no such entry, so it still needs a clean shutdown or a power cycle.

**Colour preset (triggered path).** After the factory load, the triggered path writes
collaborator A's colour tuning explicitly: `BalanceWhiteAuto Off`, `BalanceRatio` Red
0.927368 and Blue 2.98132, `GammaEnable 1`, `Gamma` 0.4. These are the values in A's
`UserSet1`, identical on both cameras. Why fixed and not auto: white balance and gamma are
applied in the camera to the Bayer data that is RECORDED, not only to a display. Measured
2026-10-28 on 24057132 by stepping them live while recording Bayer-site means of one scene
(25 ms, 24 dB): blue/green 0.42 at ratios 1.0/1.0, 1.00 at A's ratios, which is the blue
ratio after gamma (0.42 × 2.981^0.8). Gamma 0.4 against 0.8 raised all means about 3×. Every
step was reversible. So auto white balance would put a scene-dependent drift into the data,
and even factory gamma 0.8 means "raw" Bayer is not linear sensor output. Not taken from
`UserSet1`: ISP, colour matrix, sharpening and light-source preset (ISP-only, and the ISP caps
the rate at 17.25 fps; do colour correction on the host if wanted), and A's 30 ms / 13 dB
exposure (the trigger path fixes its own). The free-running path keeps factory auto: it is the
rate-benchmark path, and its upstream definition file has no white-balance entries.

How to change it: `white_balance_red`, `white_balance_blue` and `gamma` are launch arguments
of `cameras.launch.py` (defaults = A's preset). To re-tune for another light source or camera,
point the scope at a matte grey target under the working light, record Bayer-site means of
`image_raw`, and adjust the two ratios until R, G and B agree. Change values only between
recordings: they alter the recorded data, so record them beside every dataset.

Consequences to know about:
- **A refused write now stops the launch** (`strict_config`, default true). The driver only
  logs refused writes as warnings, so the launch watches its output for the driver's own
  failure phrasings and shuts down, naming the parameter. It was exercised with an
  out-of-range gain. The first version of the marker list missed the exception phrasing
  (`param gain Spinnaker: …OutOfRangeException`); that is how the gap was found.

**Roles and view rotation (settled 2026-10-28).** 24057131 is LEFT, 24057132 is RIGHT, and
the view is rotated 180° for display. The 2026-10-27 entry that made 24057132 LEFT was wrong:
on lit, same-pulse pairs rotated as displayed, it gives disparity xL − xR = −22.9 px, and the
corrected roles give +22.6 px (std 0.1 over 40 pairs); the operator confirmed both roles and
rotation on a live view. A disparity check on a dark image had first agreed with the wrong
roles (+8.4 px, correlation response 0.26), so do this check lit. Consequences:
- 24057132 stays on sync-box terminal J1 (capture channel cam0, host port P3), so cam0 is now
  RIGHT. Run `analyze_sync.py --cam0-meta /camera_right/meta`; the alphabetical default pairs
  the wrong channels (measured r = 0.45 against 0.992) and the analyzer now warns below 0.9.
  Its signed skew is cam0 − cam1 = R − L.
- Images carry `header.frame_id` `scope_left_optical_frame` / `scope_right_optical_frame`
  (decision 12), set in `cameras.launch.py`. They are sensor frames: no flip is applied to the
  data.
- The 180° roll is applied in two places only: the display path rotates each eye after
  debayering (rotating Bayer bytes changes the CFA phase), and the scope head's nominal
  transform carries it as Rz(π) about the optical axis until hand-eye calibration measures it.
  Never `ReverseX`/`ReverseY`: they change the recorded sensor-frame data (decision 12) and
  reverse the rolling-shutter row order against which the strobe was measured. Teleop's
  camera-frame mapping must use the displayed (rotated) frame, or hand motion maps mirrored.

**One-row readout shift (open, found 2026-10-28).** About 1 % of triggered frames come out of
the camera shifted down one row, and the camera then applies its white-balance gains as if the
pattern were unshifted, so the frame flashes a different colour for one frame. Evidence: both
green sites of a bad frame differ (28.5 against 39.1; equal in normal frames); predicting the
four site levels from a normal frame under that model fits within 4 %; phase correlation finds
dy = +0.98 row. Rates in 10 s bags: 6/650, 7/625 (`TriggerOverlap Off`), 6/628 (`ReadOut`),
6/657 at 15 ms exposure, and 0/860 free-running. Both cameras, at different pulses, so not the
shared trigger line. Timing is unaffected; image data is. The colour test alone is not a
reliable flag (it depends on the scene's blue/red balance); detect against neighbouring frames
by a ±1-row phase-correlation jump. Next: TriggerDelay, image height, camera firmware, FLIR.
Under 2×2 binning (60 s, 2026-10-28) it persists at the same rate: 19 of 1840 (left) and 14
of 1836 (right) frames flagged by G1/G2, with matching in-and-out vertical jumps of ~0.9
binned row against the neighbouring frames.

**Viewing full-resolution images live.** A Python subscriber receives a 6 MB `image_raw` at
~1 frame/s best effort (one lost fragment of ~100 drops the frame) and 1–15 reliable. A
reliable full-resolution viewer running during a recording cost the recorder ~120 meta
frames per camera per minute, against 0 without it. Never run such a cross-process viewer
while recording. The surgeon's display runs inside the camera process instead; see
[`console_display.md`](console_display.md).

**Recording resolution: 2×2 binning (2026-10-28).** The triggered path records 1536×1024
BayerRG8, binned in the camera (`binning:=2`, the default; `binning:=1` is 3072×2048). Why,
and the measurement that binning is colour-aware and leaves sensor timing unchanged: decision
5, amendment of 2026-10-28. 60 s triggered run with binning, both cameras: 30.000 Hz, skew
(24057132 − 24057131) −2.767 ± 3.204 µs, worst 10.75 µs, 0 missed strobes. Recording the two
image topics with meta and pulses in one `ros2 bag record` process lost 7 and 10 image frames
and 125 and 111 meta frames; the same run without images lost 0 meta frames (per-pulse join
r = 0.992). The loss is the cross-process image transport, not binning; the fix is the
composed recorder in [`console_display.md`](console_display.md), and E4 measures it.

**Storage (2026-10-28).** Both cameras binned fill the image disk at 94.4 MB/s measured
(5.7 GB/min, 340 GB/h). The workstation has ONE 1 TB NVMe, shared with another OS; Linux has
a 500 GB partition, so decision 5's dedicated image NVMe is not met on this host, and there
is no free M.2 slot for one (operator, 2026-10-28). Until a larger disk is installed (a
system reinstall, deferred past ISMR), the workflow is: record one session, export it with
`arts_record/scripts/export_bag.sh BAG DEST --delete` (copies, verifies every file by sha256
read back from the destination, writes `SHA256SUMS`, and deletes the local copy only if all
match; verified on success, an unclosed bag and a deliberately corrupted copy), then record
the next. Export between recordings only: an external USB disk would share a controller with
a camera (`docs/host_usb_topology.md`). rosbag2 has
no free-space limit, so every `image_bag` launch also starts `arts_record/disk_guard.py`: it
stops the recorder (closing the bag cleanly) when free space falls below
`image_bag_reserve_gb` (default 30). Verified on the failure path: with the reserve set 3 GB
below free space, it stopped the recorder after 32 s and the bag read back complete (965
frames per camera, metadata written); cameras kept running. Before a planned recording, run
`ros2 run arts_record disk_guard.py --check-only --path <bag> --duration-s <s>` (exit 2 =
not enough room).

**Open item — there is no `LineSource` parameter.** The shipped `blackfly_s.yaml` exposes
`line1_selector` and `line1_linemode` but nothing mapping to
`DigitalIOControl/LineSource`, so `ExposureActive` cannot be routed onto Line 1 through the
stock parameter file. Two ways out: ship an ARTS-side superset parameter file (upstream's
definitions plus `line1_linesource`, and upstream the addition), or provision `LineSource`
once into the camera's `UserSet` with a PySpin script. Prefer the parameter file — device
flash is invisible state, and invisible state is not reproducible. If the UserSet route is
taken anyway, the script belongs in `deploy/modules/flir.sh` so it is versioned.

**THE DEFINITION FILE'S ORDER IS LOAD-BEARING — read this before writing the superset.**
Traced through `spinnaker_camera_driver/src/camera.cpp` on 2026-08-18. The line parameters are
not independent: `line0_selector`, `line1_selector`, `line2_selector` and `line3_selector` all
map to the *same* GenICam node, `DigitalIOControl/LineSelector`, because GenICam configures
digital I/O through a selector — you point `LineSelector` at a line, and `LineMode` /
`LineSource` then apply to whichever line is currently selected.

So correctness depends on write order, and the write order comes from the **parameter
definition YAML**, not from the launch file. The chain: `readParameters()` registers the
on-set-parameters callback (camera.cpp:438) *before* `createCameraParameters()` runs
(camera.cpp:179); `readParameterDefinitionFile()` builds `parameterList_` by `push_back` in
file order; `createCameraParameters()` declares each entry in that order, and each declaration
carrying a launch-supplied override fires `parameterChanged()` immediately, which writes to the
camera there and then.

Two practical consequences. `line1_linesource` must be placed **after** `line1_selector` in the
definition file, or it will be applied to whatever line happened to be selected last. And
reordering entries in that file is a functional change, not cosmetic — it silently
reconfigures different physical pins.

### Bring-up ladder — nothing touches a camera until the pulse is proven

Bench procedure for each rung, written for someone who has not wired a GPIO before — resistor
identification, breadboard/screw-terminal layout, meter checks, symptom table:
[`camera_sync_wiring.md`](camera_sync_wiring.md). **Start at its section 1 and read straight
through**; that file is ordered chronologically (understand → reference maps → do), and the
rungs below are its section 9. Verifying the pigtail colour-to-pin mapping and the three
standing safety rules come *before* any wiring.

Authoritative status lives in AGENTS.md's Stage 5 entry; the boxes below are the same state
kept next to the procedure, with each rung's headline result. All five rungs passed on the
breadboard rig between 2026-08-18 and 2026-08-19.

- [x] **5a — the pulse exists, no cameras.** Meter on GPIO 25 against GND, DC volts, in the
      `BLINK` build (2 Hz, 50 % duty): the display swings 0 ↔ 3.3 V. No LED needed, and the
      meter also confirms the logic level. Production timing (~100 µs at 30 Hz, 0.3 % duty)
      reads as ~10 mV average and looks like a dead pin, which is why `BLINK` exists.
      PASSED 2026-08-18: reading fluctuated ~1.5–2.3 V, i.e. the meter averaging the square
      wave around its 1.65 V mean.
- [x] **5b — loopback.** Jumper GPIO 25 to GPIO 39 (and to GPIO 34 for a channel check). The
      firmware should report a capture delta of ≈0. This validates the capture path with no
      camera at risk.
      PASSED 2026-08-18, and it made the capture clock a measured quantity: 80 MHz,
      12.500 ns/tick. Trigger period 33 333.000 µs with **spread 0.000 µs** over 418
      intervals. The loopback jumper is permanent wiring in the two-camera build, not a
      temporary test — it is what opens each pulse record.
- [x] **5c — one camera.** OPTO_GND (pin 5) first, then pin 2 through its resistor. Confirm
      the camera transitions from waiting-for-trigger to streaming at the pulse rate.
      PASSED 2026-08-18: camera 24057131 at a flat `IN: 30.00 Hz`, matching the firmware.
      Read `IN`, not `OUT` — the driver only publishes when subscribed.
- [x] **5d — one strobe.** Add cam 1 pin 4 with its 1 kΩ pull-up; confirm a stable
      trigger-to-exposure delta.
      PASSED 2026-08-18: **729.5 µs, spread 8.6 µs over 797 pulses**, which answers
      decision 15's open question by measurement — the firmware asserts `ExposureActive` at
      FIRST-ROW exposure start. The two candidates were ~430 µs and ~17.4 ms, so there is no
      interpretive latitude. Two findings came out of this rung: the strobe is active-HIGH
      (not the open-collector active-low that was assumed), and the pull-up must be 1 kΩ
      rather than 10 kΩ — at 10 kΩ, 4–5 % of strobes were missed outright.
- [x] **5e — both cameras.** Measure skew both ways (hardware capture; `camera_time` spread
      from the `meta` topics) and commit the baseline under `docs/benchmarks/`.
      HARDWARE, SELFTEST console, 2026-08-18: **|cam0 − cam1| mean 3.4 µs, max 10.6 µs over
      720 pulses**, against a < 1 ms target.
      BOTH WAYS, from a micro-ROS bag, 2026-08-19: hardware **mean 3.52 µs, max 10.44 µs
      over 1001 pulses** (systematic 2.81 µs) — the same number on an independent sample —
      and the `camera_time` cross-check agreeing at std 22.8 µs / 87 µs peak-to-peak, which
      is larger than hardware as it must be. Report:
      [`../benchmarks/camera_sync_skew_breadboard_20260819.md`](../benchmarks/camera_sync_skew_breadboard_20260819.md).
      The residual ±8.6 µs per camera is the sensor's line period, not measurement noise;
      see `camera_sync_wiring.md` §9.
      **NOT YET THE COMMITTED BASELINE**, and deliberately so, on three counts: breadboard
      rig, development laptop rather than the production workstation, and two separate driver
      processes rather than the composed container of item 1 above. Per decision 14 the
      committed figure comes from the PCB.
      (An earlier version of this line listed "the single-camera driver rather than the
      synchronized one" as a fourth shortfall. It is not one — the single driver is the
      deliberate choice per the decision 5 amendment, and the synchronized driver is
      cancelled. The process model was the real defect, and that is item 1.)

**Known failure mode to recognize, not debug:** with `trigger_mode: On` and no pulses
arriving, the camera publishes nothing and looks broken. "No images" usually means the ESP32
is not running, not that the camera is faulty.

### Bandwidth wall — the pair cannot run at 40 Hz on one controller

3072×2048 Bayer8 = 6.29 MB/frame, so 40 Hz stereo is ~503 MB/s against roughly 400 MB/s
usable on one 5 Gbps link. This is why the second camera was invisible while the first
streamed in Stage 1 — bandwidth, not a driver fault. The committed 40 Hz baselines are
single-camera numbers and remain valid as such.

**Controller map of the development laptop, measured 2026-08-18.** An earlier note here said
"Bus 004 is empty, split the pair across the two controllers", which is misleading: there are
THREE controllers, and Bus 004 cannot be reached from any USB-A port.

| Buses | PCI | Controller | Physical port |
|---|---|---|---|
| 001 + 002 | `00:14.0` | Intel Cannon Lake PCH USB 3.1 xHCI | **every USB-A port on the machine** |
| 003 + 004 | `01:00.2` | NVIDIA TU106 USB 3.1 | the GPU's VirtualLink USB-C only |
| 005 + 006 | `06:00.0` | Intel JHL6340 Thunderbolt 3 | the Thunderbolt USB-C; powers up on attach |

Each pair is one controller exposing a USB2 root hub plus a USB3 root hub. So both Blackflys
land on Bus 002 no matter which side of the laptop they are plugged into, and they share that
controller's single 5 Gbps uplink. Note the Bus 003/004 controller sits on the discrete GPU,
which is worth remembering if GPU power management ever makes a camera disappear.

**Resolved on this machine with a USB-C dongle**, which turned out to work: camera 24057131 (L)
sits on Bus 002 through a USB-A port and 24057132 (R) on Bus 006 through the dongle, both at
5000M, i.e. on two independent controllers. An intermediate `lsusb -t` showing the dongle's hub
with no camera beneath it was transient rather than a power limit. If a dongle ever does fail
that way, the fixes in order are a **passive USB-C to USB-A adapter** (no hub) or a
**self-powered** hub — a Blackfly S draws roughly 3 W and a bus-powered hub can run out.

**The replication requirement is a controller budget, not a dongle.** What a reproducing group
needs is stated as: **one USB3 host controller per camera, each able to sustain ~400 MB/s**, or
a reduced rate/ROI that fits whatever is available. The dongle is a development-laptop
workaround for a machine whose every USB-A port hangs off a single PCH controller; it is not
part of the design and must not appear in the build guide as if it were. A workstation either
has two independent controllers or takes a PCIe USB3 card that provides them. Also do not take
the final committed dual-camera benchmark over a Thunderbolt-tunnelled controller — it is fine
for bring-up, but the number that goes in the paper should come from the production workstation,
with the topology recorded beside it (the harness already records the host).

**Agreed arrangement for the Stage 5 bring-up (2026-08-18):** keep L on Bus 002 and R on
Bus 006. The argument is debugging discipline rather than throughput — with the cameras on
separate controllers, a frame missing at rung 5e means a trigger fault, whereas on a shared
controller it could equally be a USB overrun, and separating those two costs nothing here.
Sharing Bus 002 is perfectly adequate for 5c and 5d, which trigger one camera at a time.

**Serial format trap.** sysfs reports the serial in hex and Spinnaker in decimal:
`/sys/bus/usb/devices/*/serial` = `016F152B` is the same camera as
`DeviceSerialNumber : 24057131` (0x016F152B = 24057131; `016F152C` = 24057132). Anyone matching
udev output against a driver config will otherwise conclude they are looking at two cameras
that do not exist.

**This does not gate the sync work, and it is worth being explicit about why.** What Stage 5
measures is L/R skew, and the hardware number comes from the ESP32's capture counter, which
never touches USB at all. The software cross-check reads `camera_time` out of the `~/<cam>/meta`
topics, which are a few hundred bytes per frame. Neither needs full-resolution streaming from
both cameras at once. So bring 5c–5e up at whatever the link comfortably carries — 25 Hz
(~315 MB/s, MOPS's stereo rate) or a reduced ROI — and treat "both cameras, full resolution,
full rate, zero drops" as the separate benchmark deliverable it is. Reducing the ROI vertically
has a bonus documented under AGENTS.md decision 15: it shortens the rolling-shutter readout
stagger proportionally.

## Jazzy migration — what changed

Migrating the pilot from Humble/22.04 to Jazzy/24.04 was not a rename. Five findings
matter for this module, and the first one changes what the image contains. The two build
findings below surfaced only when the module was first installed **bare metal** on 24.04 — the
Docker path had masked both, and the second of them has a successful build as its symptom.

### SpinView_QT and libSpinVideo do not work on Ubuntu 24.04

Spinnaker 4.2.0.88 officially supports 20.04 and 22.04 only. Inspecting what the shipped
binaries actually link shows the split is clean rather than fatal:

| Component | Links | On noble |
|---|---|---|
| `libSpinnaker` | libc, libstdc++, libgomp, libusb | works |
| CLI samples (`Enumeration`, `Acquisition`, `ChunkData`) | libSpinnaker only | works |
| `libSpinVideo` | `libavcodec.so.58`, `libavformat.so.58`, `libavutil.so.56`, `libswscale.so.5` | **broken** — jammy ffmpeg ABI |
| `SpinView_QT` | Qt5 + libSpinVideo | **broken** by association |

The ROS 2 driver uses `libSpinnaker`, so the ARTS data path is unaffected. SpinVideo is a
video-file writer ARTS does not use — we record MCAP. `flir_oem_install()` therefore skips
the `libspinvideo` and `spinview-qt` debs rather than shimming in foreign-ABI ffmpeg
libraries, which would be a fragile way to keep a GUI we do not need.

**Consequence for Stage 1:** its acceptance test was "SpinView_QT streams live inside the
container". That test cannot be repeated as written. The equivalent check on Jazzy is the
CLI sample tools, which exercise the same enumeration and acquisition path:

```bash
docker run --rm -it --privileged -v /dev/bus/usb:/dev/bus/usb arts-flir:jazzy Enumeration
docker run --rm -it --privileged -v /dev/bus/usb:/dev/bus/usb arts-flir:jazzy Acquisition
```

Bare metal, run from a scratch directory because `Acquisition` writes JPEGs into `$PWD`:

```bash
/opt/spinnaker/bin/Enumeration
cd "$(mktemp -d)" && /opt/spinnaker/bin/Acquisition
```

Passed bare metal 2026-08-18: 2 cameras enumerated on USB Interface 0, ten 3072×2048 frames
saved from each, `DeviceCurrentSpeed : SuperSpeed`. One caveat that matters when reading this as
a bandwidth result — **it is not one.** `Acquisition` opens the cameras one after another, so it
exercises the data path, not two simultaneous streams.

If a GUI turns out to be genuinely needed, the fix is a Spinnaker release that supports
24.04, which is a pin bump under the usual rules: bump `oem_drivers/spinnaker/VERSION`,
rebuild, re-benchmark, commit the new baseline beside the old one.

### An image-shrinking idiom in a shared module function deleted the host's apt lists

Found 2026-08-18, on the first bare-metal install of this module. `flir_oem_install()` used to
end with an unconditional `rm -rf /var/lib/apt/lists/*`. In a Dockerfile that is correct and
standard. On bare metal those lists belong to the entire machine, so the function quietly
deleted them — and apt then truthfully reported that it could not find packages that are
perfectly installable. It produced two different-looking failures in `flir_overlay_build`:

```
executing command [sudo -H apt-get install -y ros-jazzy-ament-cmake-clang-format]
E: Unable to locate package ros-jazzy-ament-cmake-clang-format
```

```
executing command [sudo -H apt-get install -y ffmpeg]
E: Package 'ffmpeg' has no installation candidate
```

Both packages exist and install normally. Nothing was wrong with either package, the repo, or
the pin — the lists were gone. The failure also surfaced two functions away from its cause, in
rosdep, which is what makes this class of bug expensive.

It compounded itself: the same function's `apt-get update 2>/dev/null || true` ran **without
sudo**, so on a host it always failed, always silently, and never repaired the damage.

Both are fixed. The cleanup is gated on `[ -f /.dockerenv ]`, and the update runs under
`$SUDO` and prints a loud warning if it fails instead of hiding it.

**The pattern to copy is `dds.sh`,** which keeps a separate `dds_image_install()` that only
`Dockerfile.base` calls, distinct from `dds_host_deps()`. `flir.sh` branches rather than
splitting because the deb extraction genuinely is shared between the two paths.

The general lesson for every module script under the decision-4 contract: **a function
consumed by both paths must contain no container housekeeping.** Sharing the function is what
prevents the bare-metal and Docker paths from drifting, and it is also exactly what lets a
Dockerfile idiom reach into the host.

**A wrong diagnosis worth recording, because the reasoning looked sound.** The clang-format
failure above was first blamed on the rolling-repo hazard in AGENTS.md's pinning policy. The
argument was tidy: rosdep only mints a `ros-jazzy-…` key for a package rosdistro lists as
released, `apt-cache` showed 43 sibling `ros-jazzy-ament-*` packages and no clang-format entry
at all, so "released on paper, absent from the archive" fit the evidence — and it even
explained why the byte-identical Docker build had succeeded on 2026-08-10.

It was wrong. **`apt-cache` reads the binary cache under `/var/cache/apt/`, which survives
deletion of the lists**, so it keeps answering plausibly about a repository apt can no longer
resolve anything from. With lists restored, `ros-jazzy-ament-cmake-clang-format` is present at
0.17.5 and installs without complaint. A `--skip-keys ament_cmake_clang_format` plus
`-DBUILD_TESTING=OFF` workaround was added and then reverted, because carrying a fix for a
phantom is worse than the phantom: it hides the real bug and it silently stops building
upstream's tests forever.

The rule that falls out: **`apt-cache` output is not evidence about repository contents unless
`ls /var/lib/apt/lists/*Packages*` is non-empty.** Check that first, before reasoning about
whether a package exists.

### The driver silently downloads an unpinned Spinnaker if the SDK is missing

Found 2026-08-18 while answering "how does a future user obtain the SDK". This one has the
longest reach of anything in this document, because its symptom is a *successful build*.

`spinnaker_camera_driver/CMakeLists.txt`:

```cmake
find_package(SPINNAKER QUIET)
if(NOT SPINNAKER_FOUND)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(SPINNAKER_LIBRARIES SPINNAKER_INCLUDE_DIRS)
endif()
```

`cmake/download_spinnaker` maps the OS codename to a version and fetches a tarball:

| codename | Ubuntu | Spinnaker fetched |
|---|---|---|
| `focal` | 20.04 | 4.2.0.88 |
| `jammy` | 22.04 | 4.2.0.88 |
| **`noble`** | **24.04** | **4.4.0.99** |
| `resolute` | 26.04 | 4.4.0.99 |

from `https://eventvisionresearch.com/Spinnaker/…` — a third-party host, not FLIR.

So on this machine, a user who skips `flir_oem_install` gets **no error**. They get a working
overlay built against Spinnaker 4.4.0.99 from an unofficial mirror, while
`oem_drivers/spinnaker/VERSION` says 4.2.0.88, and every benchmark recorded afterwards
describes a stack nobody selected. It also quietly routes around the FLIR account wall, which
is the mechanism the pinning policy relies on for this class of dependency.

This is the same failure shape as the Ruckig incident in AGENTS.md — a pinned version defeated
by something that arrived through a back door — and it gets the same remedy. `flir_overlay_build()`
now refuses to start unless both of these exist:

```
/opt/spinnaker/lib/libSpinnaker.so
/opt/spinnaker/include/Spinnaker.h
```

and its error message names the auto-download, so the next person does not have to rediscover
it. **The lesson to carry: "the driver built fine" is not evidence that the OEM install
worked.**

**A useful side effect.** That table is upstream telling us FLIR ships **4.4.0.99 for 24.04**,
while ARTS pins 4.2.0.88 — a release that officially supports 20.04 and 22.04 only, and the
sole reason SpinView_QT and libSpinVideo are broken here. Bumping the pin may retire that
limitation outright. It is a benchmarked act under the usual rules (bump `VERSION`, re-mirror
from FLIR, rebuild, re-run the benchmark, commit the new baseline beside the old), and it is
not on the Stage 5 critical path, so it is recorded rather than done mid-bring-up.

### MCAP needs no special handling

`-s mcap` and the explicit storage-plugin install were Humble workarounds. MCAP is Jazzy's
default rosbag2 storage, so the flag is gone from `flir_bench.launch.py` and from
`mcap_image_config.yaml`'s usage line. `--storage-config-file` still applies and is still
how the chunking and Zstd settings are passed.

### Benchmark reports now record the host they ran on

The Stage-3 baseline recorded the image digest but nothing about the machine, so a rate or
CPU number could not be compared against a later run on a different kernel. `run_flir_bench.sh`
now writes `host_config.json` (kernel release and preempt model, isolated CPUs, CPU model,
NVIDIA driver, `net.core.rmem_max`, `usbfs_memory_mb`, Docker version) and the analyzer
renders it as a "Host configuration" section, calling out explicitly when no CPU isolation
was active.

### What must be re-run before the numbers count — DISCHARGED 2026-08-10 (Stage 4b)

This section listed the re-validation needed after the Jazzy migration: rebuild both
images, confirm `flir_camera_driver` 3.0.5 compiles under Jazzy, and reproduce Stages 1–4.
All of it was done as Stage 4b — byte-identical 39.99 Hz / 0 drops, committed as
`../benchmarks/flir_bfs-u3-63s4c_20260810.*` (digest `sha256:350845b4b901…`). The original
checkbox form violated the documentation contract anyway (status boxes live in `AGENTS.md`
only), which is how it managed to sit here stale for ten days.

## Exit review — judge the methodology, not just the module

**Written 2026-09-08. Scope caveat, stated first:** this review is recorded BEFORE the two
Stage 5 measurements that still wait on hardware — the committed skew baseline on the
fabricated PCB (decision 14 forbids one from the breadboard) and the "both cameras, full
resolution, full rate, zero drops" number on the production workstation. Neither can change
the verdicts below: they are about how the module was BUILT, and the sync-box figures that
exist (3.4–3.5 µs mean skew, two independent methods agreeing, on a < 1 ms target) are
thirty times inside spec, so a PCB baseline can move the systematic term by microseconds but
cannot reverse a conclusion. Waiting for a Dell shipment and a PCB order to write down what
was learned in August would have been the wrong trade. Those two numbers stay open under
AGENTS.md Stage 5 / Phase 3 and are NOT claimed here.

### 1. Did the module-script / Dockerfile shared-source pattern hold up?

**For `oem_install`, yes; for `overlay_build`, only after being fixed today.** The pattern's
one job is that bare metal and Docker run the identical function, and it produced exactly the
class of catch it was designed for: `flir_oem_install`'s `rm -rf /var/lib/apt/lists/*` was
correct in the image and destructive on the host, and because the function was shared the
defect was found the first time the bare-metal path ran it (2026-08-18) rather than living on
as a permanent difference between the two paths. That is the pattern working.

Where it failed is the part nobody was looking at. `flir_overlay_build` was the only overlay
build that went through a module function; the other five `build.sh` files were independent
copies, and `arts_install.sh` never called a `*_overlay_build` at all — the claim that Docker
and bare metal shared the sequence was true of one module and asserted, in a comment, of all
of them. Two fixes learned in the flir copy (the `set -u`-after-source ordering that killed
every `env -i` build, and the audible `apt-get update`) therefore never reached the others,
and the bare-metal `ur_ros2_ws` build hit the first of them cold on 2026-08-10. An external
review found it (`../repo_structure_triage.md` item 3); it was fixed 2026-09-08 with
`deploy/modules/_common.sh` (`overlay_build_generic`, `overlay_build_dispatch`), every
`build.sh` reduced to a delegate, and the installer dispatching through the module function
where one exists. Verified by building `flir_ros2_ws` (via the installer, 4 packages) and
`ur_ros2_ws` (6 packages, MoveIt excluded through `--exclude`) from empty scratch trees
through the new path; `/opt/ros/jazzy` appears once and last in the resulting chain.

**Verdict: keep the pattern, but it holds only when the shared code is the ONLY copy.** A
comment asserting no-drift is worse than no comment, because it stops the next person from
checking. Remaining duplication: none in the overlay sequence. The `host_deps` functions are
legitimately per-module. What is still missing is a CI check that the functions named in
`deploy/docker/Dockerfile.*` are the ones `overlay_build_dispatch` resolves to — that check is
the mechanical form of the property and belongs in the Phase 0 CI item.

### 2. Actual time versus the ~4-day estimate

**Stages 0–4: on estimate.** Host readiness through the dual-path contract ran 2026-08-06 →
2026-08-10 including the Jazzy re-validation (Stage 4b), with the committed baseline
reproduced byte-identically across two distros. The camera DATA PATH — driver, rate, drops,
MCAP — was never the hard part.

**Stage 5: ~3 weeks and still not closed, and it was not in the estimate at all.** The sync
box was scoped in on 2026-08-17 by decisions 5 and 14 after reading the pinned driver source,
which is the first friction point and the most instructive: the original plan (synchronized
driver, skew from `header.stamp`) would have measured nothing, and that was discoverable only
by reading `time_keeper.cpp`, not the README. The rest of the friction, in order of cost:
the ESP-IDF + micro-ROS toolchain (five build attempts, five traps, each with a misdirecting
error — `catkin_pkg` invisible to the venv, config-gated `REQUIRES` never firing, the
transport baked into `libmicroros.a`, the UART transport not in the component, and the
transport never routing its own pins); the bare-metal FLIR path having never been exercised
until rung 5c was blocked on it; and two hardware predictions that were simply wrong (strobe
polarity, 10 kΩ pull-up), each producing a stable, plausible, incorrect number. Roughly:
one week of firmware toolchain, one week of bench rungs and doc corrections, one week of
PCB design package and layout review.

**Verdict:** the ~4-day estimate was right for what it estimated. The lesson is not "estimate
longer" but "read the pinned upstream source before designing around one of its features",
because the entire Stage 5 scope change came from one afternoon of doing that.

### 3. Did the benchmark contract capture enough to make results comparable?

**No, and every gap was found by needing the number later.** The contract (rate, drops, CPU,
RSS, I/O, image digest) was sufficient to reproduce the 40 Hz baseline across distros, and
insufficient for everything that came after. Missing, in the order they were discovered:
host kernel / RMW / sysctl state (found at the Jazzy migration — two baselines in Docker
were silently CycloneDDS while bare metal was Fast DDS); a machine CLASS instead of a
hostname (found by the anonymity check — `hostname -s` in a committed filename); the USB
controller topology (`lsusb -t`, found at 5e when correlated dropouts needed it);
the device's node-map defaults against what the launch file sets (found from a full
`NodeMapInfo` dump — `TriggerDelay` 379 µs, `ExposureAuto` Continuous, `ChunkModeActive` 0,
each capable of changing a published number with no symptom); the wiring revision (breadboard
versus board, which decision 14 now makes a validity condition); and the subscriber state
(`OUT: 0.00 Hz` beside a healthy `IN` is not a fault, and a report that recorded only OUT
would have been wrong). The skew reports already carry firmware build, pins, resistor values
and the driver used — that is the model for the rest.

**Verdict:** extend the layer-4 report header to a fixed checklist — host (kernel, RMW,
isolcpus, relevant sysctls), machine class, bus topology, device firmware + a diff of the
node map against factory defaults, wiring/board revision, and what was subscribed — and make
`analyze_bench.py` refuse to emit a report with an empty field rather than leaving it to the
author. A benchmark whose conditions cannot be restated is a number, not a measurement.

### 4. Corrected template for the next module (UR arm, then Haply)

What the FLIR pilot says to do differently, each traceable to an incident above:

1. **Module script on day one, delegating to `_common.sh`.** `deploy/modules/ur.sh` with the
   three functions before any hardware is touched; `ur_ros2_ws` gets an owner in
   `overlay_owner_module`. Never a per-overlay copy of the build sequence.
2. **Exercise both install paths at Stage 2, not Stage 4b.** The FLIR bare-metal path was
   first run three weeks after the Docker one and was blocking a bench rung when it failed.
3. **Read the pinned driver source before designing around a feature.** For UR that means
   `ur_robot_driver`'s `read()` blocking behaviour (decision 1 already rests on it), what
   RTDE actually stamps, and how `kinematics_params_file` / the factory calibration reaches
   the description — before writing a benchmark that assumes any of it.
4. **Dump the device's defaults before trusting them.** The FLIR node-map dump found four
   silent traps. The UR equivalent is the controller box's own calibration and safety
   configuration, and `ur_description`'s declared absence of acceleration/jerk limits.
5. **Measure sign and polarity conventions; never derive them.** Strobe polarity and the
   pull-up value were both reasoned correctly from a wrong premise. For UR: joint sign
   conventions against the mount, the 45° nose-down tilt, TCP versus flange.
6. **Open `docs/modules/<module>.md` at Stage 0 with the role header**, keep status ONLY in
   AGENTS.md, and tick a checkbox in the same edit that records the result. When a
   measurement overturns a prediction, edit the explanatory prose too, not just the table.
7. **Bench doc in time order, not topic order** — safety and pin verification before the
   first wire, rungs numbered, a symptom table that names the plausible-wrong-number
   failures, not only the loud ones.
8. **Benchmark report header is the fixed checklist from §3**, from the first baseline.

Recommended order stands: UR arm (Phase 1 needs it, and its module script does not exist),
then the Haply bridge.
