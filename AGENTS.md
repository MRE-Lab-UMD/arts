# ARTS — Autonomous Robotic Translational System

Agent onboarding file. Read this before doing anything in this repo. It states the project
goal, the hard rules, a summary of each locked decision, and where work stands. It is kept
SHORT on purpose: full reasoning lives in `docs/decisions/`, full layout narrative in
`docs/repo_layout.md`, pinning incidents in `docs/dependency_pinning.md`, and the dated
narrative behind each status checkbox in `docs/status_detail/`. Read those when you touch
the area they cover. `README.md` has the human-readable overview and how to run a session; the host setup log is `docs/host_setup_log.md`.

**Keep this file short.** New reasoning goes into the decision record or module doc it
belongs to, with at most a one-line headline here. A paragraph added here is loaded into
every agent session forever.

## What this repo is

The ARTS development repo — it will become the single first-party source of the system (the
`arts_ros2_ws` workspace source plus middle-overlay manifests, OEM/open-source install
recipes, deploy tooling, and docs).

The previous-generation system, **v1.4**, is the reference implementation, available
machine-locally at `reference/v1.4_ref` (a gitignored symlink; what it is:
`~/arts_local/CLAUDE.md`). Never describe it in a tracked file beyond "the v1.4 reference":
its hardware, venue or year together identify the authors. Read it when porting; never push it, never edit
it in place. Reference code is ported into ARTS, not copied.

## Hard rules (break none of these)

1. **Double-anonymous.** This repo becomes an anonymized code capsule for ISMR review. No
   identifying reference to the prior system's name, its authors, collaborators, their
   institutions or usernames, the regional arm distributor, or this machine's username or
   hostname may appear in ANY tracked file. Say "the prior system", "the v1.4 reference",
   "collaborator A". Run `deploy/ci/check_anonymity.sh` before every commit. Credit belongs
   in the paper, never in the tree.
2. **Source only the immediate underlay** (see "Sourcing rules" below). A correct chain shows
   `/opt/ros/jazzy` in `AMENT_PREFIX_PATH` exactly once, at the end.
3. **Pin everything** — never a branch (see "Dependency pinning").
4. **Both install paths are first-class.** Every module ships `deploy/modules/<module>.sh`;
   bare metal and Docker run the same functions. A function consumed by both paths contains
   no container housekeeping (gate it on `/.dockerenv`).
5. **Install failures are recorded exactly, not patched around.** The install recipe is a
   paper claim; a silent workaround falsifies it.
6. **Measure, don't assume** — and exercise the FAILURE path of every check (a guard that has
   only been seen passing is unverified).
7. **When a measurement overturns a prediction, the explanatory prose is part of the edit.**

## Project goal

A replicable, **< $150,000**, open-source 3-arm laparoscopic surgical robot for surgical
autonomy research (teleoperation, imitation learning such as SRT-H, and supervised autonomy),
built only from open-source software plus unavoidable OEM drivers. Any research institution
should be able to buy the BOM, run one install script (or build the Docker images with it), and reproduce
the system.

**Hardware: four stations in one row** (left to right, facing the system; da Vinci-familiar
naming kept deliberately; layout confirmed by the PI 2026-11-02):
- **Patient Cart** — Vention pedestal; 1× UR5 (CB3, 125 Hz) + 2× UR5e (500 Hz); da Vinci Si
  12 mm stereo endoscope fed to 2× FLIR cameras through custom optical channels; 2× da Vinci X
  instruments on custom UR–da Vinci adapters (4-motor disc drives); Karl Storz trainer box
  with cannulas; the main PC (Dell desktop); ARM HOLD relay system; Ethernet switch.
- **Support Tower** — used Karl Storz tower with monitor 4 (tower view) on top; ESU (DRE
  ASG-300) behind the ESU-TAP interposer (energy by pedal or software, both logged); Luxtec
  light source; Bovie SmokeShark II smoke evacuator; Olympus OFP flushing pump (irrigation);
  suction pump.
- **Surgeon Console** — 80/20 frame; 2× Haply Inverse3 (+ VerseGrip, with collaborator A's
  Hall-sensor grip add-on); GOOVIS HMD (plain HDMI stereo display); Savant teleop footswitch;
  monopolar and bipolar ESU pedals.
- **Supervisor Station** — a table with monitors 1 and 2, keyboard and mouse (where the
  operator runs the system), and the ARM HOLD button box.

## Locked architecture decisions (do not re-litigate)

One paragraph each. The full record — evidence, rejected alternatives, amendments — is in
`docs/decisions/NN_*.md` and MUST be read before changing anything in that area.

1. **OROCOS RTT is retired** → ros2_control chainable controllers in TWO rate-grouped
   `controller_manager` processes: 500 Hz for the two UR5e tool arms (+ wrists), 125 Hz for
   the UR5 CB3 camera arm. The stated reason (`read()` blocking on RTDE) is wrong for driver 3.x,
   which reads non-blocking by default (amendment 2026-10-29); the layout stands. Pipeline: see decision 17. Cross-manager joint states via RT-safe buffers.
   KDL → pinocchio, Reflexxes → Ruckig. [01](docs/decisions/01_orocos_retired.md)
2. **Workspace layering:** `/opt/ros/jazzy` (base underlay) → categorized MIDDLE OVERLAYS
   (`ur`, `flir`, `slicer`, `atracsys`, `isaac`, `oem`; each depends only on the base,
   pinned by `.repos`, built in its own `env -i` shell) → `arts_ros2_ws` (all first-party
   `arts_*` packages) → per-project application overlays.
   [02](docs/decisions/02_workspace_layering.md)
3. **ROS 2 Jazzy on Ubuntu 24.04** (migrated from Humble Aug 2026; EOL May 2029). MCAP is the
   default bag format. [03](docs/decisions/03_jazzy_noble.md)
4. **Bare metal AND Docker, both first-class.** `arts_install.sh` and the Dockerfiles run the
   same `deploy/modules/<module>.sh` functions; `host_setup.sh` does what containers cannot.
   OEM installers are build args, never baked into public images.
   [04](docs/decisions/04_bare_metal_and_docker.md)
5. **Time and recording:** all data via rosbag2 MCAP, two recorder processes (state bag;
   image bag on its own NVMe). Images are recorded at 1536×1024 raw Bayer via in-camera 2×2
   binning, not full resolution (amended 2026-10-28; `binning:=1` restores 3072×2048).
   `header.stamp` is never rewritten at record time. ARTS uses
   the SINGLE `spinnaker_camera_driver` with both cameras composed into ONE container (NOT
   `spinnaker_synchronized_camera_driver`, whose stamps are estimates and which discards the
   device clock); record the `~/<cam>/meta` topics — they carry `camera_time`. USB3 cameras
   have no PTP; PTP applies to GigE/networked devices only.
   [05](docs/decisions/05_time_and_recording.md)
6. **Plain ROS 2 naming; no CRTK/dVRK vocabulary.** ARTS is independent of dVRK and owns its
   instrument kinematics/coupling files. [06](docs/decisions/06_plain_ros2_naming.md)
7. **Simulation:** Isaac Lab 3.0 + Newton, same ROS 2 topic surface as the real cart;
   sim/real switch at the driver layer only. [07](docs/decisions/07_simulation.md)
8. **Single-PC RT is proven.** Production workstation (Core Ultra 9 285K, RTX 5080,
   6.8.1-1062-realtime, `isolcpus=2,4,6,7`): **19 µs** worst case under CPU + GPU load —
   quote this; the laptop's 73 µs is the cross-machine replication. RT path has no GPU
   dependency. Re-run `deploy/bench/rt_latency.sh` after any kernel/driver/BIOS change.
   [08](docs/decisions/08_single_pc_rt.md)
9. **Haply Inverse3** via the closed-binary Inverse Service (open WebSocket/JSON API),  cursor
 polled at 500 Hz; the VerseGrip stream is far slower and is stamped and reported separately.
 Hub 1.5 firmware is adopted before the first Haply benchmark. One configurable mapping file for VerseGrip buttons + 3-pedal footswitch
   (clutch, camera, instrument switch, energy, record marks).
   [09](docs/decisions/09_haply_inverse3.md)
10. **Module-first development.** Every hardware module gets its own compose profile and must
    pass the standard benchmark (bench launch + offline analyzer; rate mean ± std from MCAP
    stamps, CPU, RSS, I/O, drops; committed under `docs/benchmarks/` with the image digest),
    then climbs the integration ladder against an up-front resource budget.
    [10](docs/decisions/10_module_first_development.md)
11. **No MoveIt** in `arts_ros2_ws` or any middle overlay (immutable model, no RCM, no live
    tool-tip offset). `ur_moveit_config` is excluded in the `ur` overlay; MoveIt, if ever
    wanted, goes in an application overlay. [11](docs/decisions/11_no_moveit.md)
12. **Frames:** one tree, root `world` = the cart datum, physically realized by a calibration
    board on the pedestal. CAD-nominal (`arts_description/config/mount_transforms.yaml`) and
    measured corrections (`arts_calibration/config/`) are separate files. Record raw sensor-
    frame data plus `/tf` + `/tf_static`, never transformed data as the only copy. Cannula/RCM
    frames hang off the trainer box. [12](docs/decisions/12_frames_tf_calibration.md)
13. **Per-arm description for control, combined only for viewing.** `tool_arm.urdf.xacro`,
    `camera_arm.urdf.xacro` (rooted at their own mount frames, with ros2_control), and
    `arts_cart.urdf.xacro` (no ros2_control). Instruments swap by one launch argument.
    Exactly one node publishes the cart-level edges.
    [13](docs/decisions/13_per_arm_description.md)
14. **Stereo sync is an ARTS-built ESP32 sync box** (`arts_embedded/arts_camera_sync`) that
    fires both cameras and captures each `ExposureActive` strobe on one 80 MHz MCPWM counter,
    so L/R skew is a HARDWARE measurement. Opto-isolated Line 0 via 560 Ω; strobe pull-up
    1 kΩ; strobe is active-HIGH. The deliverable is a PCB (KiCad 9, self-contained revisions,
    fab exports committed only for a fabricated revision); no committed skew baseline may
    come from the breadboard. ESP-IDF pinned at v5.5.5. Each `arts_embedded` function gets its
    own board. [14](docs/decisions/14_camera_sync_box.md)
15. **Image-to-robot alignment for VLA data:** a constant offset is survivable, a variable
    one is fatal — the goal is a known, small, stationary offset with quantified jitter.
    Images join to pulses by `camera_time` (the driver does not publish `FrameID`); the
    `ros_time = a·ticks + b` fit is done offline in `arts_datasets`. Verified by a flash test
    and a cross-correlation test. [15](docs/decisions/15_image_robot_alignment.md)
16. **Funder demo ≠ ARTS reference run.** The 2026-10-05 funder demo ran on the legacy
    Humble stack and contributes NO data to the paper. Every paper number comes from the ARTS
    reference run on 24.04 + Jazzy. Collaborators targeting Humble are correct for their
    purpose; migration is ARTS's job at intake.
    [16](docs/decisions/16_funder_demo_vs_reference_run.md)
17. **One position-commanded control hierarchy.** Reference source (teleop follower |
    policy follower | goal mover) → task layer (resolved-rate `V_ff + K·e` with the RCM as a
    HARD QP constraint, integrated) → Ruckig + limits → per-arm joint command (UR `speedj`
    by default since the 2026-10-30 A/B, `servoj` opt-in). Teleop and
    autonomy differ only in the source; the recorded action is the absolute tip-pose target.
    Collaborator and MOPS work enters as re-derived ideas only.
    [17](docs/decisions/17_control_hierarchy.md)
18. **ROS 2 interface contract.**
    - **Role namespaces only:** `/psm_left`, `/psm_right` (UR5e + instrument), `/ecm`
      (UR5 + stereo), `/mtm_left`, `/mtm_right` (Haply), `/console`, plus the one shared
      device `/esu`. `/tf` and `/tf_static` are global.
    - **Names:** role-prefixed frames and joints with underscores (`psm_left_tool0`).
      Suffix rule: `_target` is an input, `_command` is what ARTS sent, `_state(s)` is
      measured.
    - **Moving an arm:** one input per arm, `<arm>/tool_tip_target` (pose + optional
      velocity + optional jaw).
    - **Instrument data:** `joint_states` carries the UR joints plus the instrument's
      kinematic joints; the raw discs go in `tool_actuator_states`.
    - **Energy:** routed per arm by `/esu` from each arm's `tool_info`.
    - **Controllers:** one controller manager per arm.

    Amends 1, 6, 12 and 13; topic reference in `docs/modules/ros_interfaces.md`. Code
    migrated 2026-11-01 and verified on mock hardware; not yet on a real arm. Still open:
    the ESU-TAP and the instrument drive. [18](docs/decisions/18_ros_interface_contract.md)

## Repo layout (two trees — do not confuse them)

Full narrative, including the code-intake and OEM-install-location reasoning:
[`docs/repo_layout.md`](docs/repo_layout.md).

**Tree 1 — the git repo (`~/arts`, text only):**

```
arts/
├─ oem_drivers/              manufacturer SDKs for PHYSICAL hardware — recipes + VERSION only
├─ open_source_software/     non-ROS libraries built from source (coal, pinocchio, proxsuite…)
├─ middle_overlay_ros2_wss/  third-party ROS 2 workspaces — .repos manifests + build.sh only
├─ arts_ros2_ws/src/         THE top overlay: arts_interfaces, arts_description,
│                            arts_calibration/, arts_driver, arts_controllers,
│                            arts_embedded/{arts_camera_sync, arts_esu, arts_dv_tool_driver},
│                            arts_utilities/{arts_kinematics, arts_safety, arts_teleop,
│                            arts_record, arts_datasets, arts_display}, arts_sim, arts_slicer
├─ example_app_ros2_ws/      one example application overlay
├─ hardware/                first-party mechanical designs: tool_adapter/, laparoscope/
                             (STEP, printed STLs, drawings, READMEs with bought parts)
├─ deploy/                   modules/<module>.sh, docker/, host_setup.sh, prereqs.sh,
│                            bench/, ci/
├─ docs/                     decisions/, modules/, benchmarks/, status_detail/, bom/, …
├─ reference/                gitignored machine-local symlinks (COLCON_IGNORE mandatory)
└─ arts_install.sh           thin orchestrator over deploy/modules/*.sh
```

Every non-package directory carries `COLCON_IGNORE` (which is not `.gitignore` — those
directories are fully tracked).

**Tree 2 — machine-local (`$ARTS_LOCAL`, default `~/arts_local`, ~140 GB, never in git):**
mirrored OEM archives, built open-source libraries, middle-overlay `src/build/install/log`,
`description_catalog/` (CAD/mesh staging), and `code_intake/`.

**`code_intake/<source>/` is where collaborator and earlier-project code lands.** It is READ,
never built or sourced into the ARTS chain. Layout: `0_incoming/` (as received — a full git
clone, never edited) + a REQUIRED `NOTES.md` (origin, commit hash, target distro/hardware,
license and ownership, and a verdict PER PART: PORT / REWRITE / REFERENCE-ONLY / ADOPT /
DISCARD). Intake is evaluation, not transcription: (1) is the logic right, (2) is there a
better fit for ARTS, (3) only then migrate. Expect Humble; migration is ARTS's work. Settle
ownership, anonymity and hardware match before any file is committed. Nothing leaves
`0_incoming/` by copy. If a measured comparison ever requires running a collaborator stack,
run it unchanged in a container of ITS OWN distro, never in the ARTS chain.

**Build order** (each step in a fresh `env -i` shell; one shared sequence,
`overlay_build_generic` in `deploy/modules/_common.sh`):

```
/opt/ros/jazzy → ur_ros2_ws → flir_ros2_ws → oem_ros2_ws   (default, the paper's module set)
              [→ slicer / atracsys / isaac via ARTS_ENABLE_<NAME>=1, unverified on Jazzy]
 → coal → pinocchio → proxsuite
 → arts_ros2_ws  (sources the middle overlays only)
 → example_app_ros2_ws  (sources arts_ros2_ws only)
```

### Sourcing rules (critical — do NOT violate)

Overlay `install/setup.bash` files chain to their underlay automatically. Source ONLY the
immediate underlay; never the full chain.

```
BUILD a middle overlay:   source /opt/ros/jazzy/setup.bash          ← the ONLY source
BUILD arts_ros2_ws:       source $ARTS_LOCAL/middle_overlay_ros2_wss/<each>/install/setup.bash
                          (each chains to Jazzy — do NOT also source Jazzy)
BUILD an app overlay:     source ~/arts/arts_ros2_ws/install/setup.bash
RUN arts packages:        source ~/arts/arts_ros2_ws/install/setup.bash   ← the ONLY source
RUN an app overlay:       source ~/arts/example_app_ros2_ws/install/setup.bash
```

Every BUILD runs in a fresh `env -i` shell, even a one-package rebuild: a shell that has
sourced a workspace's own `install/` (the RUN rule) must never build that workspace.

Corollary: any apt package that installs a `<lib>Config.cmake` into `/opt/ros/jazzy` wins
`find_package()` over an ARTS-built copy unless ARTS's prefix is prepended — silently. That
is why `arts_controllers` asserts the Ruckig, pinocchio and coal versions at configure time.

### `arts_driver` bringup contract

```
ros2 launch arts_driver arts_bringup.launch.py \
  psm_left:=true  psm_left_tool:=large_needle_driver \
  psm_right:=true psm_right_tool:=monopolar_curved_scissors \
  ecm:=true ecm_stereo:=true ecm_stereo_sync:=true \
  mtm_left:=true mtm_right:=true footswitch_teleop:=true head_sensor:=true gamepad:=true \
  use_fake_hardware:=false rviz:=true
```

Module solo = one flag; contention pair = two; full cart = all (decision 18 names; each arm
in its own namespace and controller manager). A switch is named after its topic path
(`ecm_stereo_left` → `/ecm/stereo_left`). `esu` is refused until the ESU-TAP port lands.
`ros2 run arts_gui arts_gui.py` starts the same switches one module at a time.
Carts are deployment profiles, not code boundaries. Procedure-specific autonomy belongs in application overlays.

## Documentation contract (each layer has ONE job)

| Layer | File | Its only job | Must NOT contain |
|---|---|---|---|
| 1. Rules + status | `AGENTS.md` | Hard rules, decision summaries, and WHERE work stands. **Authoritative for status** — checkboxes live here only, each with a one-line headline. | Reasoning longer than a paragraph; bench procedure |
| 1b. Decision record | `docs/decisions/NN_*.md` | WHY each locked decision is what it is, with evidence and amendments. | Status |
| 2. Module doc | `docs/modules/<module>.md` | WHAT the module is: artifacts, stages, pins, per-stage results, what remains. Mirrors AGENTS.md status. | Independent status claims; wiring steps |
| 3. Bench doc | `docs/modules/<substep>.md` | DO THIS AT THE BENCH, in time order. | Status; architectural reasoning |
| 4. Measured numbers | `docs/benchmarks/<name>_<date>.md` | One measurement session + the exact hardware, digest and pins. Immutable once committed. | Interpretation |

`docs/status_detail/` is an ARCHIVE of the dated narrative that used to live under the
checkboxes here (moved 2026-10-26). It is not authoritative for status, and de-duplicating it
into the module docs is an open item.

Every doc opens with a role header naming its layer. A result is written in exactly two
places — the benchmark report in full, and a one-line headline beside its checkbox — and the
checkbox is ticked in the SAME edit that records the result (else it stays `[~]` with the
reason). The FLIR module is the worked example: AGENTS.md Stage 5 →
`docs/modules/pilot_flir_camera.md` → `docs/modules/camera_sync_wiring.md` →
`docs/benchmarks/camera_sync_*` and `flir_*`.

## Dependency pinning (non-negotiable)

| Class | Pinned in | Example |
|---|---|---|
| Third-party ROS packages | `middle_overlay_ros2_wss/<ws>/<ws>.repos` | `Universal_Robots_ROS2_Driver` @ 3.10.0 |
| Open-source non-ROS libs | `open_source_software/<lib>/VERSION` | coal v3.0.4, pinocchio v3.9.0 |
| OEM SDKs | `oem_drivers/<sdk>/VERSION` | spinnaker 4.2.0.88, ESP-IDF v5.5.5 |
| apt packages | `deploy/modules/<module>.sh` | `=version` (except rolling ROS apt — see below) |
| Python packages | `requirements.txt` next to the consumer | `==` |

1. Never a branch — tag or, preferably, full commit hash. Override transitive
   `FetchContent(... GIT_TAG main)` explicitly.
2. Non-ROS C++ libraries are pinned too. Ruckig is the exception: apt `ros-jazzy-ruckig`
   0.9.2, asserted at configure time — never 0.14.0, never 0.17.3+.
3. apt gets `=version`, EXCEPT packages from the rolling packages.ros.org repo, whose exact
   versions stop being installable; prefer a git tag for anything whose numbers we publish.
4. Python gets `==`.
5. Anything unpinnable goes to `oem_drivers/` with its archive mirrored under `$ARTS_LOCAL`.

Bumping a pin is a benchmarked act: change it, rebuild, re-run the module benchmark, commit
the new baseline beside the old. When activating a module, audit what its apt dependencies
install into `/opt/ros/jazzy`. Incidents and rationale (Ruckig shadowing, coal/pinocchio
build flags, MoveIt fallout): [`docs/dependency_pinning.md`](docs/dependency_pinning.md).

## Evaluation metrics (publication target: IEEE ISMR, deadline 2026-11-10)

Measurement methods must be stack-independent where cross-platform comparison (esp. dVRK)
matters.

| Category | Metric (method) | Target |
|---|---|---|
| Latency | Teleop motion-to-motion (external 120 fps phone camera, tripod, films ArUco markers on the Inverse3 handle and the arm in one frame; main number: lag maximizing the cross-correlation of the two marker tracks during 1–2 Hz back-and-forth motion; check: onset-frame difference at matched thresholds; software path from the bag alongside) | ≤ 100 ms |
| Latency | Glass-to-glass video (closed loop: a Gray-code clock and the display output side by side under the scope; each frame reads both) | ≤ 100 ms |
| Control | Servo loop jitter mean±std + deadline misses (controller_manager diagnostics, 10 min (300 000 cycles at 500 Hz)) | 0 misses; jitter < 10% of period |
| Control | RCM deviation, two definitions: (a) kinematic — FK shaft-line-to-RCM-point distance, same definition as the open laparoscopic platform; (b) physical — vs calibrated cannula incl. calibration error; both ≥ 30 min teleop with drift slope | (a) ≤ 0.1 mm median; (b) ≤ 1 mm RMS, ≤ 2 mm peak, no drift |
| Control | Master–slave tracking RMS error (scaled master path vs measured tool path) | report; goal ≤ 1 mm RMS |
| Control | Trajectory smoothness: SPARC + LDLJ offline from recorded teleop trajectories (downsampled to 5 Hz), vs JIGSAWS experts + open-platform published values; pure post-processing of the ARTS reference run | within expert JIGSAWS range |
| Accuracy | Vision-in-loop trajectory following (MOPS protocol: stereo-tracked marker on instrument, taught trajectory, mean end-to-end pose error) | ≤ 1.2 mm / 0.3° (MOPS's better arm) |
| Accuracy | External ground truth: Atracsys fusionTrack measures physical tool-tip accuracy + RCM deviation (SDK vendored in v1.4 reference snapshot) | report; no SOTA platform publishes this |
| Data | Per-topic recording rate mean±std from MCAP stamps, 30 min full load; nominal rates set at ~70% of module ceiling | ≥ 99% of nominal; zero dropped robot samples |
| Data | Sync: L/R skew measured TWO ways — (a) hardware, ESP32 MCPWM capture of both `ExposureActive` strobes on one 80 MHz counter (12.5 ns resolution, decision 14); (b) software cross-check, per-pulse spread of `camera_time` differences from the `~/<cam>/meta` topics. NOT from `header.stamp` — the synchronized driver assigns identical stamps by construction (decision 5 amendment). PTP offset logs apply to GigE devices only | skew < 1 ms (expect < 100 µs); PTP < 1 µs |
| Data | Image-to-robot alignment (decision 15): lag maximizing cross-correlation between a vision-tracked tool-tip fiducial and FK tip position from the recorded joint states, in a real bag; report the lag AND its spread across the recording. Flash test (ESP32-fired LED on a known pulse index) as the assumption-free check on the `FrameID`↔`pulse_index` join | jitter is what matters, not bias: report lag; spread ≤ 1 frame period |
| Safety | Scripted near-collision scenarios, 3 arms over trainer box | 0 contacts; min distance ≥ QP margin |
| Human | NASA-TLX weighted workload, FLS peg transfer, N ≥ 5 operators | ≤ 50 |
| Human | FLS peg-transfer time/error vs published dVRK / da Vinci baselines | within dVRK range |
| Surgical | Ex vivo porcine: monopolar cut & coag, bipolar coag, vessel seal (+burst pressure), stapler (+leak check) | all demonstrated w/ per-task criteria |
| System | 2 h uninterrupted teleop; outside-user fresh install from README | no restarts; ≤ 1 day; BOM < $150k |

**SOTA benchmark anchors** (numbers to quote and beat/match in the paper):

| Platform | Reported | ARTS comparison |
|---|---|---|
| dVRK-Si (JHU/Vanderbilt, ISMR 2025) | PID for all arms at 1.5 kHz, 0.01 ms std; current-loop step response < 1 ms; no task/latency metrics | Report loop rate + jitter with identical statistics; state the caveat that UR joint servos are internal — ARTS measures the command-path loop |
| MOPS (SDU, ISMR 2021) | Vision-in-loop trajectory error 1.2 mm / 0.3° (UR5e), 2.7 mm / 0.7° (UR5); needle pick-up 28/30; no latency or RCM numbers | Replicate their vision-in-loop protocol; MOPS is ARTS's closest ancestor (UR arms + EndoWrist adapters + software RCM) |
| Open laparoscopic platform (under review 2026) | Kinematic RCM deviation median 0.035–0.079 mm; SPARC/LDLJ ≈ JIGSAWS experts; 85% policy success | Same-definition kinematic RCM deviation + SPARC/LDLJ from the ARTS reference run |

None of the three report motion-to-motion teleop latency or externally measured accuracy —
the latency rig and Atracsys ground-truth sessions are ARTS differentiators (and answer the
reviewer question "your RCM number is kinematic — what is it physically?").

## Development status — update these checkboxes as work completes

A fresh agent takes over at the first unchecked item, respecting phase order. Each phase must
leave a working system. Dated narrative behind each item: `docs/status_detail/`.

**ISMR schedule:** deadline **2026-11-10**; experiment freeze **2026-11-05 (Thu) night** (moved from 11-04 on 11-03: the instrument module is still with its designer). The 20-day plan,
the experiment list (E1–E8) and the revised paper claim live in the manuscript repo's
`publication_plan.md`.

### Planning
- [x] Prior system (v1.4) codebase inventory and architecture review
- [x] Architecture decisions locked
- [x] Evaluation metric suite defined

### Phase 0 — Scaffolding ([detail](docs/status_detail/phase0_scaffolding.md))
- [x] Private `arts` git repo (remote URL deliberately not recorded in any tracked file)
- [x] Host prep, Ubuntu 22.04 (superseded; kept as the record behind the 190 µs result)
- [x] Host prep, Ubuntu 24.04 laptop: Jazzy, `nvidia-driver-595-open` via DKMS with the RT
      bypass, 6.8.1-1057-realtime, `isolcpus=4,5,10,11`
- [~] Host prep, production workstation: full install `--path both` in 7 min 46 s to
      `ALL DONE` (NOT a bare-OS figure — host packages, image pull and Spinnaker mirror were
      warm); four runs, each failing on a recipe defect now fixed; `prereqs.sh` reworked
      2026-10-26 (durable DKMS bypass in `/etc/dkms/nvidia.conf`, verified by an override-only
      rebuild; fails loudly). Open: a reinstall from a bare OS to make the install time a
      paper number. Control-library apt deps (Eigen, Boost, assimp, qhull,
      OpenCV) pinned in `deploy/modules/arts.sh` since 2026-11-01.
- [x] RT + GPU compatibility: 190 µs (22.04), 73 µs (24.04 laptop), 19 µs (workstation)
- [x] Base dev image (Jazzy + CycloneDDS + MCAP + ros2_control); `deploy/modules/dds.sh` makes
      bare metal use CycloneDDS too. Open: FLIR baseline not yet re-run bare metal under
      CycloneDDS
- [x] Benchmark harness template (`arts_record/bench/`)
- [x] Both trees created, COLCON_IGNORE markers, package stubs
- [x] `.repos` manifests and VERSION files pinned
- [x] `arts_install.sh` (bare / docker / both; idempotent)
- [x] `arts_interfaces` initial set
- [x] Jazzy migration
- [~] `arts_description`: arm mounts from Vention's assembly (`mount_transforms.yaml`),
      corrected 2026-11-02 against the real arms (left/right swapped, every base turned 180°:
      the CAD base is UR's `base`, not `base_link`); pedestal, instrument adapter and stereo
      laparoscope meshed from their CAD (`scripts/import_pedestal_step.py`,
      `import_tool_step.py`) and shown in RViz by `cart_tf`, all three checked by the PI over
      the read-only bringup; the CAD puts the instrument shaft 88.388 mm and the endoscope
      42.13 mm from the flange. Per-arm control descriptions, collision capsules, instrument
      primitives and trainer box still open
- [x] `arts_driver` bringup skeleton with per-component toggles
- [~] CI: `deploy/ci/check_anonymity.sh` exists; full-chain colcon build, `ament_lint`, the
      servo smoke test and a Dockerfile-vs-dispatch no-drift check still to come
- [x] `LICENSE` (Apache-2.0, holder unfilled by design) and `CONTRIBUTING.md`
- [x] Structural review triaged (`docs/repo_structure_triage.md`); one overlay-build mechanism
      DONE 2026-09-08; extracting narrative out of this file DONE 2026-10-26 (moved to
      `docs/decisions/` and `docs/status_detail/`)

### Collaborator code intake (started 2026-10-26)
- [x] Collaborator A — velocity-based control stack used on the real arms for the funder demo
      (Humble). REVIEWED 2026-10-26, verdicts in `$ARTS_LOCAL/code_intake/ctrl_velocity/NOTES.md`.
      Law: extended-Jacobian resolved-rate RCM control, `q̇ = Jᵀ(JJᵀ+reg)⁻¹(V_ff + K·e)`,
      streamed as `speedj` at the RTDE rate, no ros2_control, RT scheduling commented out.
      Its URScript caps arm `speedj` acceleration at 1 rad/s² (upstream 40), never re-tested
      after the real vibration cause was fixed — a variable in the three-way test. A's answers
      (2026-10-26): the "sluggish" feel is mostly the grasp path, limited by the VerseGrip
      data rate (~50→~200 Hz, A's observation) and a slow Hall-sensor grip input, so no arm
      control law fixes it. Instrument disc motors are driven by `dynamixel_sdk` directly from
      the PC, which is a faster Phase 4 route than the planned ESP32 tool-drive board. A owns
      the code except its Lie-group math, vendored from a research-only-licensed library;
      pinocchio replaces it.
- [x] Collaborator B — MOPS-derived teleop stack with collision guard (Jazzy, different
      site's 3-arm cell). REVIEWED 2026-10-27, verdicts in
      `$ARTS_LOCAL/code_intake/ctrl_position/NOTES.md`. Upstream MOPS re-checked at source
      2026-10-27: no LICENSE file, only a bare "BSD" tag with no notice; B's code derives
      from it. Policy: nothing from MOPS or B enters the tree or the capsule; ARTS
      re-derives from published methods and cites the MOPS paper. KEY FINDING: since 2026-09-25 B's default IK IS A's
      velocity DLS law, integrated to positions and sent at 125 Hz to
      `forward_position_controller`, with three months of guards on top (branch lock,
      damping ramps, rate clamps, capsule collision filter, contact monitor). B's wrist
      inaccuracy most likely comes from orientation being a 0.005-weight soft objective
      plus registration errors (hypotheses, unverified).
- [x] ESU microcontroller firmware repo (Humble), re-reviewed 2026-10-26 at the updated
      upstream (now Apache-2.0, three revisions, 500 ms command expiry, standard
      `std_msgs`/`std_srvs` only). Verdict changed from REWRITE to ADOPT AS AN EXTERNAL PINNED
      COMPONENT: flash upstream firmware at a pinned commit, bridge through ARTS's Jazzy
      micro-ROS agent, and write one thin adapter onto `arts_interfaces`.
- [x] Control-law decision — DECIDED 2026-10-28 by argument, not by the planned three-way
      measurement (A and B share one law; deadline): decision 17. A's law, integrated, inside
      ARTS's chainable controller, RCM as a hard constraint. Branch / gimbal-lock regression
      test (`arts_controllers/test/branch_singularity_test.py`) PASSES on mock 2026-10-29:
      through wrist_2 = 0 (0.01°) with wrist_1/wrist_3 still, round trip back to the start
      joints, no elbow flip in a 7-move sweep; its gimbal checks seen failing on a mutated
      path, the round-trip and rate checks seen passing only. Until the freeze, E5/E6/E8
      use the existing `CartesianServoController` in free space without the RCM.

### Module pilot — FLIR camera ([detail](docs/status_detail/flir_pilot.md), module doc `docs/modules/pilot_flir_camera.md`)
- [x] Stage 0: host readiness — 2× BFS-U3-63S4C (L 24057131, R 24057132; the 2026-10-27 swap was wrong and was reverted 2026-10-28 by disparity sign; view rotated 180° in display only), udev, usbfs 1000 MB
- [x] Stage 1: Spinnaker 4.2.0.88 mirrored + installed; `Dockerfile.flir`
- [x] Stage 2: `flir_ros2_ws` + camera bringup, 40.00 Hz / 0 drops
- [x] Stage 3: 10-min baseline 39.99 Hz ± 0.00, 0 drops (`docs/benchmarks/flir_bfs-u3-63s4c_20260806.*`)
- [x] Stage 4: `deploy/modules/flir.sh` shared by both paths; rebuild reproduces the baseline
- [x] Stage 4b: Jazzy re-validation, 39.99 Hz, 0 drops (`flir_bfs-u3-63s4c_20260810.*`)
- [ ] Stage 5 — hardware trigger + skew:
  - [x] `arts_camera_sync` firmware written, toolchain installed, compiles clean
  - [~] Breadboard rig, rungs 5a–5e passed (5a signal; 5b 80 MHz capture clock measured,
        trigger jitter < 1 tick; 5c camera slaved at 30.00 Hz; 5d trigger→exposure 711.7 µs
        = first-row assertion, 1 kΩ pull-up, both strobe edges captured; 5e skew mean 3.4 µs
        / max 10.6 µs over 720 pulses)
  - [x] `arts_driver/config/blackfly_s_arts.yaml` + triggered `cameras.launch.py`
  - [x] micro-ROS firmware publishing the sync pulse (best-effort QoS); renamed
        `/ecm/stereo_sync/pulse` by decision 18 (built 2026-11-01, not yet flashed)
  - [ ] Original note on the ARTS-side definition file (kept for its ordering reasoning)
  - [x] Both cameras in ONE `ComposableNodeContainer`; dropouts did not reproduce in either
        layout, so `composed:=false` stays for a future A/B (`flir_composed_ab_20260821.md`)
  - [x] ESU-TAP PCB source recovered as a PATTERN reference (not adopted)
  - [~] Sync-box PCB `arts-camera-sync-v2`: design package written, layout approved,
        **boards received 2026-10-20**; hand-fitted; Stages A and B PASSED 2026-10-27; Stage C camera 24057132 on J1 PASSED (30.00 Hz, delay 708.17 µs, width 4977.20 µs, after forcing `IspEnable` off)
        (trigger 559.3/560.0/560.7 Ω, pull-ups 991/990/994 Ω,
        `camera_sync_v2pcb_stageA_20261027.md`). Stage C PASSED for both cameras
        (`camera_sync_v2pcb_stageC_20261027.md`; verified from a power cycle). Stage D
        PASSED 2026-10-28 after a firmware fix (the driver's `cap_edge` mislabelled ~0.1 % of
        start edges as falling; edges are now classified by pin level: 0 missed strobes in
        8991). Still open: `production/` exports not committed
  - [x] `~/<cam>/meta` topics recorded (composing renames no topic)
  - [x] Measure skew both ways ON THE PCB (hardware + `camera_time`) and commit the baseline
        — paper experiment E3. DONE 2026-10-28: L − R **+2.793 ± 3.209 µs** (24057131 − 24057132), worst 10.825 µs,
        n = 8991, 0 missed strobes; `camera_time` agrees per pulse (r = 0.990, residual
        0.447 µs std) (`docs/benchmarks/camera_sync_skew_v2pcb_fix_20261028.md`)
- [ ] Camera one-row readout shift: ~1 % of TRIGGERED frames on both cameras come out shifted one
      row with colour gains misapplied (0 of 860 free-running); not the overlap mode or exposure
      length, and it persists under 2×2 binning; cause open. Affects image data, not timing
      (`docs/modules/pilot_flir_camera.md`)
- [ ] Workstation: reproduce the 39.99 Hz FLIR baseline bare metal at Spinnaker 4.2.0.88
      BEFORE any SDK bump (4.4.0.99 is a separate benchmarked change)
- [x] Both cameras, 1536×1024 binned raw Bayer, 30 Hz, zero drops, 30 min on the workstation with the
      USB controller topology recorded — paper experiment E4. DONE 2026-10-28: 54 827 frames
      per camera for 54 827 pulses, 0 gaps, display running, written to the SYSTEM disk (no
      dedicated image NVMe) (`docs/benchmarks/e4_dual_camera_recording_20261028.md`)
- [~] Exit review: methodology verdict recorded in the module doc; corrected template to be
      applied when `deploy/modules/ur.sh` is written

### Phase 1 — Control core (single arm) ([detail](docs/status_detail/phase1_control_core.md))
- [x] `arts_controllers/CartesianServoController` (DLS IK in pinocchio → Ruckig in joint
      space → Cartesian speed governor), passing on mock hardware; tool-tip offset in the
      model via `~/set_tool_tip_offset` (metres)
- [x] 500 Hz bringup in `arts_driver` (now `ur_arm.launch.py arm:=psm_left|psm_right|ecm`), verified by
      `arts_controllers/test/run_servo_smoke_test.sh` (run via the wrapper)
- [ ] Exit: the mock servo regression stays green; on a REAL UR5e, command-path loop rate,
      jitter and deadline misses (paper experiment E5). No parity test against v1.4.
      `deploy/modules/ur.sh` written 2026-10-29 (arms in `arts_driver/config/ur_arms.yaml`);
      host setup done; preflight passes on all three arms (and fails with the switch unplugged).
      Driver bumped to 3.10.0; factory calibration of all three arms extracted 2026-10-29
      (`arts_calibration/config/`), the UR5e launch loads the arm's own file (mismatch error
      gone; a wrong-arm file raises it). RT active: control loop SCHED_FIFO 50, RTDE thread 99 (after a reboot).
      FIRST MOTION on the right UR5e 2026-10-29: 30 mm out-and-back on x/y/z at 5 mm/s,
      residuals ≤ 0.03 mm, 0 overruns after start-up (`docs/modules/ur_arms.md`). E5 done on
      the right UR5e (see Evaluation); open for the exit: the mock regression in CI

### Phase 2 — Three arms + RCM + collision
- [ ] Two rate-grouped controller_managers (500 Hz + 125 Hz); cross-manager joint-state
      mirroring via RT-safe buffers — never a single mixed-rate manager
- [~] RCM as QP constraint (pinocchio + proxsuite); cannula calibration procedure.
      Task QP → Ruckig → exact projection onto the RCM rows, switched live by
      `<arm>/set_rcm` (refused unless the shaft is within 2 mm), off at activation; mock
      2026-11-01: deviation median 0.0013 mm on a 10 mm Lissajous, tracking 0.47 mm RMS
      (decision 17 amendment). Plain `set_rcm.py on` relocates the point onto the current
      shaft at the last depth; tip position only under the RCM by default (the adapter's
      shaft is perpendicular to wrist_3); da Vinci X tip preset from A's model
      (`arts_driver/config/tool_tips.yaml`, A/B offsets disagree, flagged). Not yet on a real
      arm until 2026-11-01: on the left UR5e by gamepad, 0.0135 mm median (S1 line);
      cannula calibration open. 2026-11-07: RCM teleop on all three arms (2 tools + ECM),
      deviation median 0.016 / 0.017 / 0.043 mm; the ECM rang at ~9.5 Hz under the RCM on
      `speedj` and runs `servoj` since (`ur_arms.yaml command_interface`); `set_rcm` with the
      wrist now places the point on the shaft (decision 17 amendment 2026-11-07)
- [~] Collision guard (`arts_safety collision_guard`, 2026-11-07/08): one capsule per link fitted
      to the vendor collision-mesh SURFACE (end spheres on the joint housings) + CAD capsules for
      adapters, first 150 mm of shafts, ECM camera head and scope; pedestal = three 22 cm
      spheres between the mounts (PI); between arms and within each arm, 100 Hz from tf;
      < 20 mm holds every arm (`/arts/collision_hold`), followers disengage, release into a
      separate-only recovery. Mock test passes; real ECM self-collision roll stopped before
      contact; capsules and spheres checked by the PI in RViz. Not yet: the cross-arm trip on
      hardware; QP velocity dampers (the original plan)
- [ ] Exit: teleop-speed motion, RCM error < 1 mm, no QP deadline misses at 500 Hz

### Phase 3 — Vision + recording pipeline ([detail](docs/status_detail/phase3_vision_recording.md))
- [ ] FLIR pair integration into the recorder (trigger from the sync box; device clock via
      `~/<cam>/meta`)
- [ ] `arts_record` dual-bag MCAP profiles: `/tf`, `/tf_static`, raw sensor-frame data,
      `/ecm/stereo_sync/pulse` in the state bag, meta topics beside the images
- [ ] `ExposureActive` mode recorded as a measured fact (measured at rung 5d: first-row);
      `trigger_delay` set explicitly and identically on both cameras
- [ ] `arts_datasets`: MCAP → LeRobot export with the offline clock fit
- [ ] Alignment verification: flash test + cross-correlation test
- [ ] Exit: skew within trigger jitter; image-to-robot lag jitter ≤ one frame period;
      dataset loads in an SRT-H-style trainer

### Phase 4 — Teleoperation + instruments + energy ([detail](docs/status_detail/phase4_teleop_instruments_energy.md))
- [~] Haply Inverse3 bridge: ARTS's own `arts_driver/haply_bridge` (the third-party bridge
      resamples on a timer), Hub 1.5.0 service 3.5.7; first check 2026-10-30: both cursors
      500.0 Hz, VerseGrips 44–50 Hz of new data. Inverse3 firmware 7.6 (kept until after
      submission), VerseGrip v3.1 = current per Haply (the flash "failure" is a Hub UI bug).
      Gravity compensation 0.045 kg on both (A's left 0.025 dropped from the top) and home
      return off, set by the bridge each connection, read back; released handles hold at top,
      middle and bottom within 0.8 mm (2026-11-03, decision 9). No benchmark yet
- [~] `arts_teleop`: clutch, 3:1–5:1 scaling, camera-frame mapping. `teleop_follower.py`
      (position only, deadman or surgical clutch, re-index per engagement, box, drop and
      stale guards) passes 11 checks on mock 2026-10-30 (`test/teleop_follower_mock_test.py`;
      its direction checks seen failing under a wrong axis map). First real session on the
      left UR5e 2026-10-30 (directions "roughly correct"); handle-to-measured-joints lag
      111 ms → 84 ms with decision 17's rate law + Haply-velocity feedforward (cap 250 mm/s,
      20 rad/s² / 500 rad/s³, fast shakes followed in full) → 58 ms with `speedj`, now the
      default (`servoj_speedj_ab_20261030.md`; earlier 140 / 110 ms figures were a `/tf`
      timing artefact, decision 17 amendment). Open: refined axis map, camera-frame mapping
- [~] Input mapping layer (one file: VerseGrip + 3-pedal footswitch): Savant Elite2 node +
      `arts_teleop/config/input_mapping.yaml`; only `record_toggle` acts so far. 2026-11-01:
      `/console/footswitch_teleop` (renamed), GOOVIS `/console/head_sensor` (udev rule
      pending), Xbox `gamepad_jog` source (A's layout + deadman; 7 mock checks pass). Open:
      da Vinci clutch (head in + 10° handle alignment, A's scheme) once orientation teleop exists
- [~] `arts_gui` operator GUI (Qt): modules started one by one, presets, displays, RViz from
      the running arms; start/stop verified on mock, not yet used at the console. 2026-11-07:
      instrument per PSM (tip preset + wrist + disc file), one collision guard for all arms
- [~] `arts_dv_tool_driver`: 2026-11-01 `disc_drive` (C++, A's bus values: XC330, protocol 2,
      1 Mbps, IDs 1-5, velocity mode, PI 500/1600; refuses the EEPROM mode write unless
      allowed), dynamixel_sdk 4.0.3 pinned in oem_ros2_ws; read-only probe: all 10 servos in
      velocity mode, no errors. 2026-11-01 bench: torque on both adapters, gamepad LB jogs
      discs, bus-to-arm verified by FTDI serial, FTDI 1 ms latency rule
      (`deploy/modules/dv_tool_driver.sh`; 199 Hz with the plain sync read, superseded
      2026-11-03: Fast Sync Read, **333 Hz**; 2026-11-04 probe: 499 Hz at 1 Mbps with Return Delay Time 0, ~985 Hz at 2 Mbps); 2026-11-04 the adapter
      designer's automatic homing ported (`tool_homing`; hook, needle driver, forceps,
      scissors): hook on psm_left, 10 refits, roll within 3 ticks, physical pitch 0.52° std
      by photo (`tool_homing_hook_psm_left_20261104.md`); needle driver repeats per
      instrument but one of two sat ~5° off in yaw, so the zero = auto-home + gamepad visual
      touch-up + set_zero (PI decision; a per-type jig later), disc layout recorded (`docs/modules/dv_tool_driver.md`). Target remains: da Vinci X disc drive (up to 5 Dynamixel discs
      per arm) as ros2_control joints. Bring-up uses collaborator A's DH + coupling values, IN the tree
      with provenance and part numbers (A agreed 2026-10-29); joint sets per instrument
      (no jaw on spatula/hook; stapler blade on disc 5) (`docs/instrument_identification.md`)
      2026-11-02: A's engagement/homing ported as is (PI: the adapter designer's
      current-capped scheme, with its mode writes), one service per moving step; passes on a
      simulated adapter, but on the real adapter the automatic spin locks in falsely even with
      no instrument (A engages manually by gamepad; routine unproven). Manual engagement +
      visual zero done on psm_left (471006); instrument joints through A's coupling and
      limits (interim `tool_joint_states/target`, gamepad LB+B); coupling checked on 471006
      (psm_left) and 471205 (psm_right): each joint moves alone, PI-observed; <= 1.0 deg
      crosstalk in the driver's joints; jaw from the VerseGrip lever (`jaw_follower`, A's grasp path)
      run on psm_right, A's Hall range holds
- [x] `arts_esu` ESU-TAP interposer on Jazzy (monopolar + bipolar); v2 bypass verified
      2026-11-03 with the ESU on (10 W, no instrument): software pulses, pedals through the
      board and pedals with the board UNPOWERED all activate CUT, COAG and bipolar. Designer's
      firmware unmodified (`3.2.0-bench`, pinned; decision 18 amendment); `esu_node` on Fast
      DDS (services do not cross to CycloneDDS), 20 mock checks; energy timing from
      `/esu/state` stamps; `esu:=true` and the GUI's Energy box (enable/disable) work on the
      real board; surgeon takeover and link loss verified with the ESU on. ESU stop = its own power
      switch, not ARM HOLD (`docs/modules/esu.md`)
- [ ] GOOVIS stereo display path, independent of the recorder: headset half-width SBS
      810×1080 per eye, tower left-only 1620×1080, composed with the recorder in one
      process (`docs/modules/console_display.md`). 30 min on one monitor (E4): 0 recorder
      losses, 29.998 pairs/s, frame age 19.9 ms; record pedal pauses/resumes the recorder
      with episode marks (`footswitch_record_toggle_20261028.md`); GOOVIS + tower fullscreen
      on their own outputs, 3D, ×2 stretch and eye order operator-checked; E7 67.8 ms on
      the tower path; GOOVIS latency itself not measured
- [x] Safeguard stop (ARM HOLD), 2026-09-22: one station, monitored manual reset, Category 2
      on all three arms; stop measured by E8 on the left UR5e (below); the teleop follower
      disengages on any UR safety stop until a fresh pedal press (`docs/modules/safeguard_stop.md`)
- [ ] Exit: full teleoperated peg transfer recorded and replayed from the bag
- [ ] ARTS reference run: teleoperated partial nephrectomy on a hydrogel phantom on
      24.04 + Jazzy, recorded end to end (NOT the funder demo)

### Phase 5 — Simulation + Slicer
- [ ] Isaac Lab / Newton patient-cart twin; same topic surface
- [ ] SlicerROS2 integration (`arts_slicer`) — ask the Slicer developers which release builds
      on 24.04 and which `slicer_ros2_module` revision supports Jazzy
- [ ] Exit: the same teleop session runs against sim by switching driver profile only

### Phase 6 — Release
- [~] Dockerfiles + compose profiles: `arts:jazzy` (full chain, one image, a compose service
      per module) passes the mock servo smoke test with SCHED_FIFO and `task_qp_test` in the
      container 2026-11-01 (decision 4 amendment). 2026-11-08 rebuild at the freeze
      (`4288a7943299`, build jobs now bounded by memory after an OOM, decision 4 amendment):
      smoke test passes; three-arm mock bringup 500/500/125 Hz with the collision guard; RViz
      from the container on the host display (the X mount now uses `$XAUTHORITY`). No
      real-hardware container run; no published image; `host_setup.sh` open
- [ ] Docs: build guide, BOM (< $150k), calibration procedures
- [ ] Outside-user reproducibility pass (teleop-in-sim from README in ≤ 1 day). 2026-11-09
      first outside install: dead Spinnaker link, 4.2.0.88 not offered; fixed by
      `ARTS_ENABLE_FLIR=0` (no cameras) and `ARTS_SPINNAKER_VERSION` (explicit off-pin SDK).
      Same tester's sim-only bare-metal build then got through after: stale rolling apt
      (`ur_msgs` 2.5.0, `ur_client_library` 2.9.0, `generate_parameter_library` 0.7.1; README now
      says upgrade first; a prereq version check is open) and inner double quotes in
      `cartesian_servo_controller_parameters.yaml` descriptions, which an older generator does
      not escape (0.7.6 does; quotes now single, header regenerated and compile-checked)

### Evaluation & paper (ISMR; experiment IDs from `publication_plan.md`)
- [x] E1 install time on the workstation — 7 min 46 s, caveated (see Phase 0)
- [x] E2 RT latency on the workstation — 19 µs (`docs/benchmarks/rt_latency_ultra9-285k-8p16e_20261026.md`)
- [x] E3 stereo skew on the PCB — L − R +2.793 ± 3.209 µs, worst 10.825 µs
      (`docs/benchmarks/camera_sync_skew_v2pcb_fix_20261028.md`)
- [x] E4 dual-camera recording, 30 min — 100 % of frames, 0 gaps, skew L − R +2.687 ± 3.204 µs
      (`docs/benchmarks/e4_dual_camera_recording_20261028.md`)
- [x] E5 servo loop health, 10 min, real UR5e — 499.994 Hz, period 2000.00 ± 76.56 µs (jitter
      3.83 %), 0 missed cycles after start-up; 4 `/joint_states` messages lost by the separate
      recorder (`docs/benchmarks/e5_servo_loop_20261029.md`)
- [x] E6 teleop motion-to-motion latency — 58 / 54 ms on 120 fps video (two usable takes;
      3 s windows median 45 ms, 14–61 ms), left UR5e on `speedj`; bag software path agrees
      (58 ms) (`docs/benchmarks/e6_teleop_latency_20261030.md`)
- [x] E7 glass-to-glass latency — 67.8 ms, 60 Hz monitor, tower view windowed, closed-loop
      clock, 5 min (`docs/benchmarks/e7_glass_to_glass_20261028.md`); GOOVIS not measured
- [x] E8 ARM HOLD stop — left UR5e, from the UR's stop report: within 1 mm of rest in
      96.7 ± 1.0 ms (100 mm/s) / 101.4 ± 3.8 ms (250 mm/s), distance 7.76 / 18.41 mm, hold
      drift ≤ 0.014 mm, reset jump ≤ 0.022 mm; button-to-report delay not measured
      (`docs/benchmarks/e8_arm_hold_stop_20261101.md`); at surgical speeds 0.72 mm at 10 mm/s,
      1.46 mm at 20 mm/s (`e8_arm_hold_stop_slow_20261103.md`)
- [x] E9 full-pose teleop with the instrument wrist — left UR5e + needle driver, room frame,
      228 s engaged: roll span 202 deg, wrist pitch 95, yaw 115, jaw full range with grip;
      UR joints <= 16 deg; both cameras recorded (`docs/benchmarks/e9_full_pose_teleop_20261105.md`).
      Real-arm fixes first (yaw sign, display rotation, pen axis, rate, UR share, 60 deg bend
      limit; decision 17 amendment 2026-11-05). Tip videos to export from the image bag
- [~] Pose repeatability vs dVRK Classic (fusionTrack, ProGrasp marker, collaborator A's 5-pose
      x 30 protocol in pose mode): ARTS left UR5e 2026-11-07, 0 misses in 150 stops, ISO RP
      0.14-0.25 mm per pose, pooled l 0.078 +- 0.043 mm (RMS 0.089, max 0.21); kinematic RP
      <= 0.020 mm (`docs/benchmarks/repeatability_psm_left_20261107.md`). Open: the dVRK
      Classic run (collaborator, 2026-11-09)
- [x] RCM drift by tracker (shaft marker, pivot fitted from the motion, DASH-comparable, 3 x
      5 min, pivot <= 15 deg, roll <= 10 deg), left UR5e + ProGrasp 2026-11-07: pivot error
      0.335-0.366 +- 0.165-0.183 mm (max 1.09), drift <= 0.009 mm/min, kinematic median
      0.026 mm (`docs/benchmarks/rcm_drift_psm_left_20261107.md`); `set_rcm` with the wrist
      fixed the same day (the point was placed on the jaw axis)
- [ ] S1 (stretch) kinematic RCM deviation — `<arm>/rcm_state` publishes it (the open
      platform's definition). First real-arm datum 2026-11-01, gamepad jog, no instrument:
      median 0.0135 mm, p95 0.049, max 0.189 mm over 296 s
      (`docs/benchmarks/rcm_gamepad_psm_left_20261101.md`). Teleop S1 postponed until the
      instrument drive and the GOOVIS view make teleop meaningful (PI, 2026-11-01)
- [x] Clinical teleop (PI 2026-11-05, before the peg-transfer and tissue runs) — on the real
      left UR5e + camera arm 2026-11-05: (a) clutch pedal enabled by the GOOVIS head sensor
      (7 mock checks, both failure paths seen); (b) camera arm by BOTH MTMs on the middle
      pedal (`camera_teleop.py`): only the motion both hands share translates, steering-wheel
      roll about the optical axis (dead band 5 deg, gain 1, 15 deg/s, the image turns with the
      hands), a dVRK-style haptic bar holds the hands' distance (150 N/m, cap 2.5 N, felt by the
      PI; Inverse3 offset measured; 2026-11-06: camera motion opposite to the hands, clutch
      inside camera mode as in dVRK, mock only), the tool arms disengage; handles held in Haply
      position mode while the head is out (decision 9; a PC-side spring went unstable); pedal
      panel and two-line PSM labels on both displays; (c) re-engaging after a camera
      move through the 10 deg gate; straighten on VerseGrip button a. Default in `teleop.yaml`.
      Right tool arm up 2026-11-05 (needle driver homed on FTBEO5G1; plain disc mapping, the
      2-5 swap was a mis-seated instrument), bimanual session run. Open: GOOVIS tilt from a
      measured console model (30 deg set by hand), head-sensor removal delay unmeasured (the
      tool can drift while the operator leaves). E9 ran deadman, fixed camera, room frame
- After the benchmark freeze, before results writing (PI 2026-11-05): FLS peg transfer and
  animal-tissue runs with the clinical teleop above
- Cut to future work for this submission: NASA-TLX, reference run, Atracsys,
  MOPS vision-in-loop, SPARC/LDLJ, collision QP, LeRobot export, sim, Slicer
- [ ] Anonymized project page + code capsule; private repo never linked
- [ ] ISMR manuscript submitted (2026-11-10)

### Phase 7 — Later
- [ ] Evaluate the next LTS distro (Jazzy support runs through May 2029)

## Key files and references

- `docs/host_prerequisites.md` — the unpinnable host choices (GPU driver, PREEMPT_RT,
  `isolcpus`, account-walled SDKs) and **"Known host issues — open"**; check it before
  debugging any unexplained container, build-isolation or RT failure.
- `docs/bom/` — `bom.csv` is the single source of truth for the < $150k claim. Read its
  **"Procurement constraints"** before anyone quotes an arm: UR lists surgical use as
  unintended, and the purchase was redirected to the sanctioned channel. Never name the
  regional distributor in a tracked file.
- `docs/instrument_identification.md` — measuring da Vinci X kinematics/coupling ourselves;
  read before touching `arts_dv_tool_driver`.
- `docs/repo_structure_triage.md` — verdicts on the external structural review.
- `docs/arts_architecture_proposal.html` — DEPRECATED; do not update.
- Closest related work: the open laparoscopic platform (2× UR5e, analytical RCM),
  MOPS (Schwaner et al., ISMR 2021; UR5 + UR5e, EndoWrist adapters, software RCM — ARTS's
  closest ancestor), dVRK-Si (Xu et al., ISMR 2025). Numbers to compare against are in the
  SOTA table above.
- `reference/v1.4_ref/` (prior system snapshot: RA-L manuscript, `ros2_wss/`, vendored
  `Software/`, legacy installer) and `reference/install_readme.md` (how every module on the
  original PC was installed — cross-check it when a dependency is already present).
