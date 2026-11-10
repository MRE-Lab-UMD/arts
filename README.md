# ARTS — Autonomous Robotic Translational System

An open-source, < $150,000, three-arm robotic laparoscopic platform for surgical-autonomy
research: teleoperation, imitation learning, and supervised autonomy. It is built from
universal robot arms, standard da Vinci instruments and open-source software (ROS 2 Jazzy,
ros2_control, pinocchio, proxsuite, Ruckig), plus the unavoidable OEM drivers. It is designed so
that a lab can buy the bill of materials, run one install script (or build the Docker images with
it) and reproduce the system; no outside lab has done a fresh install yet.

ARTS has two parts:
- **The system:** a patient cart with two tool arms and a camera arm, a surgeon console, and
  one control law for teleoperation, scripted pose commands and learned policies. It holds a
  remote center of motion (RCM) in software, records everything to MCAP, and checks collisions.
- **The tool module:** UR-to-da Vinci adapters that drive da Vinci X instruments (wrist, jaw,
  roll) through Dynamixel disc drives with automatic homing, plus energy through the ESU-TAP
  board (monopolar cut and coag, bipolar; by pedal or software, logged).

Rules, locked decisions and the status checklist are in [`AGENTS.md`](AGENTS.md); the reasoning
behind each decision is in [`docs/decisions/`](docs/decisions/).

---

## The system at a glance

| Station | What it holds |
|---|---|
| **Patient cart** | Vention pedestal; 2× UR5e tool arms (`psm_left`, `psm_right`, 500 Hz) with da Vinci X instruments on the ARTS adapters; 1× UR5 CB3 camera arm (`ecm`, 125 Hz) with a 12 mm stereo endoscope on two FLIR cameras, triggered by the ARTS sync box; trainer box; main PC; ARM HOLD relays |
| **Support tower** | Monitor 4 (tower view); ESU behind the ESU-TAP board; light source; smoke evacuator; irrigation and suction pumps |
| **Surgeon console** | 2× Haply Inverse3 + VerseGrip (`mtm_left`, `mtm_right`); GOOVIS stereo head-mounted display with head sensor; teleop footswitch (clutch, camera, record); ESU pedals |
| **Supervisor station** | Monitors 1–2, keyboard and mouse (the operator GUI and RViz); ARM HOLD button box |

**Control.** Every motion source sends the same message, a tool-tip pose target
(`<arm>/tool_tip_target`). Sources are the teleop follower, the camera control, the gamepad
jog, benchmark scripts and future policies. One controller per arm turns it into joint
commands: a resolved-rate law with the RCM as a hard constraint in a QP, then Ruckig for
jerk-limited smoothing, then UR `speedj` (`servoj` on the camera arm). Teleop and autonomy
differ only in the source.

**Safety.**
- ARM HOLD stops all three arms (UR safeguard stop, Category 2).
- The collision guard holds every arm when two capsules, between arms or within one arm, come
  closer than 20 mm.
- Each controller holds if its reference goes stale, the RCM breaks by more than 5 mm, or the
  instrument state goes stale.
- The teleop follower disengages on any of these and re-engages only on a fresh pedal press.

## Measured performance

All on the production workstation and the real arms. Full reports in
[`docs/benchmarks/`](docs/benchmarks/).

| What | Result |
|---|---|
| Real-time kernel latency under CPU + GPU load | 19 µs worst case |
| Servo loop, UR5e, 10 min | 499.994 Hz, jitter 3.8 % of the period, 0 missed cycles |
| Teleop motion-to-motion latency (120 fps video) | 58 ms |
| Glass-to-glass video latency | 67.8 ms |
| Stereo camera skew (hardware-triggered, PCB) | 2.8 ± 3.2 µs |
| Dual-camera recording, 30 min | 100 % of frames, 0 gaps |
| ARM HOLD stop at 20 mm/s | 1.46 mm |
| Pose repeatability (ISO 9283 RP, optical tracker, 5 poses × 30) | 0.14–0.25 mm per pose, 0 misses |
| RCM drift (optical tracker, 3 × 5 min) | pivot error 0.34–0.37 mm mean per trial, drift ≤ 0.01 mm/min |

---

## Install

```bash
git clone <repo-url> ~/arts && cd ~/arts
./arts_install.sh          # creates ~/arts_local/ (SDKs, third-party workspaces, builds); idempotent
source ~/arts/arts_ros2_ws/install/setup.bash     # the ONLY setup file you ever source to run
```

**Before installing, bring the ROS apt packages up to date** (`sudo apt update && sudo apt upgrade`).
ROS's apt repository is rolling and ARTS cannot pin it; older copies of `ros-jazzy-ur-msgs`,
`ros-jazzy-ur-client-library` and `ros-jazzy-generate-parameter-library(-py)` fail deep in the C++
build rather than at a prerequisite check (outside install, 2026-11-09; ARTS was built with 2.6.0,
2.15.2 and 0.7.6).

The repository holds only recipes, manifests and first-party source. Everything heavy
(> 100 GB) is built into `~/arts_local/`. Vendor-account SDKs, such as FLIR Spinnaker, are
downloaded by hand: the installer says what to download and where to put it.

**No stereo cameras (mock robot, simulation)?** Skip the FLIR SDK entirely:
`ARTS_ENABLE_FLIR=0 ./arts_install.sh`. With cameras, download Spinnaker 4.2.0.88 (Ubuntu 22.04,
amd64) from [Teledyne's Spinnaker page](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/)
(free account; older releases under "All Versions") into `~/arts_local/oem_drivers/spinnaker-4.2.0.88/`.
If only a newer release is offered, extract it into `~/arts_local/oem_drivers/spinnaker-<version>/`
and run `ARTS_SPINNAKER_VERSION=<version> ./arts_install.sh`; the installer reports that this SDK
is not the one ARTS's camera results were measured with. Host setup
(RT kernel, NVIDIA driver, `isolcpus`, udev, DDS) is in
[`docs/host_prerequisites.md`](docs/host_prerequisites.md). The full host log and the
Docker images are in [`docs/host_setup_log.md`](docs/host_setup_log.md).

**Docker** (the same recipes, in containers; needs Docker with the NVIDIA container toolkit):
```bash
./arts_install.sh --path docker      # builds arts-base -> arts-flir -> arts:jazzy (FLIR SDK as for bare metal;
                                     # with ARTS_ENABLE_FLIR=0: arts-base -> arts:jazzy, no SDK)
docker compose -f deploy/docker/compose.yaml run --rm arts \
    src/arts_controllers/test/run_servo_smoke_test.sh                        # a mock arm moves
docker compose -f deploy/docker/compose.yaml run --rm arts \
    ros2 launch arts_driver arts_bringup.launch.py psm_left:=true psm_right:=true ecm:=true rviz:=true
```
The last line runs all three arms on mock hardware with RViz on your display. The smoke test
runs its own mock arm on its own ROS domain, so it does not move the arms in that RViz; add
`gamepad:=true` to the bringup to jog them. `compose.yaml` also has one service per hardware
module (`--profile patient-cart`, `--profile surgeon-console`). No image is published, so
every user builds them; the Docker path is tested on mock hardware only, not yet on the real arms.

**CAD meshes for RViz (optional, one time).** Without them RViz draws the pedestal as a grey
box and the arms without adapters; nothing else depends on them. The arm mounts never come
from a mesh: they are committed in `arts_description/config/mount_transforms.yaml`, so every
install places the arms identically. Meshes are generated into `~/arts_local/description_catalog/`
(never committed; Docker mounts that folder read-only):

1. **Pedestal (Vention's geometry, not redistributable, so you download it):** open the
   pedestal's Vention design page (the link in the pedestal row of
   [`docs/bom/bom.csv`](docs/bom/bom.csv)) → *Open in 3D* (free Vention account) →
   *Downloads* → *Download Assembly*, and save `VentionAssembly_<design>_v<N>.STEP` in
   `~/arts_local/description_catalog/parts/1_pedestal/0_incoming/`.
2. **Import** (pinned venv; the pedestal takes about 80 s and writes an ~80 MB mesh):
   ```bash
   cd ~/arts/arts_ros2_ws/src/arts_description/scripts
   python3 -m venv ~/arts_local/venvs/pedestal_import
   ~/arts_local/venvs/pedestal_import/bin/pip install -r requirements.txt
   PY=~/arts_local/venvs/pedestal_import/bin/python
   $PY -I import_pedestal_step.py ~/arts_local/description_catalog/parts/1_pedestal/0_incoming/VentionAssembly_*.STEP
   $PY -I import_tool_step.py adapter     ~/arts/hardware/tool_adapter/adapter_assembly.step
   $PY -I import_tool_step.py laparoscope ~/arts/hardware/laparoscope/laparoscope_assembly.step
   ```
   The pedestal import ends with a **mount check**: it compares where Vention's assembly puts
   the three arm bases with the committed mounts and prints `mount check PASSED`. If it fails
   (exit 2), your download is a different revision of the stand than ARTS's mounts were read
   from (v8); RViz would show the arms off the pedestal. Do not edit the mounts to match it;
   open an issue.
3. Restart the bringup: `cart_tf` logs `pedestal: Vention mesh ...` and RViz shows the real cart.

**One sourcing rule.** To run, source `~/arts/arts_ros2_ws/install/setup.bash` and nothing
else; it chains down to ROS 2. Builds always run in a clean `env -i` shell
(`./arts_install.sh --arts-only` rebuilds the ARTS workspace).

---

## Running a session

Every session follows the same order. Steps marked **(go)** move hardware: one person holds the
pendant and the e-stop and gives the go.

All terminals start with:
```bash
source /etc/profile.d/arts-dds.sh; source ~/arts/arts_ros2_ws/install/setup.bash
B=~/arts/arts_ros2_ws/src/arts_controllers/bench
```

### 1. Power and pendants
Power on the three arms and release the brakes. **Do not press Play yet.** Check the monitor
layout with `xrandr --listmonitors`: the tower display at `0,0`, the GOOVIS at `1920,0`.

### 2. Instrument start (every session, for every instrument)

An instrument is homed and zeroed at the start of every session, never just re-zeroed:
instrument state can change between sessions.

1. **Instrument OFF the adapter.** Start the homing (profiles: `forceps` for the ProGrasp and
   the bipolar forceps, `needledriver`, `hook`, `scissors`):
   ```bash
   ros2 launch arts_dv_tool_driver tool_homing.launch.py arm:=psm_left tool:=forceps
   ```
   Wait for "Start positions reached". A warning that a motor missed its start means the
   instrument was on: take it off and restart.
2. **Instrument ON, press A** on the gamepad. Wait for "All homing complete" (~30 s), then Ctrl+C.
3. **Visual zero**, with the disc drives only (no arm running):
   ```bash
   ros2 launch arts_driver arts_bringup.launch.py psm_left_tool:=prograsp_forceps psm_left_tool_actuator:=true \
       psm_right_tool:=large_needle_driver psm_right_tool_actuator:=true gamepad:=true allow_mode_write:=true use_fake_hardware:=false
   ```
   Select the arm with the gamepad (**Back** = psm_left, **Start** = psm_right). Hold **LB**
   and press **B** for joint mode, then straighten the wrist with the jaws just closed. If the
   jaws are off, use disc mode (**LB + A**) for them. Set the zero with **LB + X**, then Ctrl+C.

Instrument names (`psm_*_tool:=`): `prograsp_forceps`, `large_needle_driver`,
`bipolar_fenestrated_forceps`, `permanent_cautery_hook`, `monopolar_curved_scissors`.

### 3. Bring up the system

**With the operator GUI** (supervisor station):
```bash
ros2 run arts_gui arts_gui.py
```
- **Modules tab:** pick each PSM's instrument, then start the arms, their disc drives, the
  cameras, MTMs, footswitch, head sensor and ESU. Presets start common sets.
- The GUI starts one cart frame publisher and one collision guard for all arms. Each module's
  log shows the exact terminal command, so any run can be reproduced.
- **Displays tab:** places the GOOVIS and tower views and opens RViz, configured for the
  running arms.
- **Surgery mode tab:** turns each arm's RCM on or off, and enables or disables software energy.

**Or from one terminal** (the same switches):
```bash
ros2 launch arts_driver arts_bringup.launch.py \
    psm_left:=true psm_left_tool:=prograsp_forceps psm_left_tool_actuator:=true \
    psm_right:=true psm_right_tool:=large_needle_driver psm_right_tool_actuator:=true wrist:=true \
    ecm:=true ecm_stereo:=true ecm_stereo_sync:=true head_sensor:=true gamepad:=true \
    display:=true display_fullscreen:=true display_hmd_pos:=1920,0 display_tower_pos:=0,0 \
    image_bag:=$HOME/arts_local/<session_name>/images image_bag_paused:=true \
    rviz:=true use_fake_hardware:=false
```
`image_bag:=` records the stereo video: both cameras at 1536×1024 raw Bayer, 30 Hz, in the
camera process beside the display (decision 5). With `image_bag_paused:=true` nothing is written
until the **record pedal** (right) starts an episode, and its next press ends it; the marks are
on `/console/episode_mark` (the footswitch runs in the teleop step below). Both cameras write
about **5.7 GB per minute**, so check the free space first and export after each session
(step 6). Keep `ecm_stereo_sync:=true` whenever the cameras run: without the sync box's agent the
box reboots about every 12 s and stops triggering for about 175 ms each time.

Then press **Play** on every pendant **(go)**. In RViz the collision capsules are green, yellow
below 50 mm, and red below 20 mm (every arm holds).

### 4. RCM on (minimally invasive mode)

**Through a port (trainer box or cannula).** With the RCM off, jog each instrument through its
port with the gamepad (hold **RB**) until the shaft crosses the port. Then put the RCM point at
the port, `d` metres back along the shaft:
```bash
python3 $B/set_rcm.py on --arm psm_left  --depth <d>
python3 $B/set_rcm.py on --arm psm_right --depth <d>
python3 $B/set_rcm.py on --arm ecm       --depth <d>
```
`d` is measured from a different point on each kind of arm:
- **PSM:** from the instrument's **wrist pitch axis** (the first wrist joint, where the shaft
  ends) back to the port. This holds with `wrist:=true`; without it, measure from the wrist yaw
  axis, 9 mm further out.
- **ECM:** from the **scope's distal tip** back to the port.

Tape marks every 10 mm on each shaft, counted from the wrist (or from the scope tip), let you
read `d` at the port. `set_rcm.py` refuses an RCM that leaves less than 5 mm of insertion.
Check with `set_rcm.py status --arm <arm>`, which prints the deviation and the insertion depth.
Seat cannulas in compliant ports, not rigid clamps: the point is placed by eye, so it can be a
few millimetres off the port's true pivot.

**Without a port** (free air), `on` alone uses the current shaft at the last depth (100 mm
before the first); for the ECM, `--depth 0.05`.

In the GUI, the Surgery mode tab switches the RCM on at the current shaft.

### 5. Teleoperate (go)
```bash
ARTS_TOOL=470093_prograsp_forceps.yaml ARTS_RIGHT_TOOL=471006_large_needle_driver.yaml ARTS_CAMERA=1 \
    ARTS_CAMERA_ARGS="-p camera_max_offset:=0.02" \
    ~/arts/arts_ros2_ws/src/arts_utilities/arts_teleop/test/run_teleop_real.sh ~/arts_local/<session_name> 20
```
This starts the MTM bridge, footswitch, both tool followers, the jaw followers and the camera
control, and records everything (state bag) for the given number of minutes.
- **Head in the GOOVIS** enables. Hold the **clutch pedal** to drive the tools.
- Hold the **middle pedal** to drive the camera with both hands (pan, zoom, steering-wheel
  roll); the tools disengage meanwhile.
- After a camera move, the tools re-engage through a 10° alignment gate.

The `ARTS_TOOL` file names are in `arts_dv_tool_driver/config/instruments/`. Energy:
`ARTS_ESU=1`.

### 6. Shut down
Ctrl+C the teleop, then **stop the pendant programs**, then Ctrl+C the bringup (this closes the
image bag). Then copy the session to an external disk, checked by hash; `--delete` removes the
local copy only if every hash matches:
```bash
~/arts/arts_ros2_ws/src/arts_utilities/arts_record/scripts/export_bag.sh \
    ~/arts_local/<session_name> /media/$USER/<disk>/ [--delete]
```

### Gamepad (Xbox)

| Input | Action |
|---|---|
| Back / Start / Guide | select psm_left / psm_right / ecm |
| hold **RB** | arm jog (deadman). RCM off: left stick = tip sideways, LT/RT = in/out along the shaft, right stick X = roll, right stick Y = pitch, d-pad left/right = yaw. RCM on: left stick = pivot about the RCM, LT/RT = insert/retract |
| hold **LB** | instrument jog, refused while the arm's controller runs. A = disc mode, B = joint mode (left stick X roll, right stick X pitch, d-pad yaw, **RT opens the jaw**), **X = set the zero** |

### Collision guard

The guard is on by default with any arm. Arm links are capsules fitted at start-up to the
vendor collision meshes (one per link, end spheres centred on the joint housings); the adapters,
the first 150 mm of each instrument shaft, the endoscope camera head and the first 200 mm of the
scope are capsules from their CAD; the pedestal is three 22 cm spheres where the arms are held
(between each tool-arm mount and the camera-arm mount, and between the tool-arm mounts).

When it holds:
- every arm holds position, and the followers disengage;
- `/collision_guard/state` names the pair;
- release, then separate the arms:
  ```bash
  ros2 service call /collision_guard/release std_srvs/srv/Trigger
  ```
  After the release, motion that separates the pair is free. Moving closer holds again.

---

## Experiments

| Experiment | Instruments | How |
|---|---|---|
| FLS peg transfer (3 triangles left → right, 3 right → left, mid-air handoff) | 2× large needle driver | RCM on through the trainer box ports. Time, triangles moved, drops, failed handoffs; the bags give path length, engagements and RCM deviation: [`docs/modules/bench_session_20261109.md`](docs/modules/bench_session_20261109.md) |
| Intracorporeal suturing + knot on a silicone pad | 2× large needle driver | Same setup; time, knot security, distance from the marked points |
| Porcine cholecystectomy subtask: blunt dissection + monopolar cut | right hook, left bipolar forceps | `ARTS_ESU=1`; energy by pedal or the GUI's Energy box (logged on `/esu/state`) |
| Porcine spleen: suction + spray coag; bipolar coag | right hook, left bipolar forceps (+ suction tool) | Same, ESU mode set on the generator |
| Pose repeatability, RCM drift (optical tracker) | ProGrasp + tracker markers | [`docs/modules/bench_session_20261107.md`](docs/modules/bench_session_20261107.md) |

Mock hardware: any bringup with `use_fake_hardware:=true` (the default) runs without the arms.
The regression tests run on mock through `arts_controllers/test/run_servo_smoke_test.sh <test>`.

---

## Repository

| Path | Contents |
|---|---|
| `arts_ros2_ws/src/arts_driver` | bringup launch (one switch per module), arm launch, configs (arms, instruments, tool tips) |
| `arts_ros2_ws/src/arts_controllers` | the servo controller (resolved-rate law, RCM QP, wrist), bench scripts, mock tests |
| `arts_ros2_ws/src/arts_embedded` | `arts_dv_tool_driver` (disc drive, homing), `arts_esu` (ESU-TAP), `arts_camera_sync` (sync box firmware + PCB) |
| `arts_ros2_ws/src/arts_utilities` | `arts_teleop` (followers, camera control, gamepad, footswitch), `arts_safety` (collision guard), `arts_gui`, `arts_record`, `arts_display`, `arts_kinematics`, `arts_datasets` |
| `arts_ros2_ws/src/arts_description` | URDF/xacro for the cart, arms, adapters and endoscope; RViz configs |
| `middle_overlay_ros2_wss/`, `open_source_software/`, `oem_drivers/` | pinned manifests and recipes for everything third-party (built into `~/arts_local/`) |
| `deploy/` | install modules (bare metal and Docker share them), host setup, benchmarks, CI |
| `hardware/` | adapter and endoscope mechanical designs |
| `docs/` | decisions, module docs, bench procedures, benchmark reports, BOM |

Every dependency is pinned (tag or commit, `=version`, `==version`); see
[`docs/dependency_pinning.md`](docs/dependency_pinning.md). Layout details:
[`docs/repo_layout.md`](docs/repo_layout.md). License: Apache-2.0.
