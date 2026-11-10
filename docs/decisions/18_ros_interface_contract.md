# Decision 18 — The ROS 2 interface contract: arm namespaces, topics, messages, frames

> **Role: decision record.** Locked 2026-11-01. It defines every name a user or a
> collaborator's code meets: the namespaces, topics, message types, frames and joints, and the
> bringup and recording that follow from them. It amends [decision 1](01_orocos_retired.md)
> (one controller manager per arm), [decision 6](06_plain_ros2_naming.md) (role names
> exempted), [decision 12](12_frames_tf_calibration.md) (frame names) and
> [decision 13](13_per_arm_description.md) (assembly names). Each of those carries an
> amendment pointing here. Implementation status lives in AGENTS.md. The per-topic reference
> (type, rate, QoS, publisher) is the module doc `docs/modules/ros_interfaces.md`, kept in
> step with this file.

## The decision

ARTS exposes **one namespace per role** and nothing else at the top level, apart from one
shared device and the global transforms:

| Namespace | Role | Hardware |
|---|---|---|
| `/psm_left`, `/psm_right` | patient-side manipulators | UR5e + UR–da Vinci adapter + da Vinci X instrument |
| `/ecm` | endoscope camera manipulator | UR5 CB3 + da Vinci Si stereo endoscope + 2× FLIR |
| `/mtm_left`, `/mtm_right` | master tool manipulators | Haply Inverse3 + VerseGrip |
| `/console` | operator inputs not bound to one arm | teleop footswitch (Savant), episode marks, GOOVIS head sensor, gamepad |
| `/esu` | the one shared device | electrosurgical unit via the ESU-TAP board |
| `/tf`, `/tf_static`, `/diagnostics` | global, as ROS 2 expects | — |

A user who runs `ros2 topic list` sees these seven namespaces. Collaborator code (the
Dynamixel instrument drive, the ESU-TAP board) is integrated INTO them; it never adds
namespaces of its own. The public contract is the topic list below. Driver internals that the
stock UR launch creates under an arm namespace (e.g. `/psm_left/io_and_status_controller/*`)
are visible but are not part of the contract, so nothing outside `arts_driver` may depend on
them.

## Naming rules

1. **Roles, not devices.** A namespace says what the thing does (`psm_left`), not who made
   it (`ur5e_1`, `dynamixel`, `haply`). The single exception is `/esu`, a device shared by
   both PSMs (see "Energy").
2. **Frames and joints carry the role as an underscore prefix:** `psm_left_base_link`,
   `psm_left_tool0`, `psm_left_tool_tip`, `psm_left_shoulder_pan_joint`. A slash is not
   possible. ros2_control names every hardware interface `<joint>/<interface>` (e.g.
   `shoulder_pan_joint/position`), so a slash inside a joint name breaks the interface
   name. The UR driver's `tf_prefix` is an underscore prefix too. The v1.4 reference hit the
   same wall.
3. **Suffixes say the direction of data:**

   | Suffix | Meaning | Published by |
   |---|---|---|
   | `_target` | what is requested (input) | a user, the teleop follower, a script, a policy |
   | `_command` | what ARTS actually sent to the hardware after checks and limits (logged output) | ARTS |
   | `_state` / `_states` | what was measured (or computed from measurements) | ARTS / the hardware |

   `_states` (plural) is used only for the standard `sensor_msgs/JointState` type
   (`joint_states`, `tool_actuator_states`), following ROS custom. Every other state is
   singular.
4. **`/tf` and `/tf_static` are global.** There is one tree, and its consumers (RViz, Slicer,
   Isaac, vision) need cross-arm lookups such as a tool tip in the camera frame. A node
   started inside an arm namespace remaps `tf` → `/tf` and `tf_static` → `/tf_static`. The
   underscore prefix keeps the frames apart.
5. **SI units throughout** (m, rad, s, N). Consumers that work in other units convert
   (SlicerROS2 multiplies by 1000 itself; see "Bridges").
6. **One input per arm.** Moving an arm takes exactly one topic,
   `<arm>/tool_tip_target`. Every reference source (teleop, script, policy) writes it,
   which is decision 17's "teleop and autonomy differ only in the source" expressed as an
   interface.

## The contract

QoS classes: **L** latched (reliable, transient-local, depth 1); **R** reliable, keep-last
(the recorder subscribes with depth ≥ 1000 so that a burst cannot drop robot samples, the E5
lesson); **S** sensor, as the driver publishes (best effort allowed).

```
/psm_left/                                   (/psm_right/ identical)
  robot_description     std_msgs/String               L          UR5e + adapter + instrument (picked at launch)
  tool_info             arts_interfaces/ToolInfo      L          which instrument, its part number and model files
  joint_states          sensor_msgs/JointState        R  500 Hz  6 UR joints + the instrument's joints
  tool_actuator_states  sensor_msgs/JointState        R  ~500 Hz raw instrument discs: position, velocity, current
  tool_tip_state        arts_interfaces/ToolTipState  R  500 Hz  tip pose, twist, jaw (computed from joint_states)
  tool_tip_target       arts_interfaces/ToolTipTarget R  input   THE way to move the arm
  joint_command         sensor_msgs/JointState        R  500 Hz  copy of what the controller wrote this cycle
  status                arts_interfaces/ArmStatus     L  change  UR safety mode, robot mode, program, scaling
  wrench                geometry_msgs/WrenchStamped   R  500 Hz  UR5e flange force/torque sensor
  teleop_state          arts_interfaces/TeleopState   L  change  engaged / clutched / held, and the source
  rcm_state             arts_interfaces/RcmState      R  500 Hz  RCM on/off, point, measured deviation, insertion
  set_rcm               arts_interfaces/SetRcm        service    switch the RCM on (a point) or off
/ecm/
  robot_description, tool_info (the laparoscope), joint_states (125 Hz, 6 UR joints),
  tool_tip_state (scope tip), tool_tip_target, joint_command, status, teleop_state  — as above
  stereo_left/image_raw      sensor_msgs/Image (Bayer)        S  30 Hz
  stereo_left/camera_info    sensor_msgs/CameraInfo           S  30 Hz
  stereo_left/meta           flir_camera_msgs/ImageMetaData   S  30 Hz   carries camera_time (decision 5)
  stereo_right/              same three
  stereo_sync/pulse          arts_interfaces/CameraSyncPulse  S  30 Hz   ARTS sync box (decision 14)
/mtm_left/                                   (/mtm_right/ identical)
  joint_states          sensor_msgs/JointState        R  500 Hz  the Inverse3's 3 motor angles and velocities
  cursor                geometry_msgs/PointStamped    R  500 Hz  handle point ("cursor" is Haply's own term)
  cursor_velocity       geometry_msgs/Vector3Stamped  R  500 Hz
  stylus_state          arts_interfaces/StylusState   R  ~50 Hz  VerseGrip: orientation, buttons, grip, battery
/console/
  footswitch_teleop     sensor_msgs/Joy               R  change  Savant teleop pedals; meaning from input_mapping.yaml
  episode_mark          arts_interfaces/EpisodeMark   R  change  START / END / CHECKPOINT
  head_sensor           sensor_msgs/Joy               R  20 Hz   GOOVIS presence, buttons [present]; stale = absent
  gamepad               sensor_msgs/Joy               S  change  Xbox controller (joy_node), read by gamepad_jog
/esu/
  info                  arts_interfaces/EsuInfo       L          ESU model, board firmware, output → arm mapping
  target                arts_interfaces/EnergyTarget  R  input   request energy for an arm (or an output)
  command               arts_interfaces/EnergyCommand R  change  what the ARTS node sent to the board
  device_state          arts_interfaces/EnergyState   R  change  raw, as the board senses it (no arm)
  state                 arts_interfaces/EnergyState   R  change  device_state + the resolved arm
/tf  /tf_static  /diagnostics
```

Reserved and not built before Nov 10: `<psm>/wrist_camera/{image_raw, camera_info}` (SRT-H's
`endo_psm1/2`).

### Why each per-arm topic is what it is

- **`joint_states` holds the instrument's KINEMATIC joints, not its discs.** Whatever turns
  joint values into poses (`robot_state_publisher`, and so RViz, Slicer, Isaac; also
  pinocchio) understands only the joints in the robot description. The instrument's
  description joints are its roll, wrist pitch, wrist yaw and jaw. The discs are not
  description joints: they drive those joints through cable coupling, which URDF cannot
  express. So the message holds `psm_left_shoulder_pan_joint` … `psm_left_wrist_3_joint`
  plus the instrument's joints: `psm_left_tool_roll`, `psm_left_tool_wrist_pitch`,
  `psm_left_tool_wrist_yaw`, `psm_left_tool_jaw`. These are joint ANGLES, not an
  orientation. The set depends on the instrument (no jaw on a hook or spatula; decision 13,
  `docs/instrument_identification.md`), and `tool_info` lists it. For the UR, raw and
  kinematic are the same quantity.
- **`tool_actuator_states` is the raw truth for the instrument** (decision 12: record raw).
  Its joints are `psm_left_tool_disc_1` … `_5`, with position, velocity, and current in the
  `effort` field (documented unit: A). Converting discs to joints goes through the coupling
  matrix. When a better matrix is identified, every recording can be recomputed from this
  topic.
- **`tool_tip_state`** is the controller's own forward kinematics, published at no extra
  cost. It is the tip pose (and twist) of `psm_left_tool_tip` in `psm_left_base_link`, plus
  the jaw. It is a convenience and a recorded record. `/tf` still carries the same pose.
- **`tool_tip_target` is the only input.** A quick test is one message (a pose, the other
  fields left at their defaults): the arm moves there with the jaw unchanged. The teleop
  follower sets the velocity feedforward; a policy follower sets pose and jaw. The
  controller accepts any `header.frame_id` known to tf (default `psm_left_base_link`;
  `world` and the camera frame also work) and transforms it.
- **`joint_command` is a copy, not a channel.** The controller writes commands straight
  into the ros2_control interfaces in its own process; no topic sits in between, which is
  the point of ros2_control. Each cycle it also publishes what it wrote: joint position
  targets (UR and instrument) and the commanded joint velocities (the `speedj` values,
  decision 17). The values are the ones sent, in the same cycle. The topic exists for
  comparison against `joint_states` (tracking error), and it is SRT-H's `set_js`.
- **`status`** merges the UR driver's three separate topics (safety mode, robot mode,
  program running) and its speed scaling into one message per arm, so every arm reports
  alike. The teleop follower and the E8 tools read it.
- **`wrench`** comes from the UR5e's built-in flange force/torque sensor (frame
  `psm_left_tool0`). The adapter's weight and the instrument cables offset it, so contact
  detection needs a tracking baseline (collaborator A's approach). The UR5 CB3 has no sensor,
  only a current-based estimate, so `/ecm` has no `wrench`.

### Messages added to or changed in `arts_interfaces`

```
ToolTipTarget.msg
  std_msgs/Header header          frame_id: any tf frame; stamp: when the target was produced
  geometry_msgs/Pose pose
  geometry_msgs/Twist velocity    used only when velocity_valid (else zero feedforward)
  float64 jaw                     rad; used only when jaw_valid (else the jaw holds)
  bool velocity_valid             default false
  bool jaw_valid                  default false
```
The flags exist because a plain default would read jaw = 0, which means closed.

```
ToolTipState.msg
  std_msgs/Header header          stamp = the joint_states sample it was computed from
  geometry_msgs/Pose pose         of <arm>_tool_tip in header.frame_id (<arm>_base_link)
  geometry_msgs/Twist velocity
  float64 jaw                     rad
  float64 jaw_velocity

ToolInfo.msg
  std_msgs/Header header
  string name                     catalog key, e.g. "large_needle_driver"
  string part_number              manufacturer part number
  string family                   e.g. "da_vinci_x"
  uint8 ENERGY_NONE=0
  uint8 ENERGY_MONOPOLAR=1
  uint8 ENERGY_BIPOLAR=2
  uint8 energy_type
  string[] joint_names            the instrument joints present in joint_states
  string[] actuator_names         the discs present in tool_actuator_states
  string kinematics_file          package path of the DH + coupling file used
  string kinematics_sha256        so a recording names the exact numbers it was made with

ArmStatus.msg
  std_msgs/Header header
  string model                    "ur5e" | "ur5"
  uint8 safety_mode               values of ur_dashboard_msgs/SafetyMode
  uint8 robot_mode                values of ur_dashboard_msgs/RobotMode
  bool program_running
  float64 speed_scaling
  bool controller_active          the ARTS servo controller is active

StylusState.msg
  std_msgs/Header header          stamped on receipt (the bridge's receive time)
  geometry_msgs/Quaternion orientation
  bool[] buttons                  a, b, c, as the VerseGrip reports them
  float64 grip                    the Hall-sensor grip value, normalized 0 (open) … 1 (closed)
  float64 battery_level           0 … 1

EnergyTarget.msg                  (new)
  std_msgs/Header header
  string arm                      "psm_left" | "psm_right"; empty when output is given
  uint8 output                    EnergyCommand.MONOPOLAR | BIPOLAR; ignored when arm is set
  uint8 mode                      EnergyCommand.CUT | COAG | BLEND
  bool activate
  uint8 source                    EnergyCommand.SOURCE_*
EnergyCommand.msg                 (existing; `channel` renamed `output`; adds `string arm`, `builtin_interfaces/Duration expiry`)
EnergyState.msg                   (existing; `channel` renamed `output`; adds `string arm`, `uint8 source`, `uint32 heartbeat`)
EsuInfo.msg                       (new)
  std_msgs/Header header
  string esu_model
  string board_firmware           version and commit of the ESU-TAP firmware
  uint8[] outputs                 parallel arrays: output, the arm it feeds, and why
  string[] arms
  string[] mapping_source         "tool_info" | "config"
```
`TeleopState.active_arm` takes the role names (`psm_left` …). `InstrumentState` is retired:
it is superseded by `tool_info`, `joint_states`, `tool_tip_state` and `tool_actuator_states`.
`SetToolTipOffset` stays, as `<arm>/set_tool_tip_offset`.

## Energy

`/esu` is a device namespace because the ESU is one box feeding two arms. Which output
reaches which instrument depends on how the lab is cabled that day, so the board firmware
holds no arm names.

```
pedal / teleop / policy ──► /esu/target ──► ARTS esu node ──► /esu/command ──► ESU-TAP board ──► ESU
                                               ▲    │                                │
               /psm_left/tool_info ────────────┘    ├──► /esu/state  ◄── /esu/device_state
               /psm_right/tool_info                  └──► /esu/info
```

- **The ARTS esu node resolves arm → output from `tool_info`.** Example: left monopolar
  curved scissors and right bipolar forceps give monopolar → `psm_left` and bipolar →
  `psm_right`. It refuses an ambiguous setup and logs why: both arms monopolar, an arm with
  `ENERGY_NONE` asked for energy, or a `tool_info` that changes while energy is active. A
  config file may override the mapping for unusual cabling, and `/esu/info` records which
  source decided each mapping. It also arbitrates between pedal and software and stamps
  each command with its expiry.
- *(Superseded 2026-11-03: the firmware stays unmodified and the esu node translates; see
  the amendment at the end.)* **The board keeps the upstream safety behavior** (500 ms command expiry, the v2 bypass
  fix) and its own pedal path. The firmware is reworked to the contract: `arts_interfaces`
  messages, the `/esu` topics, reliable QoS and a heartbeat. The camera-sync firmware already
  publishes `arts_interfaces` messages over micro-ROS, so that is proven. Upstream is
  Apache-2.0 since commit `cbf2940` (2026-10-26), the same license as ARTS. Modification is
  allowed, keeping upstream's copyright notices and marking changed files
  (`$ARTS_LOCAL/code_intake/esu_tap/NOTES.md`). This supersedes the 2026-10-26 intake verdict
  "flash upstream firmware unmodified". The rework starts from the latest upstream commit
  available when the work begins (`cbf2940` is the newest staged copy).
- **What is measured and what is inferred.** The board senses which ESU output was
  activated. The arm in `/esu/state` is inferred from that output, `tool_info` and the
  cabling, because there is no current sensing at the instrument. Papers say "energy
  routed to", not "energy measured at".

## Frames

The world-frame definition, nominal-versus-measured files and calibration methods stay as in
decision 12. This decision fixes the names and the attachment points:

```
world                     decision 12: the cart datum. Until the calibration board exists it is
                          the Vention STEP origin (nominal); afterwards the board realizes it.
├── pedestal              fixed, identity: the Vention mesh attaches here (its own STEP origin)
├── cart_center           DERIVED convenience frame: nominal centre of the three arm base origins
├── calib_board           (after Nov 10) the physical board
├── psm_left_mount → psm_left_base_link → … → psm_left_tool0 → psm_left_adapter
│                         → psm_left_tool_<links> → psm_left_tool_tip
├── psm_right_mount → … → psm_right_tool_tip
├── ecm_mount → ecm_base_link → … → ecm_tool0 → ecm_scope_head
│                         ├── ecm_stereo_left_optical_frame
│                         ├── ecm_stereo_right_optical_frame
│                         └── ecm_stereo_optical_frame   centre of the rectified pair (teleop's
│                                                        camera frame); nominal from the CAD now
├── trainer_box → cannula_psm_left, cannula_psm_right, cannula_ecm     (RCM points, decision 12)
└── console               nominal placement for display only (not measured)
    ├── mtm_left_base → mtm_left_cursor
    └── mtm_right_base → mtm_right_cursor
```

- Every per-arm description is rooted at its own `<arm>_mount` (decision 13). Exactly one
  static publisher writes the edges below `world`, from
  `arts_description/config/mount_transforms.yaml` (nominal) plus
  `arts_calibration/config/` (measured).
- The UR driver's own frames keep their UR names under the prefix (`psm_left_base`,
  `psm_left_flange`, `psm_left_tool0`).
- `<mtm>_cursor` is published by the Haply bridge from `cursor` and the stylus orientation,
  so RViz and Slicer can show the handle.
- Point clouds (e.g. FoundationStereo) are stamped in `ecm_stereo_left_optical_frame` and
  reach `world` through tf (decision 12: record raw, transform later).
- Optical frames follow the ROS convention: z forward into the scene (depth is +z), x image
  right, y image down. Calibration: one stereo calibration (both intrinsics, left -> right),
  then ONE hand-eye `ecm_tool0` -> left camera; the right follows from the stereo transform,
  and `ecm_stereo_optical_frame` is DERIVED (baseline midpoint, the rectified orientation).
  Until then cart_tf publishes `ecm_tool0 -> ecm_stereo_optical_frame` nominally at the scope
  tip from the laparoscope CAD (2026-11-03).

## Controller managers (amends decision 1)

**One `controller_manager` per arm:** `/psm_left` and `/psm_right` at 500 Hz, `/ecm` at
125 Hz, each a separate process on an isolated core.
- A PSM's manager holds two hardware components: the UR5e, and the instrument drive
  (Dynamixel bus read in its own thread so it cannot stall the loop). One joint-state
  broadcaster therefore publishes UR and instrument joints in one `joint_states` message.
  A second broadcaster publishes the discs as `tool_actuator_states`.
- **Why per arm, not one 500 Hz manager for both UR5e.** The UR driver's launch (dashboard
  client, controller stopper, script sender) is built for one robot per manager, and
  inside a namespace it works almost unchanged. One arm's fault cannot stop the other. A
  manager consumes exactly one `robot_description`, which decision 13 already requires
  per arm. Decision 1's stated reason for grouping the two UR5e was found wrong on
  2026-10-29.
- **Cost:** the future collision QP reads the other arm's `joint_states` over a topic, a
  few milliseconds old, which its 100–300 ms rollout absorbs.

## Bringup

```
ros2 launch arts_driver arts_bringup.launch.py \
  psm_left:=true  psm_left_tool:=large_needle_driver \
  psm_right:=true psm_right_tool:=monopolar_curved_scissors \
  ecm:=true ecm_stereo:=true ecm_stereo_sync:=true \
  mtm_left:=true mtm_right:=true footswitch_teleop:=true head_sensor:=true gamepad:=true \
  use_fake_hardware:=false rviz:=true
```
The `<psm>_tool` argument sets the instrument in the robot description and in `tool_info`
(decision 13: instruments swap by one argument). Switching a module on is one flag;
`arts_driver/config/ur_arms.yaml` is keyed by role (`psm_left`, `psm_right`, `ecm`).

**Switch names follow the topic paths** (amendment 2026-11-01, below): a switch is the
namespace path of the topics it brings up, `/` written `_`. `ecm` is the camera ARM only;
`ecm_stereo_left`, `ecm_stereo_right` (`ecm_stereo` = both) and `ecm_stereo_sync` are the
cameras and the sync box under `/ecm`. `footswitch_teleop` is the Savant teleop pedal set;
the ESU's own pedals are read by the ESU-TAP board and come up with `esu`, under `/esu`.
Reserved for later: `psm_left_wrist_cam` (`/psm_left/wrist_cam`), `tower_suction`,
`tower_irrigation`, `tower_smoke_evac` (`/tower/...`; the support tower, renamed from
"vision cart" 2026-11-02).

## Recording (decision 5)

- **State bag:** every topic above except the images, plus `/tf`, `/tf_static`,
  `/diagnostics`. Latched topics (the three `robot_description`s, `tool_info`, `status`,
  `/esu/info`) are recorded with transient-local QoS overrides, so each bag carries its
  own model, instruments and mapping. That makes it self-describing.
- **Image bag:** `/ecm/stereo_*/image_raw`, `camera_info`, `meta`, with
  `/ecm/stereo_sync/pulse` also in the state bag for the time join (decision 15).

## Bridges

- **RViz:** one RobotModel display per arm, each reading `<arm>/robot_description`, fixed
  frame `world`, in a saved config (`arts_description/rviz/arts.rviz`).
- **SlicerROS2:** adds one robot per arm. It reads the `robot_description` PARAMETER of a
  named node (here `/<arm>/robot_state_publisher`) and looks up every link through `/tf`
  (checked in the v1.4 reference's `slicer_ros2_module`, `vtkMRMLROS2RobotNode.cxx`). It
  scales metres to millimetres itself, so descriptions stay in metres. It cannot read DAE
  meshes, so each description takes `mesh_format:=stl`:
  - UR links use the STL meshes `ur_description` already installs;
  - a mesh authored in millimetres carries `scale="0.001"`;
  - converted meshes are produced locally and never redistributed.
  A saved scene script (`arts_slicer/scripts/arts_scene.py`) adds the five robots.
- **Isaac / Newton:** imports each arm's description and follows its `joint_states`.
- **The paper's synchronized views:** one recorded bag replayed into RViz, Isaac and Slicer;
  each view is captured at the same timestamp as the FLIR frame of that instant.
- **SRT-H datasets** come from post-processing, without new topics. Their `psm1_pose`,
  `psm1_sp`, `psm1_jaw/_sp`, `psm1_js/set_js`, `psm1_rcm_pose`, `ecm_*` and `suj*` columns
  map to `tool_tip_state`, `tool_tip_target`, `joint_states`, `joint_command`, the
  `cannula_*` frames, and `/tf_static`. The images come from `/ecm/stereo_*`, joined
  through `stereo_sync` and `meta` (decision 15).

## Migration from the names in use on 2026-11-01

| Today | Under this contract |
|---|---|
| `/arts_cartesian_servo_controller/target_pose` + `target_velocity` | `/psm_left/tool_tip_target` |
| `/joint_states` (one arm, unprefixed joints) | `/psm_left/joint_states` (prefixed joints) |
| `/tcp_pose_broadcaster/pose` | driver internal; `tool_tip_state` is the contract |
| `/io_and_status_controller/{safety_mode, robot_mode, robot_program_running}` | `/psm_left/status` |
| `/haply/left/cursor`, `cursor_velocity` | `/mtm_left/cursor`, `cursor_velocity` |
| `/haply/left/versegrip/{orientation, buttons}` | `/mtm_left/stylus_state` |
| `/footswitch/joy`, `/episode_mark` | `/console/footswitch_teleop` (first `/console/footswitch`, renamed the same day), `/console/episode_mark` |
| `/camera_left/*`, `/camera_right/*` | `/ecm/stereo_left/*`, `/ecm/stereo_right/*` |
| `/arts_camera_sync/pulse` (firmware) | `/ecm/stereo_sync/pulse` |
| frames `scope_{left,right}_optical_frame` | `ecm_stereo_{left,right}_optical_frame` |
| `ur_arms.yaml` keys `tool_arm_left`, `tool_arm_right`, `camera_arm`; launch `arm:=…`, bringup `tool_arm_left:=…` | `psm_left`, `psm_right`, `ecm` |
| `EnergyCommand/State.channel` | `.output` |

Benchmark reports keep the names they were measured with; they are immutable. Tools that
read old bags keep a compatibility path or say which names they expect.

## Rejected alternatives

- **One `robot_description` for the whole cart.** Each controller manager consumes one
  description, and the three arms run in three managers. SlicerROS2 and RViz both work
  per robot. The v1.4 reference ended up per arm after trying otherwise. A combined
  description survives only as a viewing assembly (decision 13).
- **Namespaced tf (`/psm_left/tf`).** It breaks cross-arm lookups and forces a remap in
  every consumer.
- **Slash-separated frame and joint names.** They break ros2_control interface names (rule 2).
- **Device namespaces (`/haply`, `/dynamixel`, `/esu_tap`, `/footswitch`, `/camera_left`).**
  These scatter one role across several places and expose vendors instead of function.
- **Three input topics per arm** (pose, velocity, jaw). Moving the arm would then need
  three publishers. A single message with validity flags gives a one-message test and
  cannot misread an unset jaw as closed.
- **Energy inside `ToolTipTarget`.** Energy is a discrete, safety-critical command with its
  own expiry, and it can come from the pedal with no motion source. Bundled into a 500 Hz
  motion stream, a dropped motion message would become an energy event.
- **Arm names in the ESU firmware.** Cabling is a property of the day, not of the device.
- **CRTK command names** (`servo_cp`, `measured_js` …). Decision 6 stands for these. Only
  the role names are exempted, because the long alternatives are worse and every surgical
  robotics reader knows PSM, ECM and MTM. ARTS implies no dVRK compatibility.

## Amendment 2026-11-01 — the RCM topics

The RCM controller (decision 17, amendment of the same date) adds two names inside each arm's
namespace, by the rules above: `<arm>/rcm_state` (measured, every cycle: on/off, the point in
`<arm>_base_link`, the kinematic deviation from the measured joints, the insertion depth, and
whether the constraint held this cycle) and the service `<arm>/set_rcm`. `rcm_state` belongs
in the state bag: it carries the S1 metric directly and is the source of the SRT-H
`psm*_rcm_pose` column. The point is in the base frame because the controller does no tf
lookups; a calibrated `cannula_*` frame (decision 12) is transformed by the caller
(`arts_controllers/bench/set_rcm.py --frame`).

## Amendment 2026-11-01 (b) — switch names, the console inputs

Agreed with the PI the same day, after the migration:

- **Bringup switches are named after their topic paths** (see Bringup). The first draft
  had one `mtm` switch for both Haply devices, `stereo` for both cameras and `console` for
  the pedals; each became ambiguous once single-module runs were wanted (one PSM to test the
  RCM; one camera to look at an image; both cameras without the arm). `mtm_left`/`mtm_right`
  share one bridge process (one WebSocket to the Inverse Service), which publishes only the
  sides switched on.
- **`/console/footswitch` → `/console/footswitch_teleop`.** The ESU-TAP board has pedals of
  its own (monopolar, bipolar); a bare "footswitch" no longer said which. The ESU's pedals
  publish under `/esu` with the board, so they get no switch of their own.
- **`/console/head_sensor` is live:** `sensor_msgs/Joy`, `buttons = [present]`, on change
  and at a 20 Hz heartbeat, so a silent topic reads as "head absent", never as present.
- **`/console/gamepad`** (Xbox controller through `joy_node`) feeds `arts_teleop`'s
  `gamepad_jog`, a decision-17 reference source that publishes `<arm>/tool_tip_target`
  (collaborator A's layout, with a deadman).

## Amendment 2026-11-03 — the ESU-TAP firmware stays the designer's; the esu node translates

Decided with the PI when the cart's board (v2, unit 0003) was connected. **Supersedes the
"firmware is reworked to the contract" bullet under Energy.**

- **The board runs its designer's firmware unmodified**, pinned in
  `oem_drivers/esu_tap/VERSION` (protocol 2, `3.2.0-bench`, v2). Reasons: (1) the designer's
  disconnected-fixture bench record (2026-10-22: every channel, pedal takeover, lease expiry,
  agent loss, power loss) holds for that exact build, and a rework voids it; (2) Jazzy needs
  no firmware change: the wire is XRCE-DDS, and the "Humble" in the firmware is only the
  micro-ROS client library (the designer already runs it against a Jazzy agent); (3) the
  deadline. The safety rules stay where they are proven: on the board (pedals work without
  ROS, commands start disabled and expire 500 ms after their last renewal, any pedal press
  ends software control, break-before-make).
- **`arts_esu/esu_node.py` does the translation the rework was going to do**, re-deriving the
  designer's PC client (capability check, lease renewal, missed-renewal disable, logger
  edges): board protocol 2 (`/esu/activation_state`, `/esu/{cut,coag,bipolar}`, `/esu/arm`,
  `/esu/capabilities`) in, the Energy topics out. The board's raw topic stays on the graph and
  is recorded. The board's "arm" is called **enable** everywhere in ARTS.
- **Measured: services do not cross DDS vendors; topics do.** The micro-ROS agent is built on
  Fast DDS only; ARTS runs CycloneDDS (`deploy/cyclonedds.xml`). On 2026-11-03 a CycloneDDS
  call to `/esu/capabilities` never returned, the same call on `rmw_fastrtps_cpp` answered at
  once, and the board's topic reached CycloneDDS and Fast DDS listeners within 1.1 ms of the
  agent writing it (17 of 17 pedal edges). So the esu node runs on `rmw_fastrtps_cpp`, set in
  `esu.launch.py` for that process alone (it refuses to start on anything else), and
  **everything it offers is a topic, never a service**: software energy is enabled by
  `/esu/enable` (`std_msgs/Bool`, each message one deliberate act, deliberately not latched so
  a restarted node cannot re-enable itself). The rest of ARTS stays on CycloneDDS; the camera
  sync box has crossed the same way since 2026-08.
- **Messages fitted to what the board reports.** `EnergyState` carries the three contacts
  (monopolar CUT, monopolar COAG, bipolar), the source (footswitch, software, sync, takeover),
  `software_enabled`, `board_connected`, `timing_suspect` and the active arms; the
  `power_percent`, `interlock_ok` and `heartbeat` fields are gone (the board knows no power
  setting and has no interlock or heartbeat). `SetEnergyMode` is deleted (the board cannot set
  power or mode). `EsuInfo` gains the board's revision, firmware, channels, lease, boot id
  and dropped-message count. BLEND is refused: blend is chosen on the ESU front panel and
  fires on the CUT contact.
- **Late delivery across DDS vendors, and what carries energy timing.** A CycloneDDS reader
  that joins after a Fast DDS writer has already published receives that writer's first new
  sample one writer heartbeat late, and the samples after it wait behind it (reliable, in
  order); Fast DDS's default heartbeat is 3 s. Measured 2026-11-03: twice on the board (a
  2.54 s CUT press reached a Fast DDS listener on time and a late-joined CycloneDDS listener
  3.0 s late, with its release), reproduced with a plain `rmw_fastrtps_cpp` publisher (3.0 s;
  not reproduced when the reader joins first, 12/12 on time after idles up to 300 s). An
  earlier reading the same morning, that the board had held the press back, was wrong: the
  board and the Fast DDS side were on time throughout. Fast DDS readers are unaffected, so the
  esu node receives on time. `arts_esu/config/fastdds_esu.xml` sets a 100 ms writer heartbeat
  for the esu node: its `/esu/state` reached a late CycloneDDS reader 0.101 s late on the
  real board. The micro-ROS agent ignores the profile (its writers take their settings from
  the board's client), so the raw `/esu/activation_state`, which has no stamp, can still be
  logged up to 3 s late by a recorder started after the agent. **Energy timing comes from the
  `header.stamp` of `/esu/state` / `device_state`** (the esu node's receipt time, on Fast DDS);
  the raw topic is recorded for content only. `timing_suspect` stays as a guard: the board
  ignores pedal changes under its 30 ms debounce, so a release reaching the node within that
  of its press was delayed upstream of the node (never seen at the node).
- **Both CP2102N boards are matched by serial number.** The ESU-TAP's DevKitC carries the same
  USB bridge (`10c4:ea60`) as the camera sync box, and the sync box's vendor/product rule
  pointed `/dev/arts_camera_sync` at the ESU-TAP once both were plugged in (seen 2026-11-03),
  so `idf.py -p /dev/arts_camera_sync flash` would have overwritten the ESU-TAP's firmware.
  `deploy/modules/camera_sync.sh` and `esu.sh` now write serial-number rules
  (`/dev/arts_camera_sync`, `/dev/arts_esu`).

## What this does not decide

The cannula calibration (decisions 12 and 17), the
collision QP's interface, the policy follower, the wrist cameras, and the GOOVIS head
sensor. Each of those adds topics only inside the namespaces above, and only by these rules.
