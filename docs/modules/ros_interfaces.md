# ROS 2 interfaces — the topic, frame and joint reference

> **Doc layer 2 — module overview.** WHAT every public name is: topic, type, rate, QoS,
> publisher, consumers, bag, and where it stands today. The WHY is
> [decision 18](../decisions/18_ros_interface_contract.md); status checkboxes live in
> AGENTS.md. When a name changes in code, this file changes in the same commit.
>
> | | Where | For |
> |---|---|---|
> | Decision | `docs/decisions/18_ros_interface_contract.md` | why the names are what they are |
> | **You are here** | this file | the reference: look a name up here |
> | Messages | `arts_ros2_ws/src/arts_interfaces/msg/` | field definitions |
> | Bringup | `arts_driver/launch/arts_bringup.launch.py` | what starts each publisher |

**Status column:** **live** — published today under this name; **old: `<name>`** — published
today under the old name, migration pending; **planned** — not yet published.

## Namespaces

| Namespace | Role | Hardware | Controller manager |
|---|---|---|---|
| `/psm_left`, `/psm_right` | patient-side manipulator | UR5e + UR–da Vinci adapter + da Vinci X instrument | one per arm, 500 Hz |
| `/ecm` | endoscope camera manipulator | UR5 CB3 + da Vinci Si stereo endoscope + 2× FLIR BFS-U3-63S4C | 125 Hz |
| `/mtm_left`, `/mtm_right` | master tool manipulator | Haply Inverse3 + VerseGrip | — (bridge node) |
| `/console` | operator inputs | Savant Elite2 teleop footswitch, GOOVIS head sensor, Xbox gamepad | — |
| `/esu` | shared device | ESU through the ESU-TAP board | — |

QoS: **L** latched (reliable, transient-local, depth 1); **R** reliable, keep-last (recorder
depth ≥ 1000); **S** sensor, as the driver publishes. Bag: **state** / **image** (decision 5).

## `/psm_left` (and `/psm_right`, identical with the prefix changed)

| Topic | Type | Rate | QoS | Publisher (node — package) | Consumers | Bag | Status |
|---|---|---|---|---|---|---|---|
| `robot_description` | `std_msgs/String` | — | L | `robot_state_publisher` — `ur_arm.launch.py` | RViz, Slicer (as the node's parameter), Isaac, controller | state | live (UR arm only; instrument chain planned) |
| `tool_info` | `arts_interfaces/ToolInfo` | — | L | `tool_info` — `arts_driver` (`config/instruments.yaml`) | esu node, datasets, controller | state | live (part numbers and joint lists empty until filled) |
| `joint_states` | `sensor_msgs/JointState` | 500 Hz | R | `joint_state_broadcaster` — ros2_control | `robot_state_publisher`, follower, Isaac, datasets | state | live (6 UR joints; instrument joints with the drive) |
| `tool_actuator_states` | `sensor_msgs/JointState` | ~500 Hz | R | `tool_actuator_state_broadcaster` — ros2_control | datasets, instrument identification | state | planned |
| `tool_actuator_states` (interim) | `sensor_msgs/JointState` | 200 Hz | R | `disc_drive` — `arts_dv_tool_driver` | gamepad jog, bench | state | live: names `disc_1`…`disc_5`, effort in mA (raw servo units), until the ros2_control component |
| `tool_actuator_target` (interim) | `sensor_msgs/JointState` | — | R | gamepad jog (disc mode) | `disc_drive` | input | live: disc velocities, rad/s |
| `tool_joint_states` (interim) | `sensor_msgs/JointState` | 200 Hz | R | `disc_drive` with an instrument file | gamepad jog, bench | state | live: `<arm>_tool_<joint>`, rad, from the visual zero; moves into `joint_states` with the ros2_control component (a jawless instrument still lists `jaw`, held at 0) |
| `tool_joint_target` (interim) | `sensor_msgs/JointState` | — | R | gamepad jog (instrument mode) | `disc_drive` | input | live: joint velocities, rad/s, slowed near the limits |
| `tool_jaw_target` | `std_msgs/Float64` | ~100 Hz | R | `jaw_follower` (VerseGrip lever), gamepad jog (RT) | `disc_drive` | input | mock-checked 2026-11-05: the unitless jaw command, clamped to [-10, 90]: 0..90 = opening share of the instrument's jaw max, 0 = just closed, below 0 = grip (the grip current cap rises linearly to 1000 at -10; never below 150); overrides a jaw velocity in `tool_joint_target` while fresh |
| `tool_tip_state` | `arts_interfaces/ToolTipState` | 500 Hz | R | `servo_controller` — `arts_controllers` | follower, datasets | state | live (tip = tool0 + offset; jaw NaN until the drive) |
| `tool_tip_target` | `arts_interfaces/ToolTipTarget` | per source | R | teleop follower, scripts, policy follower | `servo_controller` | state | live (frame: the arm's base only so far; jaw accepted, unused until the drive) |
| `joint_command` | `sensor_msgs/JointState` | 500 Hz | R | `servo_controller` — `arts_controllers` | datasets, tracking analysis | state | live (UR joints) |
| `status` | `arts_interfaces/ArmStatus` | on change | L | `arm_status` — `arts_driver` | follower, E8 tools, esu node | state | live |
| `wrench` | `geometry_msgs/WrenchStamped` | 500 Hz | R | `force_torque_sensor_broadcaster` — `ur_controllers` (remapped) | contact monitor (later) | state | live (UR5e) |
| `teleop_state` | `arts_interfaces/TeleopState` | on change | L | `teleop_follower` — `arts_teleop` | console, datasets | state | live |
| `set_tool_tip_offset` (service) | `arts_interfaces/SetToolTipOffset` | — | — | `servo_controller` | operator | — | live (refused while the RCM is on) |
| `rcm_state` | `arts_interfaces/RcmState` | 500 Hz | R | `servo_controller` — `arts_controllers` | follower, datasets, S1 analysis | state | live on mock (deviation = point to infinite shaft line, measured joints) |
| `set_rcm` (service) | `arts_interfaces/SetRcm` | — | — | `servo_controller` | operator (`bench/set_rcm.py`) | — | live on mock; off at every activation |

**Joints in `joint_states`** (UR order as the driver reports, then the instrument):
`psm_left_shoulder_pan_joint`, `psm_left_shoulder_lift_joint`, `psm_left_elbow_joint`,
`psm_left_wrist_1_joint`, `psm_left_wrist_2_joint`, `psm_left_wrist_3_joint`, then whichever
of `psm_left_tool_roll`, `psm_left_tool_wrist_pitch`, `psm_left_tool_wrist_yaw`,
`psm_left_tool_jaw` the instrument has (`tool_info.joint_names`). The joints are in rad;
the jaw is the opening angle in rad, 0 = closed.

**Joints in `tool_actuator_states`:** `psm_left_tool_disc_1` … `psm_left_tool_disc_5`
(those present are listed in `tool_info.actuator_names`). Fields: position (rad, at the disc),
velocity (rad/s), effort = motor current (A).

**`joint_command`:** the same joint names as `joint_states`. Position = the joint target
written this cycle; velocity = the commanded joint velocity (the `speedj` value for the UR
joints).

## `/ecm`

| Topic | Type | Rate | QoS | Publisher | Bag | Status |
|---|---|---|---|---|---|---|
| `robot_description`, `tool_info`, `joint_states` (6 UR joints), `tool_tip_state`, `tool_tip_target`, `joint_command`, `status`, `teleop_state` | as `/psm_left` | 125 Hz for joint data | as `/psm_left` | as `/psm_left` | state | live on mock hardware (125.0 Hz); never run on the real UR5 |
| `stereo_left/image_raw` | `sensor_msgs/Image`, Bayer RG8, 1536×1024 | 30 Hz | S | `stereo_left` — `spinnaker_camera_driver`, composed | image | live in the launch file (not yet run under this name) |
| `stereo_left/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | S | same | image | as above |
| `stereo_left/meta` | `flir_camera_msgs/ImageMetaData` | 30 Hz | S | same | image | as above |
| `stereo_right/*` | same three | 30 Hz | S | `stereo_right` | image | as above |
| `stereo_sync/pulse` | `arts_interfaces/CameraSyncPulse` | 30 Hz | S (best effort, micro-ROS) | `arts_camera_sync` firmware (node `/ecm/stereo_sync`) via `micro_ros_agent` | state + image | firmware built, not flashed: the board still publishes `/arts_camera_sync/pulse` |

No `wrench`: the UR5 CB3 has no force/torque sensor.

## `/mtm_left` (and `/mtm_right`)

| Topic | Type | Rate | QoS | Publisher | Bag | Status |
|---|---|---|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` — `mtm_left_joint_1` … `_3`, the Inverse3's motor angles (Haply `angular_position` / `angular_velocity`, reported in degrees, published in rad) | 500 Hz | R | `mtm_bridge` — `arts_driver` | state | live |
| `cursor` | `geometry_msgs/PointStamped`, frame `mtm_left_base` | 500 Hz | R | `mtm_bridge` | state | live (499.9 Hz, 2026-11-01) |
| `cursor_velocity` | `geometry_msgs/Vector3Stamped` | 500 Hz | R | `mtm_bridge` | state | live |
| `stylus_state` | `arts_interfaces/StylusState` (orientation, buttons, grip = Haply `hall` raw value, battery) | ~50 Hz, on new data | R | `mtm_bridge` | state | live |

Device to side: `arts_driver/config/haply.yaml` (Inverse3 05E4 → left, 05DE → right; VerseGrip
1245 → left, 1237 → right). All values are stamped when the bridge receives the service
frame. One bridge process serves both sides.

## `/console`

| Topic | Type | Rate | QoS | Publisher | Bag | Status |
|---|---|---|---|---|---|---|
| `footswitch_teleop` | `sensor_msgs/Joy` (left, middle, right) | on change | R | `footswitch_teleop` — `arts_teleop/footswitch.py` | state | live |
| `episode_mark` | `arts_interfaces/EpisodeMark` | on change | R | `footswitch_teleop` (record pedal) | state (image bag until the state bag exists) | live |
| `head_sensor` | `sensor_msgs/Joy` (`[present]`) | on change + 20 Hz | R | `head_sensor` — `arts_teleop/head_sensor.py` | state | built 2026-11-01; needs `deploy/modules/head_sensor.sh` (udev) |
| `gamepad` | `sensor_msgs/Joy` (Xbox 360 layout) | on change | S | `gamepad` — `joy` package `joy_node` | — | live; consumer `gamepad_jog` |

What each pedal means comes from `arts_teleop/config/input_mapping.yaml`, not from the topic.

## `/esu`

| Topic | Type | QoS | Publisher | Bag | Status |
|---|---|---|---|---|---|
| `info` | `arts_interfaces/EsuInfo` | L | `esu` — `arts_esu` | state | built 2026-11-03, mock-tested |
| `target` | `arts_interfaces/EnergyTarget` (hold-to-run, >= 10 Hz) | R | teleop, policy, bench | state | consumer built; no producer yet |
| `enable` | `std_msgs/Bool` (each message one act; not latched) | R | operator (GUI, later) | state | consumer built |
| `command` | `arts_interfaces/EnergyCommand` | R | `esu` — `arts_esu` | state | built, mock-tested |
| `device_state` | `arts_interfaces/EnergyState` (no arms) | R | `esu` — `arts_esu`, decoded from the board | state | built, mock-tested |
| `state` | `arts_interfaces/EnergyState` (with the active arms) | R | `esu` — `arts_esu` | state | built, mock-tested |
| `activation_state` | `std_msgs/Int32` (board protocol 2: bits 0-3 channels, 4-5 source) | R | ESU-TAP firmware via `micro_ros_agent` | state (raw) | live 2026-11-03 (v2 board) |

The board's own services (`/esu/{cut,coag,bipolar}`, `/esu/arm`, `/esu/capabilities`) are
called only by the `esu` node and are not part of the contract. They live on Fast DDS (the
agent) and do not answer CycloneDDS clients, so the `esu` node runs on `rmw_fastrtps_cpp` and
offers ARTS topics only (decision 18, amendment 2026-11-03). The board's "arm" is called
**enable** in ARTS. One commanding program per ROS domain: never run the designer's own
commander beside the `esu` node.

## Global

| Topic | Publishers | Bag |
|---|---|---|
| `/tf` | each arm's `robot_state_publisher` (remapped from its namespace), `mtm_bridge` (cursor frames) | state |
| `/tf_static` | `cart_tf` — `arts_driver` (the edges below `world`, from `arts_description/config/mount_transforms.yaml`; calibration corrections when they exist), `robot_state_publisher`s (fixed joints), each arm's servo controller (`<arm>_tool0 → <arm>_tool_tip` with the tool-tip offset in use, re-sent on every accepted `set_tool_tip_offset`) | state |
| `/diagnostics` | each controller manager (loop timing) | state |

## Frames

```
world → pedestal | cart_center | calib_board (later)
      → psm_left_mount → psm_left_base_link … psm_left_tool0 → psm_left_adapter → psm_left_tool_* → psm_left_tool_tip
      → psm_right_mount → … → psm_right_tool_tip
      → ecm_mount → ecm_base_link … ecm_tool0 → ecm_scope_head → ecm_stereo_{left,right}_optical_frame
      → trainer_box → cannula_psm_left | cannula_psm_right | cannula_ecm
      → console → mtm_left_base → mtm_left_cursor | mtm_right_base → mtm_right_cursor
```
Until the adapter and instrument links are in the description, `<arm>_tool_tip` hangs directly
off `<arm>_tool0` and is published by the servo controller itself, so in RViz it marks where
the controller thinks the tip is (the `tip:=` preset or the last `set_tool_tip_offset`).
`world` is the Vention STEP origin until the calibration board exists; `cart_center` is
derived (decision 12 amendment). Edges below `world` and the `console` placement are nominal
until calibrated.

## What a bridge needs

- **RViz:** one RobotModel display per arm (`<arm>/robot_description`), fixed frame `world`;
  saved as `arts_description/rviz/arts.rviz` (`arts_bringup.launch.py rviz:=true`).
- **SlicerROS2** (for collaborator E):
  - **Platform:** Ubuntu 24.04, ROS 2 Jazzy, CycloneDDS.
  - **Robots:** five, one per arm and master device. For each arm, the
    `robot_description` parameter is on node `/<arm>/robot_state_publisher`, and links
    come from `/tf`.
  - **Meshes and units:** STL via `mesh_format:=stl` (live: the UR links then use the STL
    meshes `ur_description` installs; checked 2026-11-01, no DAE left in the description);
    descriptions in metres (SlicerROS2 scales to mm).
  - **Images:** optional, `/ecm/stereo_left/image_raw` (Bayer).
  - **Pins:** ARTS pins `slicer_ros2_module` v1.2; the v1.4 reference ran commit `7d8ab6e`
    (May 2024).
  - **Agreed with E (2026-11-02):** once the ARTS repo is complete, E tests Slicer and
    SlicerROS2 against it on Ubuntu 24.04 + ROS 2 Jazzy and confirms the working versions;
    ARTS does not test them before then. Loading the ARTS descriptions and meshes is ARTS's
    job, and the import path was already exercised on the prior system.
- **Isaac / Newton:** each arm's description imported as an articulation, driven from
  `<arm>/joint_states` (live or replayed).
- **Datasets (SRT-H style), by post-processing:**
  - `tool_tip_state` → `psm*_pose`;
  - `tool_tip_target` → `psm*_sp`;
  - `joint_states` / `joint_command` → `psm*_js` / `psm*_set_js`;
  - jaw → `psm*_jaw` / `_jaw_sp`;
  - `rcm_state.point` (or the `cannula_*` frames) → `psm*_rcm_pose`;
  - `/tf_static` → `suj*`;
  - `/ecm/stereo_*` → `left_img_dir` / `right_img_dir`, joined through `stereo_sync/pulse`
    and `meta` (decision 15).
