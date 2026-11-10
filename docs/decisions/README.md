# Architecture decision records

> **Role: decision rationale (layer 1b).** The full reasoning behind each locked
> decision. `AGENTS.md` carries a one-paragraph summary of each and is authoritative
> for status; these files are authoritative for WHY. Amend a decision here and update
> its summary in `AGENTS.md` in the same commit.

- [01 — OROCOS RTT retired; two rate-grouped ros2_control managers](01_orocos_retired.md)
- [02 — Workspace layering](02_workspace_layering.md)
- [03 — ROS 2 Jazzy on Ubuntu 24.04](03_jazzy_noble.md)
- [04 — Bare metal and Docker, both first-class](04_bare_metal_and_docker.md)
- [05 — Time and recording](05_time_and_recording.md)
- [06 — Plain ROS 2 naming, no CRTK/dVRK vocabulary](06_plain_ros2_naming.md)
- [07 — Simulation](07_simulation.md)
- [08 — Single-PC real time](08_single_pc_rt.md)
- [09 — Haply Inverse3 and the input mapping layer](09_haply_inverse3.md)
- [10 — Module-first development and the benchmark ladder](10_module_first_development.md)
- [11 — No MoveIt](11_no_moveit.md)
- [12 — Frames, TF tree and calibration architecture](12_frames_tf_calibration.md)
- [13 — Per-arm robot description](13_per_arm_description.md)
- [14 — ESP32 stereo sync box and hardware skew measurement](14_camera_sync_box.md)
- [15 — Image-to-robot time alignment for VLA training data](15_image_robot_alignment.md)
- [16 — Funder demo vs ARTS reference run](16_funder_demo_vs_reference_run.md)
- [17 — One position-commanded control hierarchy](17_control_hierarchy.md)
