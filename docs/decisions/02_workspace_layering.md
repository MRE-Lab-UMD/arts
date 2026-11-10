# Decision 2 — Workspace layering

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

2. **Workspace layering (owner's design, keep it):** `/opt/ros/jazzy` (the base underlay —
   in ROS 2 "underlay" means this, so the layer above is called a MIDDLE OVERLAY, not an
   underlay) → categorized middle overlays (`ur_ros2_ws`, `flir_ros2_ws`, `slicer_ros2_ws`,
   `atracsys_ros2_ws`, `isaac_ros2_ws`, `oem_ros2_ws`; each depends only on the base, never
   on a sibling; version-pinned via vcstool `.repos` manifests; each built in its own `env -i`
   shell) → `arts_ros2_ws` on top (ALL first-party `arts_*` packages, this repo) → per-project
   application overlays (users only `source ~/arts/arts_ros2_ws/install/setup.bash`). Full
   tree in "Repo layout" below.
