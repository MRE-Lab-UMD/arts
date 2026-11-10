# Decision 13 — Per-arm robot description

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

13. **The robot description is PER-ARM for control and combined ONLY for viewing
    (decided 2026-08-17).** Decision 1 forces this, not taste: a `controller_manager`
    consumes exactly ONE `robot_description`, and the two UR5e tool arms run at 500 Hz
    while the UR5 camera arm runs at 125 Hz, so a single three-arm document cannot be the
    control description. The decision-10 benchmark ladder forces it too — bringing up one
    arm alone must not require the other two to exist. v1.4 had this right and ran both
    modes: `astr_bringup.launch.py` launched two `ur_control.launch.py` instances, each
    with its own `description_file`, namespace, `tf_prefix`, `robot_state_publisher` and
    `controller_manager`, while the combined `astr_one_urdf.urdf.xacro` existed only for
    MoveIt and sim.
    So `arts_description` ships MACROS plus thin assemblies: `tool_arm.urdf.xacro` (one
    arm + adapter + selected instrument + its ros2_control block), `camera_arm.urdf.xacro`
    (UR5 + scope), and `arts_cart.urdf.xacro` (all three, NO ros2_control, for RViz, sim
    and whole-cart collision only). The existing `urdf/arts.urdf.xacro` is that viewing
    assembly and MUST NOT become the control description.
    Instruments swap by ONE launch argument — `instrument:=monopolar_curved_scissors`
    resolving to an include of `urdf/instruments/${instrument}.xacro`. This is a
    deliberate upgrade over v1.4, which swapped the entire top-level file
    (`electrocautery_arm.urdf.xacro` versus `vacuum_arm.urdf.xacro`); the argument form is
    what actually delivers "change the tool piece, not the model".
    TF HYGIENE CONSEQUENCE, and it is the part that bites: per-arm URDFs must be ROOTED AT
    THEIR OWN MOUNT FRAME with no pedestal inside, and EXACTLY ONE node — a static
    publisher fed from the calibration YAML — publishes the cart-level edges. If each
    per-arm URDF declares `world` and `cart_base` for itself, three `robot_state_publisher`
    instances end up publishing the same edge, which is the single-parent violation in
    practice even though each document looks correct in isolation.
    Geometry note: the two tool mounts are tilted 45° NOSE-DOWN and the camera mount is
    vertical, so each arm's `base_link` z-axis differs. `arts_driver/config/
    ur5e_initial_positions.yaml` was derived in an untilted frame and must be re-derived
    per mount for reach into the trainer box — the singularity-avoidance argument behind
    it is unchanged, but the joint values are not transferable.

## Amendment 2026-11-01 — role names and STL meshes (decision 18)

- **Assemblies:** `psm.urdf.xacro` (per PSM: UR5e + adapter + instrument, with ros2_control
  for the UR and the instrument drive) and `ecm.urdf.xacro` (UR5 + laparoscope) replace
  `tool_arm` / `camera_arm`. Each is rooted at `<arm>_mount` and uses the role prefix.
  `arts_cart.urdf.xacro` stays viewing-only.
- **Instrument argument:** the launch argument is `<psm>_tool:=<catalog key>`, and it sets
  `tool_info` as well.
- **Mesh format:** every assembly takes `mesh_format:=dae|stl`. SlicerROS2 cannot read DAE.
  UR links then use the STL meshes `ur_description` installs; meshes authored in mm carry
  `scale="0.001"`.
