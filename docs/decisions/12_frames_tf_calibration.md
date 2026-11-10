# Decision 12 — Frames, TF tree and calibration architecture

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

12. **Frames, TF tree and calibration architecture (decided 2026-08-17).** One tree, one
    parent per frame, and THE ROOT IS A PHYSICAL DATUM. Read this before adding any frame.

    **The root is `world`, defined as the cart datum and physically realized by a
    calibration board rigidly mounted to the pedestal.** A geometrically elegant
    alternative was proposed and REJECTED: the intersection of the camera-mount face
    normal with the line joining the two tool-mount centroids. It fails three tests a
    root frame must pass. It is NON-OBSERVABLE — a virtual point in space that no arm can
    touch and no camera can see, so nothing can ever measure it directly. It is
    SELF-REFERENTIAL — derived from the three mounts that are the very things being
    measured, so re-measuring one mount silently moves the origin and invalidates every
    previously calibrated number elsewhere on the cart. And it is UNREPRODUCIBLE without
    the CAD, which defeats the replication goal. Keep it if wanted as a DERIVED
    convenience frame (`cart_symmetry_point`, a child of `cart_base`, parent of nothing)
    because it is a genuinely useful way to describe the geometry in the paper.
    For the CAD datum use the Vention STEP file's OWN origin, documented against an
    identifiable physical feature (a named corner of the base extrusion, or better a pair
    of tooling holes). It costs nothing, it is traceable, and because the Vention design
    is published by public design ID, a reader who pulls the design gets the identical
    origin for free.

    **Nominal and measured are TWO DIFFERENT FILES, never one.** `arts_description/config/
    mount_transforms.yaml` holds CAD-nominal numbers and is never overwritten by a
    calibration run; `arts_calibration/config/<frame>_correction.yaml` holds the measured
    delta together with its date, method, and residual. Two reasons this is not
    bookkeeping pedantry: the CAD-versus-measured discrepancy is a number the paper wants
    and it is destroyed the moment calibration overwrites nominal, and a fresh clone with
    no calibration yet must still bring up a sane model rather than zeros. The v1.4
    reference did the single-file version (`astr_calibration/config/
    electrocautery_arm_base_link_in_checkerboard.yaml` loaded straight into the URDF joint
    origin via `xacro.load_yaml`) and it works, but it conflates design intent with
    measurement and cannot answer "how far off was the CAD".

    **You can only calibrate what you can observe, which is why the board exists.** No arm
    can observe the pedestal datum, so the datum is realized by an artifact the arms can
    observe. v1.4 already proved this pattern — its `world` frame WAS the physical
    checkerboard plane, with each arm registered into it independently and no arm-to-arm
    calibration at all. ARTS keeps that and upgrades the method. PRIMARY: hand-eye (Zivid
    on each flange, board fixed, N poses, solve AX=XB) because it is non-contact, it
    returns base→board and flange→camera in one solve, and it produces a residual so you
    know when it went wrong. CROSS-CHECK: v1.4's three-point pointer touch of board
    corners, retained precisely because two independent methods disagreeing by more than
    a millimetre is the only cheap way to catch a systematic error. EXTERNAL: the Atracsys
    fusionTrack measures all three arm bases and the board in one tracker frame, which is
    the independent check on the arm-to-arm geometry and is already claimed as an ARTS
    differentiator in the metrics table.

    **Flange-to-tool-tip is NOT a fixed transform, so do not try to calibrate it as one.**
    A wristed da Vinci instrument has three joints between flange and tip. What is fixed
    is flange→instrument-mount; everything beyond it is the DH-plus-coupling model (dVRK
    values for first bring-up, per Phase 4). Pivot calibration and corner-touching both
    require the wrist locked at a known configuration, and what they then measure is the MOUNT
    transform, not "the tip". For teleoperation this does not need to be accurate: the
    surgeon closes the loop visually through the scope, so a roughly-right offset tuned
    live via the existing `~/set_tool_tip_offset` service is sufficient. The rigorous
    measured version stays deferred to `docs/instrument_identification.md`.

    **Point clouds are published in the SENSOR frame and planned in the CART frame, and
    these do not conflict.** REP-103/105: sensor data is stamped in the sensor's own
    optical frame, so `header.frame_id = scope_left_optical_frame` on the raw cloud. The
    scope head is bolted to the camera arm and therefore MOVES, so publishing tissue
    coordinates in the scope frame would make the phantom's xyz change every time the
    camera is repositioned — planning and any world-referenced output therefore live in
    the static cart frame, exactly as v1.4 did (Zivid published in `zivid_link`,
    `pc_helper.py` transformed to `world` before merging). The recording rule that follows
    is absolute: RECORD RAW CLOUDS IN THE SENSOR FRAME PLUS `/tf` AND `/tf_static`, NEVER
    TRANSFORMED DATA AS THE ONLY COPY. A calibration later found to be 2 mm off is
    recoverable offline from raw data plus transforms, and permanently unrecoverable if it
    was baked in at record time.

    **The laparoscope is never calibrated against the board directly.** Calibrate
    `camera_flange → scope_left_optical` once by hand-eye (move the camera arm, watch the
    fixed board, same math and same artifact as the arm-base step) and L↔R by stereo
    calibration through the scope optics; the scope-to-world relationship then falls out
    of the arm's own kinematics at every instant. Two cautions: the optical center of a
    rod-lens path sits at no mechanical feature you can measure, so ONLY optical
    calibration finds it; and if the scope can roll within its mount, that is a JOINT and
    must be modeled or mechanically locked — not absorbed into a fixed transform.

    **Cannula / RCM frames hang off the trainer box, not off the arms.** An RCM is a
    property of the patient, not of the robot, so each arm's QP LOOKS UP its target rather
    than being kinematically parented to it. This also keeps the single-parent rule intact
    when two arms share one workspace.

    The tree (calibration loops are MEASUREMENTS, not TF edges — pick the spanning tree
    and decide deliberately where the loop-closure error lives):

    ```
    world                        == cart datum, physically realized by the calibration board
    ├── cart_board               static, from board mounting
    ├── cart_base                pedestal datum → pedestal geometry
    │   ├── tool_left_mount  → tool_left_base_link  → … → tool_left_flange
    │   │                            → adapter → instrument joints → tool_left_tip
    │   ├── tool_right_mount → tool_right_base_link → … → tool_right_flange → …
    │   └── camera_mount     → camera_base_link     → … → camera_flange
    │                                → scope_mount → scope_head
    │                                     ├── scope_left_link  → scope_left_optical_frame
    │                                     └── scope_right_link → scope_right_optical_frame
    └── trainer_box → cannula_left / cannula_right / cannula_scope   (RCM points)
    ```

## Amendment 2026-11-01 — frame names (decision 18)

Decision 18 renames the frames in the tree above. Every frame carries its role as an
underscore prefix (a slash breaks ros2_control interface names):
- `tool_left_*` → `psm_left_*`, `tool_right_*` → `psm_right_*`, `camera_*` → `ecm_*`;
- `scope_{left,right}_optical_frame` → `ecm_stereo_{left,right}_optical_frame`;
- `cannula_{left,right,scope}` → `cannula_{psm_left,psm_right,ecm}`.

Three frames are added:
- `pedestal`, identity under `world`, where the Vention mesh attaches;
- `cart_center`, the derived convenience frame, defined as the nominal centre of the three
  arm base origins, replacing `cart_symmetry_point`;
- `console`, a display-only placement for the MTMs.

Unchanged: the root and its definition, nominal versus measured files, the calibration
methods, and the record-raw rule. Until the board exists, `world` is the Vention STEP origin.
