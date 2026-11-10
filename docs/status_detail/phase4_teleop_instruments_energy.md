# Phase 4 — Teleoperation, instruments, energy — status narrative

> **Role: status narrative archive.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Checkboxes here are a SNAPSHOT and are NOT authoritative — status lives only in
> `AGENTS.md`. Narrative that duplicates a module doc is still to be de-duplicated
> into that module doc (see `docs/repo_structure_triage.md`). Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

- [ ] Haply Inverse3 bridge (`oem_ros2_ws/haply_inverse3_ros2`, pinned; thin ARTS-side
      config/launch in `arts_driver`; Inverse Service WebSocket, 500 Hz poll)
- [ ] `arts_utilities/arts_teleop`: clutch, 3:1–5:1 scaling, camera-frame mapping
- [ ] Input mapping layer: VerseGrip buttons + programmable 3-pedal footswitch → clutch,
      camera control, instrument switch, energy, record marks (single configurable mapping
      file)
- [ ] `arts_embedded/arts_dv_tool_driver`: da Vinci X disc drive as ros2_control joints
 (ESP32 micro-ROS bus-mastering Dynamixel servos); ARTS-owned instrument
 kinematics/coupling files — monopolar curved scissors + bipolar fenestrated forceps
 FIRST (the reference-run instruments). Those files are MEASURED, not transcribed: see
 `docs/instrument_identification.md` — but the FULL protocol there is a TODO DEFERRED
 PAST FIRST BRING-UP. TO BRING THE TOOL DRIVE UP, USE dVRK's PUBLISHED DH + COUPLING
 VALUES: they are the only validated numbers for these instruments, so starting from them
 means a misbehaving tool is a bug in ARTS's drive, coupling sign or homing rather than
 one unknown chasing another. (This argument used to be stated as "a funder demo is the
 wrong place to debut an in-house identification"; that framing died on 2026-08-19 when
 the funder demo turned out to run on the legacy stack, but the conclusion is unchanged
 and now rests on the debugging argument, which was always the better one.) Using them is
 fine; the licensing boundary is
 REDISTRIBUTION, not use — keep them in an attributed third-party file in `$ARTS_LOCAL`
 or gitignored, never shipped in the published tree under ARTS's Apache-2.0. Bench-check
 them first, because they are validated for dVRK's hardware and not for the in-house adapter:
 match part numbers (Xi `470xxx` has no dVRK file) and verify disc ordering and signs.
 The big worry — whether the published coupling folds in the PSM's own gear ratios and
 so would not transfer — is ANSWERED (2026-08-12): MOPS's `eua_control/__init__.py`
 factors motor-to-disc (`t0`) SEPARATELY from disc-to-joint (`t1,t2,t3,w`), and its
 disc-to-joint values ARE dVRK's. So dVRK's coefficients are instrument-internal and
 transfer; only `t0` is ours, being a design parameter of the in-house adapter (1 if the
 Dynamixels are direct-drive). The in-house Xi numbers are MOPS's `EUA2Transmission`. Note
 that survey work claiming "MOPS has no coupling matrix" was wrong — it is in the
 driver, not the URDF. Still open: whether EUA2 folds motor-to-disc into t1/t2/t3
 (it declares no t0), and the same disc-ordering/sign check for the bipolar forceps.
 Stage A homing is still ours and still on the critical path — dVRK publishes no homing
 procedure for these instruments. The measured parameter set is for the paper's error
 budget, and replaces the dVRK values before publication. dVRK's and MOPS's instrument geometry is
 UNLICENSED (verified 2026-08-11 — dVRK's `instrument-cad` deliberately ships with no
 license at Intuitive's request, MOPS has no LICENSE file at all), so none of it may be
 reused, and dVRK's tool JSONs must not be transcribed either. Reverse-engineering from
 instruments we own is clean, and the resulting numbers are better than the published
 ones because they carry an uncertainty. Note Stage A of that protocol depends on
 Dynamixel present-current feedback for hard-stop homing.
- [ ] `arts_embedded/arts_esu` ESU-TAP PCB interposer (ESP32 micro-ROS): footswitch sense-then-
  forward + software energy activation, both logged — monopolar + bipolar channels first.
  The board already exists physically but its DESIGN SOURCE IS NOT IN THIS TREE; recovering it
  into `arts_esu/hardware/` is listed under FLIR Stage 5 because the sync-box PCB depends on
  the toolchain and house style it establishes. See decision 14, PCB workflow.
- [ ] GOOVIS stereo display path (GStreamer/GPU, independent of recorder)
- [x] Safeguard stop (ARM HOLD), 2026-09-22: one supervisor-desk station, monitored manual
 reset, Category 2 on the safeguard-stop inputs of all three arms; on 2× UR5e and the UR5
 CB3 it froze them without a drop, stayed frozen when released, and resumed only after
 reset. Stopping time not measured. Single station is final (2026-10-20). See
 `docs/modules/safeguard_stop.md`.
- [ ] Exit: full teleoperated peg transfer recorded and replayed from the bag
- [ ] **ARTS reference run: teleoperated partial nephrectomy on hydrogel phantom on
      24.04 + Jazzy, recorded end-to-end as the ISMR example dataset.** NOT the Oct 5 funder
      demo, which runs on the legacy Humble stack and contributes no data to the paper — see
      "Two separate deliverables" above. No fixed external date, so schedule it once the
      recording pipeline of Phase 3 is actually trustworthy rather than to hit a deadline.
