# Phase 3 — Vision and recording — status narrative

> **Role: status narrative archive.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Checkboxes here are a SNAPSHOT and are NOT authoritative — status lives only in
> `AGENTS.md`. Narrative that duplicates a module doc is still to be de-duplicated
> into that module doc (see `docs/repo_structure_triage.md`). Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

- [ ] FLIR pair: shared hardware trigger from `arts_embedded/arts_camera_sync` (decision 14).
      NO PTP on this pair — USB3 has none; the ESP32 sync box replaces it. Device clocks are
      recorded via the `~/<cam>/meta` topics, NOT `header.stamp` (decision 5 amendment).
      Delivered by FLIR Stage 5 above; this row tracks its integration into the recorder.
- [ ] `arts_utilities/arts_record`: dual-bag MCAP profiles (state bag + image bag on NVMe).
      Must record `/tf` + `/tf_static` and the raw sensor-frame clouds — never transformed
      data as the only copy (decision 12), and `/arts_camera_sync/pulse` into the state bag
      with the `~/<cam>/meta` topics beside the images. `header.stamp` is NEVER rewritten at
      record time (decision 15).
- [ ] MEASURE which `ExposureActive` mode the firmware implements, because it CANNOT be
 asserted: BFS-U3-63S4C firmware 1801.0.9.0 has no `ExposureActiveMode` node
 (verified 2026-08-18, decision 15 amendment). Rung 5d separates the two cases by
 three orders of magnitude — ~50–60 µs for first-row assertion versus ~17 ms for
 last-row — so record the measured value and state the mode as a measured fact.
 Set `trigger_delay` explicitly and IDENTICALLY on both cameras in the same pass:
 it defaults to 379 µs, it is per-camera, and a mismatch manufactures a skew figure
 that is correct as a measurement and meaningless as physics.
- [ ] `arts_utilities/arts_datasets`: MCAP → LeRobot export; offline alignment by the ESP32
      pulse index and `camera_time`, not by `header.stamp`. The `ros_time = a·ticks + b` fit
      over pulse arrivals lives here — it is the only estimated hop in the chain, and its
      residual is the alignment uncertainty the paper reports (decision 15).
- [ ] Alignment verification, both tests of decision 15: flash test (ESP32-fired LED on a
      known pulse index validates the `FrameID`↔`pulse_index` join with no modelling) and
      the cross-correlation test (vision-tracked tip fiducial vs FK tip position — the total
      pipeline offset AND its jitter, measured from a real bag). The jitter is the number
      that decides whether the dataset is trainable; bias is absorbed by training.
- [ ] Exit: L/R skew within trigger jitter; image-to-robot lag jitter ≤ one frame period;
      dataset loads in an SRT-H-style trainer
