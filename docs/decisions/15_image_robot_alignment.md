# Decision 15 — Image-to-robot time alignment for VLA training data

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

15. **Image-to-robot time alignment for VLA training data (decided 2026-08-17).** The
    question this answers is "what makes an (image, action) pair a true statement", and it is
    a different question from teleop latency. Read this before touching `arts_datasets` or
    the recorder.

    **THE DISPLAY IS NOT IN THE DATA PATH, so its latency cannot corrupt the dataset.**
    The GOOVIS path and the recorder are independent consumers of the same stream (a Phase 4
    requirement, and this is why). A frame shown 80 ms late makes the surgeon react 80 ms
    late, and that reaction is stamped when it was actually made, so the recorded pair stays
    true. Display lag degrades how well a human can operate — the glass-to-glass metric,
    ≤ 100 ms — and additionally means the demonstrated policy embeds the operator's reaction
    to a delayed image. Neither is a timestamp problem.

    **A CONSTANT OFFSET IS SURVIVABLE; A VARIABLE ONE IS FATAL.** This sets the actual
    target. If every image is stamped 12 ms early, the model learns the mapping present in
    the data and reproduces it at inference — the bias is absorbed. If the offset wanders
    between 4 and 30 ms frame to frame, one visual scene maps to different actions across the
    dataset and the mapping being learned does not exist. So the goal is NOT zero offset. It
    is an offset that is known, small and STATIONARY, with the residual jitter quantified.

    **The action stream needs no correction at all**, which is what makes this tractable: a
    commanded action IS a ROS message created on the host, so it has no existence prior to
    being stamped. Only the image has to be transported onto that timeline, in three hops:
    (1) image → pulse index, by integer join of the driver's `ChunkData` `FrameID` against
    `CameraSyncPulse.pulse_index` — no time comparison, so no clock can corrupt it;
    (2) pulse index → capture tick, already hardware-measured per camera (decision 14);
    (3) capture tick → ROS time, the ONLY estimated hop, and estimated well because the
    trigger is periodic: fit `ros_time = a·ticks + b` over the pulse messages' agent arrival
    times. The fit averages transport jitter down by ~√N (54,000 samples in a half-hour
    episode at 30 Hz) AND recovers `a`, the ESP32-crystal-to-host-clock ratio, which would
    otherwise appear as slow drift over a long recording. Report the residual as the
    alignment uncertainty.

    AMENDED 2026-08-19 — **HOP 1 AS WRITTEN IS NOT IMPLEMENTABLE, because NEITHER DRIVER
    PUBLISHES `FrameID`.** Verified in the pinned 3.0.5 source: the chunk FrameID is read
    into the driver's internal `Image` struct, used for drop counting and for the
    synchronized driver's gap detection, and then discarded. `flir_camera_msgs/ImageMetaData`
    carries six fields and none of them is a frame counter. Note the false friend that will
    otherwise cost somebody an hour: `sensor_msgs/Image.header.frame_id` is the TF frame
    NAME string, not the chunk counter.
    So hop 1's best property — "no time comparison, so no clock can corrupt it" — is not
    available off the shelf. The replacement keeps the integer RESULT while being honest that
    it is now derived from time: assign each frame to the pulse whose predicted exposure
    instant is nearest, using `camera_time` from the meta topic. This is defensible because
    the camera's device clock and the ESP32's capture counter are timing THE SAME PHYSICAL
    EVENT — exposure start — so the comparison is between two measurements of one instant
    rather than between two unrelated clocks. THE MARGIN IS WHAT MAKES IT SAFE, and it is
    the number to report rather than assert: half a trigger period is 16.7 ms at 30 Hz
    against a few µs of fit residual, about three orders of magnitude, so a misassignment
    requires the clock fit to be wrong by more than an entire frame.
    TWO CONSEQUENCES. The flash test in this decision's verification section is no longer
    optional — it was the assumption-free check on a join that is now estimated, so it is
    the only thing standing behind hop 1. And publishing FrameID is a small, high-value
    upstream contribution (one field on `ImageMetaData`, one assignment in `camera.cpp`)
    that would restore the pure-integer join; worth doing through upstream rather than a
    fork, and not on any critical path.

    **THE STEREO PAIR IS NOT GLOBAL SHUTTER — verified against the FLIR spec 2026-08-17.**
    BFS-U3-63S4C is a Sony IMX178 running ROLLING SHUTTER WITH GLOBAL RESET, and both halves
    of that matter. Global reset means every row starts integrating at one instant, so a
    single well-defined exposure start exists and it is exactly what the `ExposureActive`
    rising edge marks — the sync box is timestamping the right thing. But only the start is
    common: each row's exposure ENDS staggered by its readout, so exposure length grows down
    the frame and one frame's light-collection span is the exposure time plus the FULL SENSOR
    READOUT, ~17 ms at 3072×2048. That is set by the line rate, so LOWERING THE FRAME RATE
    DOES NOT REDUCE IT. At the 60 mm/s tool-speed limit that is ~1 mm of tool travel smeared
    across the frame's vertical extent — the same order as the RCM accuracy budget, so it is
    a caveat the paper states rather than a bug to fix. Shrinking the recorded ROI vertically
    shortens the stagger proportionally if it ever binds. It also produces a top-to-bottom
    brightness gradient that will otherwise be misread as vignetting or bad white balance.
 CONFIGURATION TRAP WITH NO SYMPTOM: on a global-reset sensor `ExposureActive` can assert
 at first-row exposure start (*Any pixel*) or last-row exposure start (*All pixels*), and
 those differ by the whole readout. The wrong choice adds a silent ~17 ms to every
 absolute timestamp while leaving the stereo skew correct, because the error is
 common-mode. ASSERT *Any pixel* EXPLICITLY; never inherit the default.
 AMENDED 2026-08-18 — THAT INSTRUCTION CANNOT BE FOLLOWED ON THIS CAMERA, so the
 requirement changes from "configure it" to "measure it once and record it". The full
 GenICam node map of BFS-U3-63S4C at firmware 1801.0.9.0 was dumped with Spinnaker's
 `NodeMapInfo` and contains NO `ExposureActiveMode` node — the only `Active` features
 present are `Device Indicator Mode`, `Chunk Mode Active`, `SequencerSetActive` and
 `Line Source: ExposureActive` itself. The behaviour is therefore whatever the firmware
 does and is not selectable.
 This costs nothing, because rung 5d already discriminates the two cases by three orders
 of magnitude: first-row assertion puts trigger→`ExposureActive` in the hundreds of
 microseconds, last-row assertion at ~17 ms. There is no ambiguous middle. So
 record the measured figure in the Stage 5 baseline and state which mode the firmware
 implements as a MEASURED fact — which is a stronger claim than a configured one.
 MEASURED 2026-08-18 ON CAMERA 24057131 AT RUNG 5d: **729.5 µs, so the firmware asserts at
 FIRST-ROW exposure start (*Any pixel*)**, spread 8.6 µs over 797 pulses. The budget behind
 that figure is `TriggerDelay` 379 µs (the model's floor) + optocoupler in and out + strobe
 rise, so the earlier "~50–60 µs" estimate was low by omitting `TriggerDelay` — the
 CONCLUSION is unaffected, since the discriminator is three orders of magnitude wide.
 The camera's strobe polarity was ALSO measured rather than derived, and came out opposite
 to the open-collector expectation: the pin idles LOW against the pull-up and rises while
 exposing. See the 5d entry in the status section — a wrong polarity produces a stable,
 tight, entirely plausible number that is silently the exposure's END.
 SEPARATE AND MORE DANGEROUS FINDING FROM THE SAME DUMP: `TriggerDelay` defaults to
 **379 µs** on every trigger selector, and it is a PER-CAMERA setting. Two cameras left on
 differing values produce a skew that is pure configuration artifact — a real number,
 measured correctly by the hardware, describing nothing physical. It is exactly the failure
 shape decision 14 exists to eliminate, arriving through a knob nobody looked at. Set
 `trigger_delay` EXPLICITLY AND IDENTICALLY on both cameras (the driver's own parameter file
 documents the floor as 9) and record the value beside every skew figure. Also from that
 dump, three defaults that must be overridden rather than inherited: `TriggerOverlap` is
 `Off` for the `FrameStart` selector we use (so a trigger arriving during readout is
 dropped, which at 30 Hz with ~17 ms readout plus a 15 ms exposure is marginal — set
 `ReadOut`), `ExposureAuto` is `Continuous` with `ExposureTime` 14996 µs against a 15000 µs
 upper limit (auto exposure makes the strobe width wander and must be `Off` with a fixed
 time), and `ChunkModeActive` is `0`, i.e. THE `FrameID` THIS DECISION'S HOP 1 JOINS ON IS
 NOT BEING TRANSMITTED AT ALL until chunk mode is enabled.
 One piece of good news from the same dump: `Line0` is already `Input` / `OptoCoupled` and
 `Line1` is already `Output` / `ExposureActive` / `OptoCoupled`, and since `User Set
 Default` is `Default` and the camera had just been power-cycled, that is the FACTORY
 default rather than leftover state from the 22.04 experiments. Set both explicitly anyway
 — a vendor default is not a guarantee, and the point of the parameter file is that a fresh
 camera behaves identically.

    **What stays assumed rather than measured, largest first.** The HAPLY is the weakest
    link: the Inverse Service is a closed binary polled over WebSocket, so operator hand
    position carries no device timestamp. The UR arms are better but not free — RTDE gives
    measured joint states a bounded age of about one control period before `read()` stamps
    them, so proprioception sits ~2 ms behind the action stream. Both are small and roughly
    stationary, i.e. the acceptable category above, but MEASURE THEM ONCE and record the
    numbers rather than assuming.

    **Verify the chain, do not trust it.** Two tests, and the second is the acceptance
    criterion. FLASH TEST: fire an LED in the cameras' field of view from the ESP32 on a
    known pulse index; the frame containing the flash must be the frame whose `FrameID` maps
    to that index. Validates hops 1–2 end to end with no modelling, and reuses the latency
    rig already required. CROSS-CORRELATION TEST: move the tool tip in a fast sinusoid, track
    a fiducial on it in the recorded video, compute tip position by FK from the recorded
    joint states, and find the lag maximizing correlation between the two traces. That lag is
    the TOTAL image-to-robot offset of the complete pipeline, measured from the real bag,
    including everything the flash test assumes; its spread across the recording is the
    jitter figure that decides whether the dataset is trainable. Reuses the MOPS
    vision-in-loop apparatus, so it is nearly free.

    **Recording rule, absolute.** Record the sync pulses (`/ecm/stereo_sync/pulse` since decision 18) into the state bag and the
    `~/<cam>/meta` topics beside the images, and NEVER rewrite `header.stamp` at record time.
    The corrected timeline is computed offline in `arts_datasets` at LeRobot-export time,
    because LeRobot carries one timestamp per frame and the export is therefore where an
    offset must be baked in — acceptable only because MCAP still holds the raw ingredients. A
    fit later found wrong is then recoverable; a bag written with corrected stamps as its
    only copy is not. Same principle as the never-record-only-transformed-clouds rule in
    decision 12.
