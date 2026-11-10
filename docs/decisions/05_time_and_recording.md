# Decision 5 — Time and recording

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

5. **Time & recording:** one PTP (IEEE 1588) clock domain; FLIR pair PTP-slaved and fired by a
   shared hardware trigger; ALL data recorded via rosbag2 MCAP with device hardware timestamps
   in `header.stamp`. MCAP is the default bag format in Jazzy — no special install or
   `-s mcap` flag needed. Tune the writer (chunking / zstd presets) via
   `--storage-config-file`; playback auto-detects the format.
   Two recorder processes: bag A robot/teleop/tool/energy (~10 MB/s), bag B
   images on dedicated NVMe (Bayer raw preferred, GPU JPEG fallback; never RGB8, no video
   GOPs for training data). Footpedal events mark episode boundaries. `arts_datasets` exports
   MCAP → LeRobot format for SRT-H.
   AMENDED 2026-08-17 — "hardware timestamps in `header.stamp`" CANNOT be taken literally
   for the stereo pair, and the reason is a property of the driver, not of the cameras.
   Verified by reading the pinned `flir_camera_driver` 3.0.5 source:
   `spinnaker_synchronized_camera_driver`'s `TimeKeeper::getTimeStamp()` takes the camera
   timestamp as an UNNAMED, UNUSED argument and derives stamps from host arrival time +
   chunk `FrameID` + a Kalman-smoothed common-frame-time estimator (`time_estimator.cpp`);
   its README states plainly that "the Timestamp is not used at the moment". By design it
   assigns IDENTICAL stamps to every frame of one trigger pulse — that is the whole point
   of the package, and it means MEASURING L/R SKEW FROM `header.stamp` MEASURES NOTHING.
   The single-camera `spinnaker_camera_driver` is the opposite: it honors
   `adjust_timestamp` and does put the device clock in `header.stamp` (confirmed in FLIR
   Stage 2). That difference between the two drivers of the same repo is the trap.
   The device clock is still reachable and MUST be recorded: both drivers publish
   `~/<cam>/meta` (`flir_camera_msgs/ImageMetaData`) carrying `camera_time`, the camera's
   own clock. Record the meta topics beside the images — negligible bandwidth, and the
   only path to a real per-camera timestamp under the synchronized driver.
   DECIDED 2026-08-19, CANCELLING A PLANNED ACTION ITEM: **ARTS DOES NOT USE
   `spinnaker_synchronized_camera_driver`.** It keeps the SINGLE `spinnaker_camera_driver`
   and puts BOTH cameras as composable nodes in ONE `ComposableNodeContainer`. Upstream's
   `spinnaker_camera_driver/launch/multiple_cameras.launch.py` is the template and is
   already an externally-triggered stereo example (`trigger_mode: On`, chunk mode on,
   `adjust_timestamp: True`, `trigger_overlap: ReadOut`). The Stage 5 checklist previously
   said to switch to the synchronized driver; that is cancelled, and the reasoning lives
   here so nobody re-adopts it on the strength of its name.
   The synchronized driver buys exactly ONE thing — identical `header.stamp` across the
   frames of one pulse — and that is the thing this amendment has just finished declaring
   untrustworthy. ARTS gets stereo correspondence from the trigger itself and skew from the
   sync box, so adopting it means paying for a feature that must then be ignored.
   It also COSTS something real, verified in the 3.0.5 source: when a synchronizer is
   attached, `camera.cpp` takes the synchronizer branch and never reaches the
   `adjust_timestamp` or PTP paths, so the per-camera DEVICE-CLOCK stamp that the single
   driver does provide is silently replaced by the estimator's output. Trading a
   measurement for an estimate is the wrong direction.
   `camera_time` IS NOT A REASON TO PREFER EITHER: the synchronized driver instantiates the
   same `spinnaker_camera_driver::Camera` class, so both publish `~/<cam>/meta` through
   identical code. Only the topic layout differs (`<node>/<cam>/meta` for one composed node
   versus `<cam>/meta` per node).
   WHAT MUST STILL CHANGE IS THE PROCESS MODEL, AND IT IS THE PART THAT MATTERS. ARTS
   currently runs TWO `camera_driver_node` PROCESSES, and upstream's own documentation
   states that Spinnaker "does not support two programs accessing the Spinnaker SDK at the
   same time, even if two different cameras are accessed. Strange things happen, in
   particular with USB3 cameras." That is now the leading explanation for the correlated
   12–15 Hz dropouts logged at rung 5e and provisionally blamed on the host: a single shared
   SDK is the only resource that spans two independent USB controllers, which is exactly the
   simultaneity that ruled out per-controller bandwidth. One container, one process,
   `use_intra_process_comms: True`.
   Also: USB3 cameras have NO PTP support whatsoever, so for the FLIR pair "one PTP clock
   domain" reduces to the shared hardware trigger plus the sync box of decision 14. PTP
   remains the mechanism for any GigE or networked device added later, and the two
   statements do not conflict — they cover different buses.
   AMENDED 2026-10-28 — RECORDING RESOLUTION. Images are recorded at 1536×1024 raw
   BayerRG8 per camera, made in the camera by 2×2 averaging binning, not at the sensor's
   3072×2048. Full resolution was assumed when this decision was written, before any
   hands-on recording. At 30 Hz it is ~189 MB/s per camera (~380 MB/s for the pair, ~1.4 TB
   per hour), which the data does not need: 1620×1080 class images suffice for imitation
   learning (confirmed with collaborator A), and the console display is 1620×1080 per eye
   (`docs/modules/console_display.md`). Binned it is ~47 MB/s per camera.
   Why binning and not scaling on the host: the raw-Bayer rule above stands. A host-side
   resize to 1620×1080 must debayer first, so it produces RGB8 (~157 MB/s per camera, barely
   less than full-resolution Bayer) and is a transform recorded in place of the measurement
   (decision 12). The camera's binning keeps the Bayer mosaic, so the recorded frame is still
   sensor data. Measured 2026-10-28 on both BFS-U3-63S4C, firmware 1801.0.9.0: selector
   `All`, mode `Average` gives 1536×1024 BayerRG8 whose four Bayer sites keep the
   full-resolution levels (R 21.0 vs 21.0, B 17.5 vs 17.6; a neighbour average would make
   them equal), i.e. colour-aware; same field of view; `Sensor` binning has max 1 and `ISP`
   binning is not writable with the ISP off. Binning happens after readout, so sensor timing
   is unchanged: max 59.6 fps either way; a 60 s triggered run gave trigger→exposure 709 µs
   and skew (24057132 − 24057131) −2.767 ± 3.204 µs, worst 10.75 µs, 0 missed strobes,
   against the full-resolution E3 baseline −2.793 ± 3.209 µs. The cost is depth precision
   (half the disparity resolution); full resolution stays one launch argument away
   (`binning:=1`) for when storage and transfer allow, and must be re-benchmarked then.
