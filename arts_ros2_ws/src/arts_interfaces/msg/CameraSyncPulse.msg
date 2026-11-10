# One camera-trigger pulse emitted by the ARTS sync box (arts_embedded/arts_camera_sync).
# See AGENTS.md decision 14 and docs/modules/pilot_flir_camera.md Stage 5.
#
# WHY THIS MESSAGE EXISTS: spinnaker_synchronized_camera_driver assigns IDENTICAL
# header.stamp values to every frame produced by one trigger pulse, so L/R skew is
# structurally unmeasurable from image stamps. This message carries the hardware
# measurement instead: all times below come from a single MCPWM capture counter on the
# ESP32, so differences between cameras are exact to one tick.

# stamp is the micro-ROS session estimate of the trigger instant in ROS time. It is the
# COARSE anchor that ties this pulse to robot data (sub-millisecond, and only as good as
# the last session sync). For anything needing microseconds, use the tick fields.
std_msgs/Header header

# Sentinel for "no capture recorded on this channel for this pulse" — camera absent, not
# triggered, strobe not wired, or the edge was missed.
uint32 NO_CAPTURE = 4294967295

# Monotonic pulse counter since firmware boot. Never resets, and is the invariant that
# ties a trigger edge to the image frames it produced (pair it with the driver's ChunkData
# FrameID). Wraps at 2^32, which at 30 Hz is over four years.
uint32 pulse_index

# Trigger edge, raw value of the 32-bit capture counter. Reported for traceability only:
# the counter wraps every 53.7 s at the ESP32's 80 MHz capture clock, so do NOT difference it
# across pulses without unwrapping — trigger_period_ticks below is already wrap-correct.
uint32 trigger_ticks

# Interval from the previous trigger edge to this one, in capture ticks. This is the
# trigger's own jitter, measured in hardware: its spread bounds every sync claim made
# downstream. 0 on the first pulse.
uint32 trigger_period_ticks

# Ticks from the trigger edge to each camera's ExposureActive assertion. Index order
# matches the camera_ids parameter of the sync node; entries beyond camera_count and any
# missed edge are NO_CAPTURE. INTER-CAMERA SKEW IS THE DIFFERENCE BETWEEN TWO ENTRIES —
# each camera's absolute value also contains the optocoupler propagation delay (18 us in,
# 36 us out), which is common-mode and therefore cancels in the difference but not in the
# absolute figure.
uint32[3] exposure_delay_ticks

# Ticks each camera's ExposureActive stayed asserted, same indexing and same NO_CAPTURE
# sentinel. NOT needed to date a frame — exposure_delay_ticks above does that — and carried
# because it is the cheapest available check that the camera did what it was told. It should
# equal the commanded ExposureTime; a value that drifts frame to frame means ExposureAuto is
# still Continuous, and a value differing between cameras is a real stereo asymmetry rather
# than a timing one. It also catches the polarity error found on 2026-08-18: if the firmware
# labels the wrong edge as the start, this comes out as the trigger period minus the
# exposure and every delay above is really a delay to the exposure's END.
uint32[3] exposure_width_ticks

# Number of camera channels actually configured, so a consumer never has to guess which
# array entries are meaningful.
uint8 camera_count

# Nanoseconds per capture tick, read back from the hardware rather than assumed, because
# the capture timer's resolution depends on the MCPWM group clock and the ESP-IDF version.
float32 tick_period_ns

# False until the firmware has completed a micro-ROS time sync. While false, header.stamp
# is meaningless and only the tick fields are usable.
bool time_synced
