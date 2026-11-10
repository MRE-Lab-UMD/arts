# Decision 9 — Haply Inverse3 and the input mapping layer

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

9. **Haply Inverse3** talks through the closed-binary Haply Inverse Service (documented open
   WebSocket/HTTP JSON API) — OEM-driver category, consistent with the openness policy. Bridge
   polls at 500 Hz (matched to UR5e); multi-kHz is unnecessary. Isaac Lab's `HaplyDevice`
   Python client is a good starting point. The Inverse3 serial protocol is not public: the
   service binary is a hard dependency (document this). Operator inputs beyond the cursor —
   VerseGrip buttons AND a programmable 3-pedal footswitch — get an explicit configurable
   mapping layer (one mapping file, not hardcoded) covering clutch, camera control, instrument
   switching, energy activation, and recording marks.

## Amendment 2026-10-26 — the VerseGrip is not a 500 Hz source; new Hub/firmware

**Rate.** "Polls at 500 Hz" holds for the Inverse3 cursor position. It does not hold for the
VerseGrip: collaborator A reports its orientation/button stream arriving at roughly 50 Hz at
the default service latency setting (16) and roughly 200 Hz at setting 1, and a lab-built
Hall-sensor grip lever read through the VerseGrip is slower still. This is a collaborator
observation, not an ARTS measurement. Consequences until it is measured:
- The bridge publishes cursor position and VerseGrip data with their OWN stamps and the
  benchmark reports their rates separately; never resample the VerseGrip up to 500 Hz and
  report it as such.
- Teleop latency (E6) is measured on handle TRANSLATION, which comes from the Inverse3. No
  grasp-latency or orientation-latency number is claimed from the same rig.
- Measuring it is cheap once the bridge runs: per-field message rate and inter-arrival
  spread at latency settings 16 and 1, stylus still and moving.

**Haply Hub 1.5 + device firmware (vendor release, 2026-10).** Inverse3 firmware 7.10.1 plus
new VerseGrip firmware add a VerseGrip "Tracking Profile" (Stable / Balanced / Responsive)
that re-applies orientation calibration to stop drift over hours; Responsive corrects at
shorter intervals and follows orientation more closely. Firmware is flashed from the Hub GUI,
VerseGrip over its own USB-C cable. Rules that follow:
- Adopt it BEFORE the first Haply benchmark, so there is no baseline to invalidate. After
  that, a firmware or profile change is a pin bump and re-benchmarks.
- The Hub version, Inverse3 firmware, VerseGrip firmware, service latency setting and
  tracking profile are recorded in every benchmark report that involves the Haply
  (firmware is device state; it cannot be pinned in the tree, only recorded).
- The profile is a filter on orientation, so it can add orientation lag. Choose it before
  E6 and keep it fixed. The drift claim itself is a vendor claim: verify it with a
  ≥ 1 h docked-stylus orientation log before relying on it for the 2 h teleop metric.
- Answered 2026-10-29: Hub 1.5.0 is published for Linux as a public `.deb` (no account), now
  pinned by URL and SHA-256 in `oem_drivers/haply/VERSION` and installed by
  `deploy/modules/haply.sh`; it bundles the service as a library instead of the prior
  separate 3.4.16 service. Its .deb omits one dependency, `libusb-0.1-4` (the Hub fails to
  start without it); the module installs it pinned. With it the Hub starts on 24.04
  (workstation, 2026-10-29) and listens on 127.0.0.1 ports 10000, 10001 and 10020 (its log; it
  writes `libhv.<date>.log` into the directory it is started from). Still open: which port
  carries the device API, and its protocol version.
- VerseGrip flashing from the Linux Hub 1.5.0 FAILS (2026-10-29, three attempts: 1.3.0 twice,
  1.2.4 once). Each upload completes and the image is set pending, but after the reset the
  Hub mis-detects the pen's USB port as an Inverse3 ("Permission denied", repeating), never
  confirms the image, and the pen reverts to v3.1 on its next power-cycle (stopping
  ModemManager made no difference). NOT Linux-specific: the same failure on a Windows 10
  laptop and a Windows 11 PC with Hub 1.5.0 (2026-10-30), so the earlier plan to flash from Windows instead is
  void; the pen stays on v3.1 pending Haply's answer. The check for any flash is the version
  shown over the wireless link after a power-cycle.
- Haply's answer (email, 2026-10-31): a known UI bug, to be fixed in a release expected in
  early November. The error is false; a pen showing v3.1 HAS the new firmware. So both
  VerseGrips are taken as current. Checked 2026-11-01: the Hub shows the new "Tracking
  Profile" setting for both VerseGrips (1245 and 1237), both set to Off. Off is kept, as
  the setting E6 ran with: the follower uses position only, so orientation filtering played
  no part, and changing the setting is a re-benchmark.
  Haply also says the Inverse3 flash does not hit this bug. **The Inverse3s stay on 7.6
  until after the ISMR submission.** E6 was measured on 7.6, no remaining paper experiment
  uses the Haply, and a flash before the 2026-11-04 freeze would make E6's firmware
  differ from the system that is shipped. After the submission, flash 7.10.1, then repeat
  the bridge rate check and a short E6 as a benchmarked pin bump.

## Amendment 2026-10-30 — ARTS writes the bridge

The third-party `haply_inverse3_ros2` (reserved in `oem_ros2.repos`) republishes the last
frame on a 100 Hz timer with publish-time stamps, which is the resampling this decision
forbids. ARTS's bridge is `arts_driver/haply_bridge.py`: one WebSocket client for all devices,
each message stamped on receipt. The service answers each client reply with one frame, so the
bridge paces its replies to set the rate (500 Hz; unpaced it ran at 3.5 kHz), and every frame
repeats the last VerseGrip state, so VerseGrip topics are published only on change. First
check, both devices and both pens connected: cursors 500.0 Hz (2.00 ± 0.11 ms), pens 44–50 Hz
at the default service latency setting, in line with collaborator A's ~50 Hz.

**Handle axes, measured 2026-10-30** (bridge running, operator facing the device front,
`$ARTS_LOCAL/haply_axes_20261030/`): left Inverse3 (05E4) right = −y, up = +z, toward the
operator = −x; right Inverse3 (05DE) right = −y, up = +z, toward = **+x**, confirmed by three
simultaneous pulls on both. The Hub's handedness setting therefore mirrors one device's frame,
so the handle-to-operator map is per device and may be a reflection
(`arts_teleop/config/teleop.yaml`). It is fixed to the device, so it survives moving the
Haply as long as the operator faces its front.

**Gravity compensation, 2026-11-03** (`$ARTS_LOCAL/haply_gravity_20261103/`). The Hub had
compensation enabled at `scaling_factor` 0 (none) and home return on, so a released handle
went back home. The bridge now sets both on every connection, as collaborator A's node did
(compensation on, home return off), and logs the read-back: the service answers `ok` to a
POST even for a device id it does not have, so the read-back is the check. A's values (left
0.025, right 0.045 kg) left the left handle dropping from the top of its travel (z ≈ 295–358
mm) at up to 148 mm/s, while it held lower down. Both handles carry the same VerseGrip and
grip add-on, so the left was set to 0.045 too: released and left alone, the left handle then
held at z 324, 194 and 56 mm (21, 14, 170 s) within 0.8 mm, the right at 48 mm for 243 s
within 0.1 mm (operator: all three heights hold). The VerseGrip with the grip add-on weighs
~35 g on a scale (each side), so the value that holds the handle is ~10 g more than the
handle: `scaling_factor` (kg per A's comment) covers more than the grip, e.g. part of the
device's own linkage, and is used as a tuned value, not a measured mass. At 0.035 on both
(same test): the left held at z 335, 213 and 54 mm within 0.7 mm, the right sank from every
height (13 descents, up to 213 mm/s). So the left needs about its handle's mass and the right
~10 g more, the difference being in the device, not the grip. 0.045 is kept on both: the one
value tested to hold on both, with nothing floating up (a released handle is also held by
friction, so a range of values passes this test).

**VerseGrips must be switched on (2026-10-31).** After a power-down the pens stay off even
with both transceivers enumerated: the service then lists the Inverse3s but no
`wireless_verse_grip`, and the Hub shows them offline. Pressing each pen's power button
brought both back (1237 at 45 %, 1245 at 58 % battery). A pen can also drop offline while
powered (both its LEDs lit, Hub showing it offline, 2026-11-02, right pen): power-cycling the
pen and re-plugging the transceivers brought it back.

## Amendment 2026-11-05 — the bridge renders forces

The bridge no longer sends only zero forces. `/mtm_<side>/cursor_force` (N, device base frame)
is passed to `set_cursor_force` while it is younger than `force_timeout_s` (0.05 s) and capped
at `max_force_n` (3 N); otherwise zero, so a sender that stops leaves the handles free. Its one
sender is `camera_teleop.py`'s two-hand bar (dVRK's ECM coupling re-derived: 150 N/m on the
change in the hands' distance, cap 2.5 N, only while camera control is on). Measured before
use: 0.5 N for 0.3 s along each device axis moved each lightly held handle positive along that
axis (4–8 mm; right +z 0.2 mm), so force and position share axes
(`$ARTS_LOCAL/haptic_bar_20261105/force_dir_1.txt`). The two Inverse3 origins were measured
by holding the handles level with the ball joints 15 cm apart: the right origin sits
[0.624, 0.018, 0.012] m from the left in the operator frame; before that, the steering angle was
computed from a left-pointing hand line and felt reversed.

**Handle hold, 2026-11-06.** While the head sensor reports no head (or is silent),
`mtm_hold.py` holds both handles where they were, as the da Vinci holds its masters. A first
version rendered a spring on the PC (300 N/m, 5 N s/m, as a cursor force): with the ROS round
trip inside the loop, a small pull grew into a shake that spread to the other Inverse3, and the
PI had to hold both. It now sends the latched point as `/mtm_<side>/cursor_position_target`,
which the bridge forwards as the service's `set_cursor_position` (the service runs the
position loop, ~4 kHz per Haply's 3.5.x docs; no gains are documented). Checked on the left
handle: untouched it stayed within 2.2 mm for 5 s; pushed 12–16 mm it returned each time
without oscillation (`$ARTS_LOCAL/haptic_bar_20261105/hold_left_*.txt`); then both handles with
the head sensor, judged good enough by the PI. Off by default in `run_teleop_real.sh`
(`ARTS_MTM_HOLD=1`). The VerseGrip's orientation has no actuator and is not held.
