# Bench session 2026-11-01 — first hardware runs of the 2026-11-01 modules

**Layer 3 (bench doc): do this at the bench, in order.** Status lives in `AGENTS.md`;
results go to `docs/benchmarks/` and a one-line headline beside the checkbox.

Every terminal: `source /etc/profile.d/arts-dds.sh; source ~/arts/arts_ros2_ws/install/setup.bash`.
The GUI: `ros2 run arts_gui arts_gui.py` (Modules tab, mock hardware UNCHECKED). Stop what
passed before starting the next item. Real arm: the operator holds the pendant and the e-stop,
and stops the pendant program BEFORE the driver.

## H1 — cameras + sync box (10 min)

1. `lsusb | grep -c "Blackfly S"` → 2 (a late camera kills the camera container).
2. GUI preset **Stereo + sync** (cameras + hardware trigger + sync agent).
3. Check: `/ecm/stereo_left/image_raw` and `/ecm/stereo_right/image_raw` at 30 Hz;
   `/ecm/stereo_sync/pulse` at 30 Hz with `exposure_delay_ticks` filled (not 4294967295) for
   both cameras; `/ecm/stereo_{left,right}/meta` present.
4. Pipeline check: stop the cameras; tick **record by pedal**; start; press the Savant right
   pedal (record_toggle) twice ~10 s apart; the bag under `~/arts_bags/images_*` holds both
   images, the meta topics, `/console/episode_mark` START/END and `/console/footswitch_teleop`.
5. Pass: 30 Hz both, strobes on every pulse, one START/END pair. Stop the cameras.

## H2 — head sensor + gamepad (5 min)

1. GUI: **GOOVIS head sensor**, **Gamepad (Xbox) jog**.
2. `ros2 topic echo /console/head_sensor` while the operator puts the GOOVIS on and takes it
   off: buttons `[1]` with the head in, `[0]` out, 20 Hz either way.
3. `ros2 topic echo /console/gamepad`: each stick, trigger, bumper and Back/Start/Guide
   moves the index the jog node expects (axes 0/1 left, 2 LT, 3/4 right, 5 RT, 6/7 d-pad;
   buttons 4 LB, 5 RB, 6 Back, 7 Start, 8 Guide).

## H3 — adapter discs, NO instrument fitted (10 min)

1. GUI: **PSM left adapter discs** (only). The log must say "torque on, velocity mode";
   a refusal about the operating mode means STOP (no EEPROM write today).
2. Gamepad: Back (psm_left), hold LB, left stick X gently: disc 1 turns, max 10 rpm.
   Note WHICH adapter turns. If it is the right arm's, swap the two ports in
   `arts_dv_tool_driver/config/disc_buses.yaml`.
3. Each disc in turn (left X = disc 1, left Y = disc 2, right Y = disc 4, d-pad X = disc 5);
   release LB → all stop. `/psm_left/tool_actuator_states` follows.
4. Stop; repeat for the right adapter.

## H4 — psm_left on the real arm (30–45 min)

1. Pendant: External Control program ready, not playing. GUI: **PSM left**; play the program.
2. First-motion regression (`docs/modules/ur_arms.md`), small and slow.
3. Tip preset: the RViz tip frame (`psm_left_tool0` + 0.609 m along +y, 88.5 mm out) should sit
   at the adapter's instrument-wrist position; note the discrepancy by eye.
4. RCM: `ros2 run arts_controllers set_rcm.py on --arm psm_left` (current shaft, 100 mm behind
   the wrist); `set_rcm.py status`.
5. Teleop (MTM left + footswitch from the GUI) with `/psm_left/rcm_state` recorded, as long as
   the operator can (30 min for the drift claim, S1); then `set_rcm.py off` and a short
   RCM-off control. Optionally a gamepad jog (RB) under the RCM.
6. Pendant program stop → GUI stop.
