# Module: arts_gui — the operator GUI

**Layer 2 (module doc): what the module is.** Status lives in `AGENTS.md` (Phase 4).

## What it is

`arts_utilities/arts_gui/scripts/arts_gui.py`, a Qt window (the ROS `python_qt_binding`,
PyQt5 on Jazzy: no new dependency) that starts and stops ARTS modules one at a time:

```bash
source ~/arts/arts_ros2_ws/install/setup.bash     # the RUN rule; every module inherits it
ros2 run arts_gui arts_gui.py
```

It is a supervisor, never in a control or data path. Each module is its own
`ros2 launch arts_driver arts_bringup.launch.py <switch>:=true cart_tf:=false ...` process
group with the bringup's switch names (decision 18), so every click is reproducible from a
terminal: the exact command is the first line of the module's log
(`~/.ros/arts_gui/<module>.log`). `cart_tf` runs once, while any module runs.

- **Modules tab.** Patient cart: `psm_left`, `psm_right`, `ecm`. Endoscope: one camera
  process with options (left, right, hardware trigger from the sync box, record by pedal).
  Surgeon console: MTMs (left/right, one bridge), teleop footswitch, GOOVIS head sensor,
  gamepad jog. Support tower: ESU (disabled until ported). Presets start a set (PSM left,
  Left camera, Stereo + sync, ECM + stereo + sync, Teleop left, Teleop both). "Mock
  hardware" applies to the arms.
- **Displays tab.** GOOVIS side-by-side and tower views, each on a chosen output (the GOOVIS
  is preselected by its EDID name); image viewer for a single camera (left if running);
  RViz with a config generated from the running arms (one robot model each) and cameras.

## Rules it keeps

- The cameras, recorder and surgeon display are ONE process (decisions 5 and 14), so display
  and recording options apply when the cameras start ("Restart cameras with these settings").
  The surgeon display needs both cameras.
- Recording by pedal runs the Savant node inside the camera process (episode marks land in
  the image bag); the separate teleop-footswitch module is refused meanwhile (both would grab
  the device).
- Stopping a real arm asks first: stop the pendant program, then the driver.
- Closing the GUI leaves modules running; reopening re-attaches (`~/.ros/arts_gui/state.json`).

## Verified

2026-11-01, offscreen, mock arm: start psm_left (cart_tf follows), RViz config with only
`/psm_left/robot_description`, stop (exit 0; cart_tf stops after the last module). Not yet
used at the console with real hardware.

## Open

Status tab (joint states, rates, RCM, ARM HOLD, recorder), image overlays (recording, energy,
RCM, clutch, closest arm distance), single-camera surgeon view, the ESU and vision-cart rows.
