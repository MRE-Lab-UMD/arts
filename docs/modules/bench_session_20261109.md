# Bench session 2026-11-09 — FLS peg transfer, RCM on, 3 experts + 3 novices

**Layer 3 (bench doc): do this at the bench, in order.** Status lives in `AGENTS.md`;
results go to `docs/benchmarks/` and a one-line headline beside the checkbox.

ARTS shell in every terminal:
`source /etc/profile.d/arts-dds.sh; source ~/arts/arts_ros2_ws/install/setup.bash`, and
`B=~/arts/arts_ros2_ws/src/arts_controllers/bench`. The supervisor holds the pendant and the
e-stop, gives the go for every motion, and stops the pendant programs BEFORE the bringup.

## The task (read this to each operator)

- **Board:** the FLS peg board in the trainer box, under the endoscope. Before each trial, 3
  **green** triangles sit on the **left** pegs and 3 **orange** triangles on the **right** pegs.
- **Green, one at a time:** lift with the LEFT instrument, hand it to the RIGHT instrument in the
  air, and place it on a free peg on the right.
- **Then orange, one at a time:** lift with the RIGHT instrument, hand it to the LEFT instrument
  in the air, and place it on a free peg on the left.
- A triangle dropped where you can still see it may be picked up again, by either instrument. A
  triangle dropped out of view is lost: leave it.
- Press the **record pedal** (right) when you are ready to start; press it again when the last
  triangle is placed. The supervisor stops the trial at **300 s** if it is still running.
- **Pedals:** the instruments follow your hands whenever your head is in the viewer. Left =
  clutch: hold it to let go of the instruments and reposition your hands, release to follow
  again (the surgical clutch); middle = camera (both hands move the endoscope; the instruments stop meanwhile);
  right = record. After a camera move, the instruments re-engage once your hands' orientation is
  within 10° of the instruments'.

## What is counted (no score; the behaviour is reported)

- **Time:** record pedal START to END (the episode marks in the state bag).
- **Transferred, of 6:** triangles resting on a peg on their target side at END.
- **Drop in view / out of view:** a triangle leaving both instruments before it rests on a peg,
  split by whether it stays in the endoscope view. A drop during a handoff is a drop.
- **Failed handoff:** a handoff that needs more than one attempt (the receiving grasp misses, or
  the triangle goes back to the passing instrument), without a drop.
- **From the bags** (`peg_transfer_analyze.py`): tip path length per instrument, time engaged,
  clutch engagements, camera moves, RCM deviation (median, p95, max), collision guard holds.

The supervisor tallies the four counts live on paper. Claude then checks them against the
endoscope video (`analysis/review.mp4`, which comes from the FLIR images, not the phone or Nikon)
and writes them into `annotation.yaml`. The phone and Nikon film the cart for the multimedia
submission only.

## Operators

E1–E3 experts, then N1–N3 novices; IDs only, no names in any file. Expert = an engineer with
2+ years of surgical robot teleoperation (mostly the dVRK or equivalent); novice = no prior
surgical robot teleoperation (PI, 2026-11-09).
Every operator gets the same familiarisation before the recorded trial: the briefing above,
then 5 min of free practice with the triangles, not recorded. One recorded trial per operator.

## S0 — once, before the first trial

1. **Disk:** `df -h ~/arts_local`. A trial writes about 5.7 GB per minute of images (up to
   ~30 GB at the 300 s cap) plus ~1 GB of state. Export each trial before the next.
2. **Trainer box:** box over the board, ports in the box's compliant top, light source on. Tape
   marks every 10 mm on each needle driver shaft from the wrist, and on the scope from its tip.
3. **Instruments** (README step 2, both arms): needle drivers OFF, homing
   (`tool:=needledriver`), ON + A, then visual zero (LB + B, straighten, LB + X). Both arms use
   `471006_large_needle_driver.yaml` (one file per instrument type).
4. **Terminal 1, arms** (no cameras here; they start per trial in terminal 2):
   ```bash
   ros2 launch arts_driver arts_bringup.launch.py \
       psm_left:=true psm_left_tool:=large_needle_driver psm_left_tool_actuator:=true \
       psm_right:=true psm_right_tool:=large_needle_driver psm_right_tool_actuator:=true wrist:=true \
       ecm:=true head_sensor:=true gamepad:=true rviz:=true use_fake_hardware:=false
   ```
   Play on all three pendants **(go)**.
5. **Ports and RCM** (README step 4), per arm, **(go)** for each jog: RCM off, jog the shaft
   through its port (hold RB), read `d` at the port from the tape, then
   ```bash
   python3 $B/set_rcm.py on --arm psm_left  --depth <d>   # d from the wrist pitch axis
   python3 $B/set_rcm.py on --arm psm_right --depth <d>
   python3 $B/set_rcm.py on --arm ecm       --depth <d>   # d from the scope tip
   python3 $B/set_rcm.py status --arm psm_left            # deviation and insertion, each arm
   ```
   Write the three `d` values in the session notes. The RCM stays on for all trials; if an arm's
   bringup restarts, set it again.

## Each trial (the test run first, then E1, E2, E3, N1, N2, N3)

1. Trial directory, e.g. `T=~/arts_local/peg_20261109/E1` (test run: `.../test`).
2. **Terminal 2, cameras + display + image recorder (paused):**
   ```bash
   ros2 launch arts_driver arts_bringup.launch.py cart_tf:=false \
       ecm_stereo:=true ecm_stereo_sync:=true \
       display:=true display_fullscreen:=true display_hmd_pos:=1920,0 display_tower_pos:=0,0 \
       image_bag:=$T/images image_bag_paused:=true use_fake_hardware:=false
   ```
   `ecm_stereo_sync:=true` is required: without the sync box's agent the box reboots every ~12 s
   and stops triggering (E9's images have 33 such gaps).
3. **Terminal 3, teleop + state bag** (10 min window; the trial itself is the record-pedal
   episode inside it):
   ```bash
   ARTS_TOOL=471006_large_needle_driver.yaml ARTS_RIGHT_TOOL=471006_large_needle_driver.yaml \
       ARTS_CAMERA=1 ARTS_CAMERA_ARGS="-p camera_max_offset:=0.02" \
       ~/arts/arts_ros2_ws/src/arts_utilities/arts_teleop/test/run_teleop_real.sh $T/teleop 10
   ```
4. Reset the board (3 green left, 3 orange right). The operator's head goes in, and the
   operator presses **record**, does the task, then presses **record** again. Start the phone
   or Nikon before the first record press. The supervisor tallies the counts and notes the
   time if the 300 s cap is reached.
5. Stop: Ctrl+C terminal 3 (the arms hold), then Ctrl+C terminal 2 (closes the image bag).
6. **Claude analyses** (about 2 min for a 5 min trial) and checks the counts against the video:
   ```bash
   python3 ~/arts/arts_ros2_ws/src/arts_utilities/arts_teleop/bench/peg_transfer_analyze.py $T
   ```
   Output in `$T/analysis/`: `report.md`, `result.json`, `start.png`, `end.png`,
   `contact.png`, `review.mp4`. Any `FAIL` in the data checks means the trial is repeated before
   the next operator, after the cause is found.
7. **Export** once the report says `Data OK: yes` and the counts are in `annotation.yaml`. The
   small results stay on the disk (`peg_20261109_results/`, a few tens of MB per trial); the
   bags go to the SSD, checked by hash, and only then are deleted locally:
   The destination must be an EXTERNAL drive (here the USB easystore, which mounts at
   `easystore1`); the internal Windows partition is on the same NVMe as `~/arts_local`, so a
   `--delete` export there would leave a single copy on the source disk.
   ```bash
   R=~/arts_local/peg_20261109_results/$(basename $T); mkdir -p $R
   cp -r $T/analysis $T/annotation.yaml $T/teleop/run_info.txt $R/
   D=/media/$USER/easystore1/2027_ISMR/peg_20261109; mkdir -p $D
   ~/arts/arts_ros2_ws/src/arts_utilities/arts_record/scripts/export_bag.sh $T $D/ --delete
   ```

## After the six trials

`python3 .../peg_transfer_analyze.py --summary ~/arts_local/peg_20261109_results` prints one
table over every trial. The committed report is
`docs/benchmarks/peg_transfer_20261109.md` (Claude writes it from the per-trial results).
