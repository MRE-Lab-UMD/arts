# UR arms (two UR5e tool arms, one UR5 CB3 camera arm)

> **Doc layer 2 of 4 — module overview.** What the arm module is and what every arm needs
> before it is used. Status belongs in AGENTS.md (Phase 1 exit, Phase 2). This file does not
> own an independent status list.
>
> | | Where | For |
> |---|---|---|
> | Decisions + status | AGENTS.md; decisions 1, 11, 12, 17 | why, and what is done |
> | **You are here** | this file | install, network, calibration, real time, speed limits |
> | Arms, IPs, ports | `arts_ros2_ws/src/arts_driver/config/ur_arms.yaml` | per-arm facts |
> | Driver pin | `middle_overlay_ros2_wss/ur_ros2_ws/ur_ros2.repos` | versions and why |

## Install (`deploy/modules/ur.sh`)

```
./deploy/modules/ur.sh host_deps       # sudo: static host IP, firewall, real-time rights
./deploy/modules/ur.sh overlay_build   # ur_ros2_ws at its pins, MoveIt excluded (decision 11)
./deploy/modules/ur.sh preflight       # read-only: host IP, ping, PolyScope version per arm
```

On each pendant: the External Control URCap installed, pointing at `host_ip` and that arm's
`script_sender_port` from `ur_arms.yaml`. Only one PC can hold `host_ip` on the arm network
at a time. The arm launch (`ur_arm.launch.py arm:=psm_left|psm_right|ecm`, decision 18) takes the arm's
IP and all four driver ports from `ur_arms.yaml`, and refuses a `robot_ip:=` that disagrees.

**Wired port must hold `host_ip`.** On the workstation the arm switch is on `enp131s0` with
the NetworkManager profile `MIS-ARTS` (manual, 192.168.2.105/24). A second profile on the
same port, `Internet` (DHCP, autoconnect priority 10), wins when it can, and the arms are
then unreachable: seen 2026-10-30 and again after the 2026-10-31 recabling (port on
192.168.1.2 from a router, all three arms down). Check with `nmcli -t -f DEVICE,CONNECTION
dev` (expect `enp131s0:MIS-ARTS`) or `ur.sh preflight`; fix with `nmcli con up MIS-ARTS`.
Since 2026-10-31 the `Internet` profile is gone: the operator re-made it into a second
`MIS-ARTS` (same manual address), so either profile now gives the arm network.

## Every arm runs with its own factory calibration — required

Each UR arm leaves the factory with measured kinematic parameters that differ from the
nominal model by up to several millimetres at the tool. The driver detects a mismatch at
start-up ("The calibration parameters of the connected robot don't match", seen on the right
UR5e 2026-10-29) but runs anyway, and every tool pose, RCM point and accuracy number is then
computed on the wrong arm. So no ARTS benchmark runs on nominal kinematics:

1. Extract once per arm (read-only; no program runs, nothing moves):
   `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<ip> target_filename:=<file>`
2. Store it as `arts_calibration/config/<arm>_kinematics.yaml` (decision 12: measured
   corrections live apart from CAD-nominal values).
3. The arm launches pass it as `kinematics_params_file` (`ur_arm.launch.py` selects it
   with `arm:=psm_right|psm_left|ecm`); the start-up mismatch error must be absent. Checked
   both ways on the right UR5e 2026-10-29: its own file, no error; the left arm's file, error.
   Re-extract after any arm service or controller-box swap. The driver's "Calibration
   checksum" log line prints the hash of the FILE it was given, not the arm's, so it proves
   nothing; the absence of the mismatch error is the check.

Driver 3.10.0 or later is required: 3.8.0's tool never connects, and 3.9.0 writes the link
rotations in a convention the URDF reads differently (`ur_ros2.repos`, BUMPED note).

## Real-time scheduling

`ur.sh host_deps` gives the `realtime` group rtprio 99 and unlimited memlock and adds the
user to it. It takes effect only in a NEW user session: on 2026-10-29 a GNOME log-out did not
restart the per-user `systemd --user` manager (kept alive by a remote editor session), so new
terminals still lacked the group; a reboot did it. Check with `id -nG | grep -w realtime` and
`ulimit -r` (99). With it, the controller_manager logs "Successful set up FIFO RT scheduling
policy with priority 50" and the UR client's RTDE thread "SCHED_FIFO OK, priority 99"
(right UR5e, 2026-10-29). The RT thread is not yet pinned to the isolated cores
(`isolcpus=2,4,6,7`, decision 8); E5 states which it used. Without it the controller_manager logs "Could not
enable FIFO RT scheduling policy" and the 500 Hz loop runs at normal priority; E5 is never
measured in that state.

## Speed limits: the pendant slider does NOT apply

ARTS streams joint positions, which the driver's URScript executes with `servoj` at a fixed
step time. The pendant speed slider scales only the driver's trajectory mode (used by
`scaled_joint_trajectory_controller`), not `servoj` streaming (`external_control.urscript`,
checked 2026-10-29). Speed is therefore bounded by:

- ARTS: the servo controller's Cartesian and joint limits (`arts_controllers/config/`), and
  the commanded path itself (`first_motion_real_arm.py --speed`);
- the robot: the pendant's Safety configuration (joint and tool speed limits, reduced mode)
  and protective stops, which apply in every mode.

For first motion of a new arm or a new controller, lower both: a slow `--speed` and a
reduced-mode tool speed limit on the pendant.

## First motion, right UR5e (2026-10-29)

Driver 3.10.0, the arm's own calibration, controller loop SCHED_FIFO 50 (not core-pinned),
pendant External Control, no reduced mode, operator on the e-stop.
`first_motion_real_arm.py --step 0.03 --speed 0.005`: held 10 s with no command (0.010 mm);
30 mm out and back along x, y, z at 5 mm/s, residuals 0.01–0.03 mm (measured by the
controller's own FK, so this shows tracking, not accuracy); back at start within 0.01 mm,
joints within 0.001°. Controller_manager overruns: one at start-up, none in ~5 min after it.

## First motion, left UR5e (2026-10-30)

Same check, `arm:=tool_arm_left` (now `psm_left`; its own calibration and ports from `ur_arms.yaml`):
held 10 s (0.022 mm); out and back along x, y, z at 5 mm/s, residuals 0.01–0.02 mm; back at
start within 0.02 mm, joints within 0.002°; one start-up overrun. The tool-arm bases are
mounted at an angle, so base axes run diagonally in the room: `first_motion_real_arm.py
--axes x` moves one axis at a time for an operator to name its direction. Seen from the
left Haply position: +x right-and-down, +y toward the operator, +z left-and-down (estimates;
`arts_teleop/config/teleop.yaml` operator_to_base).

## Command interface: speedj by default (2026-10-30)

The servo controller streams joint VELOCITIES by default (`command_interface:=velocity`, UR
`speedj`); `command_interface:=position` streams positions for `servoj`. Same scripted
shakes on the left arm: 4–9 ms from command to the measured joints with `speedj`, 34–40 ms
with `servoj`; a hard-killed driver left the arm still and the pendant program stopped by
itself ([`servoj_speedj_ab_20261030.md`](../benchmarks/servoj_speedj_ab_20261030.md)).
Whether the pendant speed slider scales `speedj` was not checked; the section above was
written for `servoj`.

## E5, right UR5e (2026-10-29)

10 min, loop pinned to isolated core 4, non-blocking read: 499.994 Hz, period
2000.00 ± 76.56 µs (3.83 % jitter), 0 missed cycles after start-up
([`e5_servo_loop_20261029.md`](../benchmarks/e5_servo_loop_20261029.md)). The bag lost 4 of
314 146 `/joint_states` messages in a separate recorder process: the state bag needs the same
treatment the image bag got (composed recorder or tuned QoS) before "zero dropped robot
samples" can be claimed.

