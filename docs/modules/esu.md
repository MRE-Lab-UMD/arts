# Module: arts_esu — the ESU-TAP footswitch interposer

**Layer 2 (module doc): what the module is.** Status lives in `AGENTS.md` (Phase 4). Why it
is built this way: decision 18, amendment 2026-11-03.

## What it is

The ESU-TAP board (v2 on this cart, unit 0003) sits in the footswitch cables at the support
tower: monopolar pedal (Bovie BV-1253B) → board → ESU monopolar input, bipolar pedal
(BV-1254B) → board → ESU bipolar input, both on the back panel of the one DRE ASG-300. Each
channel has a bypass relay (the pedal straight to the ESU when the board is unpowered) and an
activation relay (the board closes the ESU-side contact for a pedal press or an accepted
software command). Relays on a 5 V barrel supply; the ESP32 on USB (P1, the patient-cart
hub, `docs/host_usb_topology.md`).

The firmware is its designer's, unmodified (`oem_drivers/esu_tap/VERSION`). ARTS adds:

| Artifact | Job |
|---|---|
| `scripts/esu_node.py` | board protocol 2 → the `/esu` contract (`docs/modules/ros_interfaces.md`); runs on `rmw_fastrtps_cpp` |
| `scripts/fake_esu_tap.py` | the board's ROS surface and control rules, for mock tests |
| `launch/esu.launch.py` | micro-ROS agent + esu node (`fake:=true`: fake board instead) |
| `test/esu_mock_test.py` | 20 checks, node + fake on Fast DDS, rig on CycloneDDS |
| `deploy/modules/esu.sh` | udev rule `/dev/arts_esu` by the CP2102N's serial number |

```bash
ros2 launch arts_esu esu.launch.py                 # board on /dev/arts_esu
ROS_DOMAIN_ID=73 python3 arts_ros2_ws/src/arts_embedded/arts_esu/test/esu_mock_test.py
```

Mock tests run on their own ROS domain: on the cart's domain, the fake's services and the
real board's would both answer the node.

## How energy behaves

- **Pedal:** active while held, off on release, handled on the board without ROS. ROS sees it
  in `/esu/state` (source footswitch).
- **Software:** `/esu/enable` (`true`, once, deliberately), then `/esu/target` held at >= 10 Hz
  while energy is wanted. The node renews the board every 100 ms; the board drops a command
  500 ms after its last renewal. A target older than 0.3 s, `activate=false`, a disable, two
  unanswered renewals in a row or ANY pedal press switch software energy off; a pedal press
  also clears enable, which must be given again. One software output at a time.
- **Routing:** monopolar → the arm whose `tool_info` says monopolar, bipolar → the arm whose
  says bipolar; ambiguous (both the same) is refused; `monopolar_arm`/`bipolar_arm` override.

## Results

The benchmark report for the paper: `docs/benchmarks/esu_tap_v2_20261103.md` (lease
503.1 ± 3.8 ms, n = 8; takeover 11 ms; bypass; link loss). The table below is the dated log.

| Date | What | Result |
|---|---|---|
| 2026-11-03 | Board identity over `/esu/capabilities` | protocol 2, firmware `3.2.0-bench`, v2, 3 channels, pedal build, watchdog, lease 500 / refresh 100 ms, boot id 15, 0 dropped. The designer confirmed that this reply means the current firmware (the pinned commit's image) |
| 2026-11-03 | Pedals through the board, ESU off (cables to the ESU connected), read-only | 6 presses (CUT, COAG, bipolar, twice): all 12 edges, source footswitch, right order |
| 2026-11-03 | Latency of a board message on the PC | agent receipt → DDS write 0.13–0.38 ms; → Fast DDS listener 0.1–1.1 ms; → CycloneDDS listener 0.1–1.1 ms (17 edges, 5 presses after > 30 s idle, 3 taps of 0.21–0.29 s) |
| 2026-11-03 | First message to a late-joining CycloneDDS reader | 3.0 s late (twice on the board, a 2.54 s press arriving with its release; reproduced in software); the board and Fast DDS readers on time. Not the board, as first read that morning. Cause: Fast DDS's 3 s writer heartbeat across vendors (decision 18 amendment) |
| 2026-11-03 | Same, with `config/fastdds_esu.xml` (100 ms heartbeat) | `/esu/state` (esu node) 0.101 s late, stamp correct; raw `/esu/activation_state` still 3.0 s (the agent ignores the profile): energy timing comes from `/esu/state` stamps |
| 2026-11-03 | CycloneDDS → board service | no answer (Fast DDS: immediate); the reason for the node's RMW |
| 2026-11-03 | Software → ESU-side contact, meter, ESU unplugged | CUT on plug pins 1–2 for the commanded 30 s, then opened by the lease 0.506 s after the last renewal (board report); COAG and bipolar not confirmed by meter (battery died) |
| 2026-11-03 | Pedal plugs, meter | BV-1253B: 1–2 CUT, 1–3 COAG, 1–4 always closed; BV-1254B: one pair switched, one pair always closed. Both always-closed links pass through the board to the ESU side (monopolar 1–4; the bipolar pair) |
| 2026-11-03 | **With the ESU on** (DRE ASG-300, 10 W CUT / COAG / bipolar, return pad connected, no instrument on any output) | A. software 2 s pulses: the ESU activated on CUT, COAG and bipolar, each 2.5 s (2 s renewed + the 500 ms lease); B. pedals through the powered board: all three activated, all edges logged; **C. board unpowered (barrel and USB out): all three pedals activated the ESU — the bypass works**; power restored with no pedal held, the board rejoined by itself (boot 16) |
| 2026-11-03 | Fault: surgeon takeover, ESU on (10 W, no instrument) | software CUT on; bipolar pedal pressed: CUT released as a takeover, bipolar on from the pedal 11 ms later (one control cycle, break-before-make), the next CUT renewals refused, bipolar off on release (2.5 s), board left disabled |
| 2026-11-03 | Fault: link loss, ESU on | software CUT on, the micro-ROS agent killed 4.06 s in: the ESU tone stopped by itself about 0.5 s later (operator, by ear); agent restarted: outputs open, board still disabled, no reboot (boot 16) |
| 2026-11-06 | **Two energy pedals, ESU on** (10 W on all three outputs, no instrument; all six orders of CUT / COAG / bipolar, each first held, the second added and released, `$ARTS_LOCAL/esu_combo_20261106/bag`) | the board reported both contacts closed every time; the ASG-300 delivers only the FIRST pedal pressed and ignores the second while the first is held; releasing the first while the second is still held gives fault **F6** and no output until all are released (PI, at the generator). Replayed through the display's model: F6 at each such release (5 of 6 orders; in the sixth the second pedal was released first), the first pedal delivered otherwise. Two pedals closed in one board message from rest: not measured |
| 2026-11-03 | `esu_mock_test.py` | 19/19 pass (20/20 after the 1 Hz snapshot check was added); the takeover check first failed on a test artefact (a blocking `ros2 param set` let the target go stale), the lease check first passed falsely (the node was not killed: `ros2 run` wrapper) — both fixed and re-run |

## Stopping energy

ARM HOLD does not reach the ESU, and the ESU has no external stop input to wire it to (unlike
the UR control box). Energy stops: pedal released; software energy 0.5 s after its last
renewal (the board's lease) or at once on disable, an OFF, or any ESU pedal press; and
everything at the ESU's own power switch.

## Operating it

`arts_bringup ... esu:=true`, or the GUI's ESU row (Modules tab). The Surgery mode tab's
Energy box shows the board's state (connected, software energy enabled, the active output and
its arm) from `/esu/state`, which the node publishes on every change and as a 1 Hz snapshot
(source SYNC), and has Enable / Disable buttons for software energy (`/esu/enable`). Checked
2026-11-03 on the real board, read-only: bringup found it (boot 16, disabled) and the GUI
(offscreen) showed "board connected | software energy disabled | active: none".

## In teleop recordings

The surgeon fires energy with the physical ESU pedals (PI, 2026-11-03); software energy stays
disabled in teleop. `run_teleop_real.sh` with `ARTS_ESU=1` starts the ESU and records
`/esu/state`, `device_state`, `command`, `info` and the raw `activation_state`. The energy
action label for learning is `/esu/state` (source footswitch) by its `header.stamp`; the raw
topic's bag time can be up to 3 s late for the first event (decision 18, 2026-11-03).

**Contacts are not delivered energy (measured 2026-11-06, Results).** `/esu/state` reports the
contacts the board drives; with two energy pedals down, the ASG-300 delivers only the first
one pressed, and after an F6 it delivers none until all are released. Energy timing from a
recording must apply that rule (the display's pedal panel does: `arts_display`), or an
ignored pedal is counted as energy delivered.

## Still to do

1. A `/esu/target` producer for policies and replay (none needed for teleop).
2. Not tested, stays a limitation: controller stall with an output on, sense and stuck-driver
   faults; USB replug with a pedal held is an operating rule (don't).
3. Paper: the ESU's stop is its power switch (no limitation sentence). The hung-controller
   limitation stands: the v2 watchdog drives only the bypass relays.
