# Camera Sync Box Benchmark Report — stereo skew, measured two ways

**Date:** 2026-08-19 04:57 UTC
**Rung:** FLIR Stage 5e (`docs/modules/pilot_flir_camera.md`) — first measurement taken over
micro-ROS from a recorded bag rather than the firmware's own console.
**Sync box:** `arts_embedded/arts_camera_sync`, ESP32-D0WD-V3 rev 3.1, ESP-IDF v5.5.5,
firmware in `MICROROS` mode (app version `6b7c90f-dirty`).
**Cameras:** 2 &times; FLIR Blackfly S BFS-U3-63S4C, firmware 1801.0.9.0, serials 24057131
(cam0 / `camera_left`, USB Bus 002 / PCH) and 24057132 (cam1 / `camera_right`, USB Bus 006 /
Thunderbolt dongle).
**Bag:** 8056 pulses over 268 s, 2.6 MB MCAP (`/arts_camera_sync/pulse` +
`/camera_left/meta` + `/camera_right/meta`).

> **PRELIMINARY — NOT THE COMMITTED BASELINE.** Taken on the breadboard bench rig and on the
> development laptop. Per AGENTS.md decision 14, no committed skew baseline may come from this
> rig: resistor instances, lead lengths and contact resistance move the systematic term by the
> same order as the 2.79 &micro;s systematic measured here. Re-take on the fabricated PCB, on
> the production workstation. What this run does establish is that the *measurement chain* is
> sound end to end, which is what rung 5e is for.

## Configuration

| Setting | Value |
|---------|-------|
| Trigger rate | 30 Hz (MCPWM, 1 MHz timer &rarr; 30.0003 Hz exact) |
| Trigger pulse width | 100 &micro;s |
| Capture clock | 80 MHz APB, 12.500 ns/tick (read back from the driver, not assumed) |
| Capture channels | 3 of 3 in MCPWM group 0: 2 strobes + trigger loopback |
| Camera trigger input | Line 0, opto-isolated, 560 &Omega; series |
| Camera strobe output | Line 1, `ExposureActive`, 1 k&Omega; pull-up |
| Strobe polarity | active HIGH at the pin (measured, rung 5d) |
| `TriggerDelay` | 379 &micro;s, explicit and identical on both cameras (model floor) |
| `ExposureTime` | 5000 &micro;s commanded, `ExposureAuto` off |
| `TriggerOverlap` | `ReadOut` |
| Transport | micro-ROS custom UART, UART0 at 115200 baud, agent v2.4.3 |
| Host middleware | CycloneDDS (`rmw_cyclonedds_cpp`) |

## Trigger

| Metric | Value |
|--------|-------|
| Period | 33333.000 &micro;s, min = max, spread **0.000 &micro;s** |
| Pulses recorded | 8056 |
| Pulse-index gaps | **0** (index span 8056) |
| `time_synced` | true on every message |

Spread below one 12.5 ns tick is the instrument floor, not a measurement. The 0.33 &micro;s
shortfall against an ideal 33333.33 &micro;s is quantization of an integer period register at a
1 MHz timer, identical on every pulse, and cancels out of every difference.

## (a) Hardware — two `ExposureActive` captures on one 80 MHz counter

| Metric | cam0 (24057131) | cam1 (24057132) |
|--------|-----------------|-----------------|
| Trigger &rarr; exposure start | 712.107 &plusmn; 2.262 &micro;s | 709.318 &plusmn; 2.266 &micro;s |
| Range | [707.612, 716.550] &micro;s | [704.950, 713.750] &micro;s |
| Exposure width | 4977.022 &plusmn; 0.043 &micro;s | 4976.338 &plusmn; 0.044 &micro;s |
| Strobes missing | 29 / 8056 (0.36%) | 9 / 8056 (0.11%) |

**Skew (cam0 &minus; cam1), signed, n = 8018:**

| Metric | Value |
|--------|-------|
| Mean (systematic) | **+2.791 &micro;s** |
| Std (random) | **3.188 &micro;s** |
| Range | [&minus;5.562, +11.088] &micro;s |
| Worst case &#124;skew&#124; | **11.088 &micro;s** |

Against the AGENTS.md target of < 1 ms and decision 14's own expectation of < 100 &micro;s, so
the box beats its specification by a factor of ninety on the worst single pulse.

The skew is signed deliberately. The sign says which camera leads and is the systematic term;
averaging absolute values would destroy it and inflate the mean by the random spread. The
2.791 &micro;s mean reproduces the 2.63 &micro;s measured from the firmware console at rung 5e,
and 0.684 &micro;s of it is directly attributable to the strobe side, because that is exactly
how much the two exposure WIDTHS differ.

## (b) Software cross-check — the cameras' own `camera_time` device clocks

| Metric | Value |
|--------|-------|
| Paired frames | 8090 |
| Raw delta spread | 615.280 &micro;s |
| Delta drift | &minus;2.267 &micro;s/s (&minus;2.27 ppm between the two camera crystals) |
| **Detrended residual std** | **3.223 &micro;s** |

The absolute delta is meaningless — two device clocks with unrelated epochs — and the raw
spread is dominated by the crystals drifting apart over 268 s, which is why it is detrended
before comparison.

**THIS IS THE RUNG 5e ACCEPTANCE CRITERION AND IT PASSES: 3.223 &micro;s against the
hardware's 3.188 &micro;s, a 1.1% difference between two entirely disjoint measurement
paths.** One is an interval on a single ESP32 counter; the other is a subtraction of two
timestamps minted inside the cameras and carried through Spinnaker, the ROS driver, CycloneDDS
and MCAP. Nothing is shared between them except the physics, so the agreement is evidence
that the number is real rather than an artifact of either path.

Neither figure comes from `header.stamp`, per decision 5's amendment.

## Self-calibrated strobe rise delay

`ExposureTime` from chunk data minus the hardware-measured width isolates the optocoupler's
rise delay per camera, in band, without a separate experiment:

| Camera | Implied rise delay |
|--------|--------------------|
| `camera_left` (cam0) | 26.978 &micro;s |
| `camera_right` (cam1) | 27.662 &micro;s |

Both sit on the ~25 &micro;s fixed optocoupler turn-off floor identified at rung 5d, confirming
that the 1 k&Omega; pull-up has taken the resistor-dependent term down to a few microseconds.
This bias is inside every ABSOLUTE trigger-to-exposure figure and is common-mode, so it cancels
out of the skew.

## Capture tick &rarr; ROS time fit (decision 15, hop 3)

| Metric | Value |
|--------|-------|
| Crystal ratio error | 7.686 ppm (ESP32 against the host clock) |
| Residual std | 170.7 &micro;s |

This is the only ESTIMATED hop in the image-to-robot chain. The residual is an upper bound, not
the alignment uncertainty: it is dominated by micro-ROS transport over a 115200-baud UART plus
agent scheduling, both of which sit outside the measurement the fit exists to place on the ROS
timeline. Recovering the 7.7 ppm ratio matters for long recordings — left uncorrected it is
7.7 &micro;s of drift per second of episode.

## Notes and caveats

- **Missing strobes (29 and 9) are the camera not exposing, not a lost edge.** With both
  cameras streaming full resolution at 30 Hz the USB drivers periodically dip and recover
  (logged in AGENTS.md); a pulse with no exposure is the correct report. The sync measurement
  never touches USB.
- **`/arts_camera_sync/pulse` is BEST-EFFORT.** `ros2 topic echo` defaults to reliable and
  therefore prints nothing at all; pass `--qos-reliability best_effort`. `ros2 bag record`
  adapts to the offered QoS on its own and recorded all 8056 messages with zero index gaps.
- Recording a bag needs `flir_camera_msgs` on the path, or rosbag2 cannot resolve
  `ImageMetaData` and silently records no meta topic at all.

## Reproduce

```bash
# terminal 1 — micro-ROS agent
source $ARTS_LOCAL/middle_overlay_ros2_wss/oem_ros2_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/arts_camera_sync -b 115200

# terminal 2 — cameras, slaved to the box
source ~/arts/arts_ros2_ws/install/setup.bash     # needs flir_camera_msgs in the chain
ros2 launch arts_driver cameras.launch.py trigger:=true

# terminal 3 — record, then analyze
ros2 bag record -o sync_bag /arts_camera_sync/pulse /camera_left/meta /camera_right/meta
python3 ~/arts/arts_ros2_ws/src/arts_embedded/arts_camera_sync/bench/analyze_sync.py sync_bag
```
