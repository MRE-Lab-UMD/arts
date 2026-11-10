# FLIR composed-container A/B and skew cross-check — 2026-08-21

**Doc layer 4 of 4 — one measurement session, immutable once committed.** Authoritative
status: `AGENTS.md` (FLIR Stage 5). Module doc: `docs/modules/pilot_flir_camera.md`. Bench
rig: `docs/modules/camera_sync_wiring.md`. Compares against
`camera_sync_skew_breadboard_20260819.md`.

## Question this session answers

Does moving both cameras into one `ComposableNodeContainer` (a) eliminate the correlated
12–15 Hz dropouts logged at rung 5e, and (b) perturb the sync-box skew measurement?

**Answers: (a) inconclusive — the dropouts did not reproduce in EITHER layout; (b) no —
the skew figures through the composed container match the breadboard baseline to within
sample-to-sample variation.**

## Setup

- Development laptop (`devlaptop-6c12t`), Ubuntu 24.04, kernel 6.8.1-1057-realtime,
  bare metal, CycloneDDS.
- 2× Blackfly S BFS-U3-63S4C, full resolution 3072×2048 BayerRG8, `ExposureTime` 5004 µs,
  hardware-triggered at 30 Hz by the `arts_camera_sync` breadboard rig (micro-ROS firmware,
  agent running, `/arts_camera_sync/pulse` at 29.985 Hz).
- Topology as decided for bring-up: left 24057131 on Bus 002 (PCH, direct USB-A), right
  24057132 on Bus 006 (Thunderbolt dongle), verified by `lsusb -t` before launching.
- No image subscribers during the A/B runs — same condition under which the 5e dropouts
  were originally observed.
- Launch: `arts_driver cameras.launch.py trigger:=true` with `composed:=false` / `:=true`.

## A/B result: acquisition rate, 5 minutes per layout

Per-camera `IN` rate from the driver's 5 s status lines, first (partial) window discarded:

| Layout | Camera | Windows | Mean | Min | Max | Windows < 28 Hz |
|---|---|---|---|---|---|---|
| two processes | left | 60 | 30.00 | 30.00 | 30.00 | 0 |
| two processes | right | 60 | 30.00 | 29.80 | 30.20 | 0 |
| composed | left | 60 | 30.00 | 30.00 | 30.00 | 0 |
| composed | right | 60 | 30.00 | 30.00 | 30.00 | 0 |

**The rung 5e dropout phenomenon did not occur in the control arm**, so this session
cannot discriminate the shared-SDK hypothesis. What it does establish: the composed
container runs both cameras at full resolution and full rate on real hardware (previously
verified only without cameras), and the two-process layout is also currently healthy.
`composed:=false` therefore STAYS — its diagnostic value is exactly a future session in
which the dropouts reappear. The plan to delete it after "the comparison is a committed
benchmark" assumed the defect would reproduce; it did not.

## Meta-topic recording under the composed layout

34.5 s MCAP recorded during a composed run — this is the empirical check on the corrected
claim that composing renames no topic:

| Topic | Type | Count |
|---|---|---|
| `/arts_camera_sync/pulse` | `arts_interfaces/CameraSyncPulse` | 1033 |
| `/camera_left/meta` | `flir_camera_msgs/ImageMetaData` | 1036 |
| `/camera_right/meta` | `flir_camera_msgs/ImageMetaData` | 1036 |

Same topic names as the two-process layout, recorded by the same `ros2 bag record`
invocation as the 2026-08-19 session, no changes needed.

## Hardware skew through the composed container, vs the breadboard baseline

From the pulse messages in that bag (ticks cast to signed before differencing;
12.5 ns/tick read back from the message itself):

| | two processes, 2026-08-19 | composed, this session |
|---|---|---|
| pulses paired | 1001 | 1032 |
| skew \|cam0−cam1\| mean | 3.52 µs | 3.40 µs |
| skew max | 10.44 µs | 10.19 µs |
| systematic (difference of means) | 2.81 µs | 2.63 µs |
| cam0 delay / width | 712.15 / 4977.00 µs | 711.68 / 4977.26 µs |
| cam1 delay / width | 709.33 / 4976.32 µs | 709.06 / 4976.53 µs |
| trigger period spread | 0.000 µs | 0.000 µs |
| missed strobes | 3 of 1004 (cam0) | 1 of 1033 (cam0) |

Every figure agrees to within the sample-to-sample variation already seen between the
SELFTEST console and the 08-19 bag. Expected — the skew path never touches the ROS process
model — but now measured rather than argued.

## What this session does NOT establish

- Not the committed skew baseline: still the breadboard rig on the development laptop.
  Per decision 14 that baseline comes from the fabricated PCB, on the production
  workstation.
- Not a verdict on the shared-SDK dropout hypothesis: the defect is currently
  non-reproducible. If dropouts reappear in any composed-mode session, re-run this A/B
  before anything else.
