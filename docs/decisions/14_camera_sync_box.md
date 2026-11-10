# Decision 14 — ESP32 stereo sync box and hardware skew measurement

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

14. **Stereo camera sync is an ARTS-built ESP32 sync box, and skew is measured in HARDWARE
    (decided 2026-08-17).** USB3 Blackfly S cameras have no PTP, and per the decision-5
    amendment the synchronized driver's `header.stamp` is a smoothed estimate rather than a
    device clock, so neither the bus nor the driver can supply a trustworthy sync number.
    The generator is therefore first-party: `arts_embedded/arts_camera_sync`, an ESP32
    (DevKitC-32E / WROOM-32E) that fires all cameras from one hardware edge AND captures
    each camera's `ExposureActive` strobe back on an MCPWM capture channel at the 80 MHz
    group clock, i.e. 12.5 ns resolution.
    That feedback path is the whole reason to build the box rather than buy a pulse
    generator. L/R skew becomes a HARDWARE measurement — the difference between two
    capture timestamps on one counter — completely independent of ROS, Spinnaker, DDS and
    any clock estimator, which is a stronger number than dVRK-Si, MOPS or the open
    laparoscopic platform report. Each pulse is published over micro-ROS carrying pulse
    index, trigger instant and the per-camera trigger-to-exposure deltas, so the metric
    falls out of the recorded bag instead of needing a separate experiment. Fitting pulse
    arrivals against ROS time also restores what USB3's missing PTP took away: it puts the
    trigger instant on the same timeline as the robot data.
    Wire the OPTO-ISOLATED input (Line 0, pin 2) rather than the non-isolated lines, for
    two reasons. A monopolar ESU shares this cart, and isolation is the only thing between
    its ground currents and the camera front end. And pin 1 — which the upstream driver's
    own `follower_example.launch.py` uses as `trigger_source: Line3` — has a MAXIMUM
    INPUT-HIGH OF 3.6 V, so a 5 V drive there destroys the camera; that footgun is
    designed out rather than documented. The opto path costs 18 µs of propagation delay,
    which is common-mode to every camera and therefore cancels out of the skew entirely.
    Series resistor: NOT the obvious 1 kΩ. FLIR's own table gives 3.1 mA at 3.3 V through
    1 kΩ, BELOW the 3.5 mA minimum input current, so 1 kΩ is an intermittent trigger that
    looks like a software bug. Use 560 Ω (≈5.5 mA); 680 Ω is also fine, 330 Ω exceeds the
    7 mA ceiling. Full pinout, pin map and bring-up ladder in
    `docs/modules/pilot_flir_camera.md` Stage 5. FLIR's own warning applies and is not
    optional: CONNECT OPTO_GND BEFORE APPLYING ANY VOLTAGE TO A GPIO LINE. The bench
    procedure for someone who has not wired a GPIO before — resistor identification,
    breadboard/screw-terminal layout, meter checks, symptom table — is
    `docs/modules/camera_sync_wiring.md`. Both required values (560 Ω, 1 kΩ) are stock in
    the BOJACK 25-value kit on hand, so no series/parallel chaining is needed.
    THE BREADBOARD RIG IS BENCH-ONLY AND THE DELIVERABLE IS A PCB (decided 2026-08-18). A
    spring-clip contact on stranded wire fails intermittently, and intermittently here means
    a dropped strobe on some pulses with a plausible skew computed from the rest — the exact
    silent-wrong-number failure this box exists to eliminate; the cart's monopolar ESU also
    makes unshielded flying leads a bad trade against the isolation that motivated Line 0;
    and a breadboard photo is not a reproducible build instruction. Same form as `arts_esu`:
    two-layer board, DevKitC SOCKETED on headers (no reflow for a reproducer, two-minute swap
    for a dead board), passives on board, screw terminals for the vendor pigtails, loopback
    as a track, third camera footprinted. AMENDED 2026-08-27: the pigtail strain-relief
    clamp is a 3-D-printed case feature, not NPTH cable-tie pairs on the PCB. The four
    corner M3 holes are the board-to-case interface. The clamp itself is still required —
    a tugged screw is still a dropped strobe with a plausible skew — but it is a later
    enclosure task, not a layout constraint. Do not grow the board for ties. SEQUENCING, and it is a
    measurement argument not an aesthetic one: NO COMMITTED SKEW BASELINE MAY COME FROM THE
    BREADBOARD, because resistor instances, lead lengths and contact resistance move the
    systematic term by the same order as the 2.63 µs systematic offset measured at 5e. Finish
    5e on the rig, feed its findings into the schematic (1 kΩ not 10 kΩ, active-high strobe,
    the 379 µs `TriggerDelay` floor, strain relief — none of which were known a week ago),
    then fabricate and baseline on the board.
    PCB WORKFLOW, AND A REPRODUCIBILITY HOLE FOUND WHILE DEFINING IT (2026-08-19). "Same form
    as `arts_esu`" above is currently an unverifiable instruction: there are NO EDA files
    anywhere in this repo or in the v1.4 reference snapshot — `arts_esu` is a bare
    `package.xml` + `CMakeLists.txt` stub, and the ESU-TAP board that exists physically was
    designed outside the tree. So the first PCB action is not the camera sync box at all: it
    is RECOVERING THE ESU-TAP SOURCE (EDA project, Gerbers, BOM) INTO THE REPO. Three reasons
    it outranks the new board. A PCB whose only source is one person's laptop is not
    reproducible under any reading of this project's goal, and that is true today, before any
    new board exists. It leaves with whoever drew it. And it is the thing that tells us which
    toolchain and house style the sync box should match, which cannot be decided in the
    abstract.
    HARDWARE SOURCE LIVES IN THE PACKAGE, NOT IN A SHARED DRIVE:
    `arts_ros2_ws/src/arts_embedded/<pkg>/hardware/<revision>/` holds the EDA project and a
    `README.md` recording the board revision, the fab house and options ordered, and which
    firmware revisions it are known good with. Each revision is a SELF-CONTAINED directory: its
    symbols, footprints and 3-D models are vendored beside it and registered by project-relative
    path, so a clean clone opens with no user-specific library paths. TOOLCHAIN IS **KiCad 9**,
    which is not an arbitrary preference — its files are text so diffs and reviews work, and it
    is what the existing ESU-TAP board is drawn in (recovered 2026-08-19), so choosing anything
    else would mean maintaining two.
    FABRICATION EXPORTS ARE COMMITTED ONLY FOR A REVIEWED, FABRICATED REVISION, under
    `hardware/<revision>/production/`; day-to-day Gerbers, BOMs and placement files are
    gitignored. This CORRECTS a rule written into this file earlier on 2026-08-19 that said to
    commit a fab zip beside the source unconditionally — the ESU-TAP repo already had the better
    policy and it is adopted rather than overridden. Both properties are kept, and the ordering
    of the two clauses is the whole point: an export regenerated on every commit drifts out of
    step with the source SILENTLY, which is worse than one being absent, while a reader without
    KiCad must still be able to order the board that physically exists. Tying the export to a
    reviewed revision satisfies both, and it is the same argument that puts `.repos` manifests
    in the tree rather than build instructions in prose.
    DIVISION OF LABOUR: the design PACKAGE is authored here (netlist and connection table,
    BOM with distributor part numbers, layout and clearance constraints, fab notes, test
    procedure); the DRAWING is done by someone with EDA experience; the review before ordering
    happens here against the package. Do not accept a layout that has not been checked against
    the connection table pin by pin — DRC verifies geometry and says nothing about whether a
    footprint's pin 1 is the pin the schematic meant, which is the error class that produces a
    board that assembles cleanly and mismeasures.
    NAMES: collaborators are never named in tracked files, for the same reason the regional
    arm distributor is not (see `docs/bom/`) — this tree becomes the anonymous review capsule.
    Credit belongs in the paper's author list and acknowledgements, not in AGENTS.md.
    THE CAPTURE-CHANNEL BUDGET IS THE BINDING CONSTRAINT, not pins. Every capture — both
    strobes AND the trigger loopback — must share ONE capture timer for the differences to be
    exact, and MCPWM group 0 offers exactly three channels. Two cameras plus the loopback
    fills it. The third camera is provisioned in the pin map and in `CameraSyncPulse`, but
    activating it means `ARTS_SYNC_TRIGGER_LOOPBACK=n`, after which camera 0's strobe becomes
    the reference and absolute trigger-to-exposure latency is lost; inter-camera skew, the
    metric the paper reports, survives either way. `arts_sync_config.h` fails the BUILD on
    over-subscription rather than letting it surface at 3 a.m. on the bench. Moving the
    loopback to group 1 would free the slot and is deliberately not done — absolute latency
    would then be a difference across two independent counters, which is the class of caveat
    this whole design exists to remove.
    ONE NUMBER IS EASY TO GET WRONG AND HAS NO SYMPTOM: on the plain ESP32 the capture clock
    is APB at 80 MHz (12.5 ns/tick, 32-bit wrap every 53.7 s), NOT the 160 MHz that applies
    to the MCPWM timer clock and to the S3's capture source. That constant only scales
    `tick_period_ns`, so a wrong value silently doubles or halves every published skew with
    nothing else looking amiss. The firmware therefore READS THE RESOLUTION BACK from the
    driver and only falls back to a constant on ESP-IDF older than the getter (< 5.1).
    Toolchain: ESP-IDF is pinned at v5.5.5 in `oem_drivers/esp_idf/VERSION` and installed by
    `deploy/modules/camera_sync.sh`. `IDF_TOOLS_PATH` is FORCED to
    `$ARTS_LOCAL/oem_drivers/esp_idf_tools`, because Espressif's default drops ~1.5 GB of
    xtensa toolchain into `~/.espressif` — outside both ARTS trees, recorded by nothing, and
    invisible to anyone auditing a machine. `export.sh` reads that variable too, so sourcing
    it bare finds no toolchain; source the generated
    `$ARTS_LOCAL/oem_drivers/esp_idf_env.sh` instead, which sets it first. That module is the ONE exception to "every module's
    oem_install runs in a full install": the toolchain is ~2 GB and is needed to BUILD
    firmware, not to run a board already flashed, so it is gated behind
    `ARTS_ENABLE_ESP_IDF=1`. Its `host_deps` is NOT gated — that writes the udev rule giving
    the board the stable path `/dev/arts_camera_sync`, which the launch file defaults to.
    Never source ESP-IDF's `export.sh` from a shell rc file: it prepends its own Python venv
    to `PATH` and shadows the system Python for everything else in that shell, colcon
    included.
    EACH arts_embedded FUNCTION GETS ITS OWN BOARD. The sync box, the ESU tap and the tool
    driver are three separate ESP32s with three separate firmwares that never interact. The
    ESU is mentioned in this decision for ONE reason only — it shares the cart, so its ground
    currents are an argument for the OPTO-ISOLATED camera line — and for no other. Note the
    isolation choice stands on its own without that argument anyway: pin 1's absolute maximum
    input-high is 3.6 V, so anyone following upstream's `trigger_source: Line3` from a 5 V
    board destroys a camera.
