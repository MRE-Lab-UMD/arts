# Module pilot — FLIR camera — status narrative

> **Role: status narrative archive.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Checkboxes here are a SNAPSHOT and are NOT authoritative — status lives only in
> `AGENTS.md`. Narrative that duplicates a module doc is still to be de-duplicated
> into that module doc (see `docs/repo_structure_triage.md`). Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

- [x] Stage 0: host readiness — 2× Blackfly S BFS-U3-63S4C, USB3, serials 24057131 (L) /
      24057132 (R). udev rules + usbfs_memory_mb=1000 confirmed. SpinView_QT streams OK.
- [x] Stage 1: Spinnaker 4.2.0.88 mirrored + `install.sh` implemented; `Dockerfile.flir`
      built (`dpkg-deb -x` for non-interactive EULA); SpinView_QT streams live inside container
      (USB3, `--privileged`, `/dev/bus/usb` mount). Verified 2026-08-06.
- [x] Stage 2: `flir_ros2_ws` built as Docker layer; `arts_driver` camera bringup wired
      (cameras.launch.py, 40 fps, chunk data, adjust_timestamp). ROS driver streams at
      40.00 Hz / 0% drops. Host `rmem_max` fix required for CycloneDDS (see README.md).
      `header.stamp` confirmed: device clock timestamps (not host wallclock).
- [x] Stage 3: benchmark harness built (`arts_record/bench/` — template for all modules).
      10-min baseline: 39.99 Hz ± 0.00 (100%), 0 drops, 26.4 GB Zstd, CPU 126%/153% peak,
      756 MiB. Report + JSON at `docs/benchmarks/flir_bfs-u3-63s4c_20260806.*`.
- [x] Stage 4: `deploy/modules/flir.sh` rewritten with full implementations; Dockerfile.flir
      refactored to RUN the same functions; `arts_install.sh` wires module scripts.
      From-scratch rebuild reproduces baseline: 39.99 Hz, 0 drops (digest `sha256:b7618fec…`).
- [x] Stage 4b (Jazzy re-validation, 2026-08-10): rebuilt on Jazzy/24.04/PREEMPT_RT and
      re-benchmarked — 39.99 Hz ± 0.00, 0 drops, byte-identical acceptance result
      (`docs/benchmarks/flir_bfs-u3-63s4c_20260810.*`, digest `sha256:350845b4b901…`).
      Two harness bugs found and fixed while doing it, both of which produced wrong
      numbers rather than visible failures: `flir_host_deps` wrote usbfs_memory_mb to
      modprobe.d but never applied it to the running kernel (usbcore is built in, so it
      stayed at 16 MB until a reboot), and it only WARNED about missing udev rules that
      `dpkg-deb -x` bypasses — ARTS now writes them itself, else a fresh bare-metal install
      has root-only cameras while the privileged Docker path masks the problem. The stats
      collector polled `docker inspect`, which keeps succeeding for an EXITED container, so
      it never terminated and appended 0% rows that dragged the mean CPU down; it now polls
      `.State.Running`.
- [ ] Stage 5 — IN PROGRESS 2026-08-17, and it is NOT gated on the laparoscope rig. The
      hardware trigger can be built and the skew measured on the bare camera pair; only the
      final stereo optical calibration needs the scope. Scope revised by decisions 5 and
      14 after reading the pinned driver source. Substeps, in order:
  - [x] `arts_embedded/arts_camera_sync` ESP32 firmware WRITTEN 2026-08-17 — not yet
        compiled or flashed, so treat every number in it as unverified. One MCPWM generator
        signal fanned to three pads through the GPIO matrix (simultaneous edges; toggling
        three pins in software would not be) + capture of each `ExposureActive` return on
        one shared capture timer, into a lock-free ring the publish task drains.
        `arts_interfaces/CameraSyncPulse` carries pulse index, trigger instant, trigger
        period and per-camera trigger-to-exposure deltas. THREE RUN MODES exist because
        the micro-ROS build takes UART0 for its transport and therefore has no console:
        bring the hardware up under SELFTEST (prints statistics), then switch. Pin map
        provisions THREE cameras; see the capture-channel budget in decision 14 for what
        the third one costs.
  - [x] Toolchain plumbing DONE and RUN 2026-08-17: `oem_drivers/esp_idf/VERSION` (v5.5.5),
        `deploy/modules/camera_sync.sh` (host deps, dialout, udev →
        `/dev/arts_camera_sync`, ESP-IDF clone + toolchain, micro-ROS agent overlay),
        `oem_drivers/esp_idf/install.sh` wrapper. ESP-IDF v5.5.5 installed on this machine
        (685 MB clone in `$ARTS_LOCAL/oem_drivers/esp-idf-v5.5.5`, 1.5 GB of xtensa
        toolchain in `~/.espressif`).
        ORDERING BUG FOUND BY RUNNING IT: ESP-IDF's installer creates its private Python
        venv as its LAST step, AFTER fetching ~2 GB, and on 24.04 `python3 -m venv` is a
        non-functional stub until `python3.12-venv` is present (pulled by `python3-venv` in
        `host_deps`). Skipping host_deps therefore costs the whole download and then dies on
        the final line. `oem_install` now checks `ensurepip` up front and refuses in one
        second; it is idempotent, so the retry after fixing skipped the clone and the
        already-extracted toolchains.
        BUILD FIRMWARE IN A ROS-FREE SHELL. The venv has
        `include-system-site-packages = false` and is genuinely isolated from apt Python,
        but a venv still honours `PYTHONPATH`, and a sourced ROS overlay puts
        `/opt/ros/jazzy` plus every overlay's `site-packages` there. Harmless in the
        observed direction — the installer printed resolver complaints about `launch-ros`
        and `generate-parameter-library-py` having unmet deps, which is only Debian's apt
        Python packages lacking pip metadata, and nothing was written into any ROS tree —
        but the leak runs both ways, so use `env -i`.
  - [x] FIRST COMPILE CLEAN 2026-08-17, no errors and no warnings, in the default SELFTEST /
        2-camera / loopback-on / 30 Hz / 100 µs configuration. 202 KB binary, 81 % of the app
        partition free. So the ESP-IDF v5.x MCPWM API usage is confirmed to be real, and
        `mcpwm_capture_timer_get_resolution()` exists at this pin (the 80 MHz fallback is
        therefore dead code on v5.5.5, kept only for older IDF). NOT YET FLASHED, so every
        TIMING number in the firmware remains unverified — compiling proves the API, not the
        physics. `firmware/build/` and the generated `sdkconfig` are gitignored; only
        `sdkconfig.defaults*` are tracked.
  - [~] Wire it per `docs/modules/camera_sync_wiring.md` (bench guide: resistor
        identification, breadboard vs screw-terminal division of labour, meter checks,
        symptom table) and climb the 5a–5e ladder. Opto-isolated Line 0 in, 560 Ω per
        camera, `ExposureActive` out on Line 1 with 1 kΩ pull-up. OPTO_GND connects
        FIRST, always. Nothing on the green wire, ever.
        FIRST FLASH AND RUNG 5a PASSED 2026-08-18. ESP32-D0WD-V3 rev 3.1, flashed over
        `/dev/arts_camera_sync` (the udev symlink works), `BLINK` build boots clean and
        reaches its banner. Meter on GPIO 25 against GND read ~1.5–2.3 V fluctuating,
        i.e. the averaging case around the 1.65 V mean of a 50 % duty 3.3 V square wave —
        a pass, and the guide now says so explicitly because a steady 1.65 V reading is
        otherwise easy to misread as a dead pin. Timing numbers are STILL unverified;
        5b is the first rung that exercises MCPWM capture and prints real figures.
        TWO DOC DEFECTS FOUND BY DOING IT, both of which produce a plausible wrong
        conclusion rather than a visible error. (1) The guide said to remove the loopback
        jumper before 5c. That is backwards: with `ARTS_SYNC_TRIGGER_LOOPBACK=y`,
        `on_loopback_capture()` is the ONLY caller of `open_record()` in `sync_engine.c`,
        so the loopback edge is what opens every pulse record — pull the jumper and zero
        records reach the ring and SELFTEST reports "no captures" on every channel even
        with both cameras triggering correctly at 30 Hz. The jumper is PERMANENT wiring
        for the two-camera build, and it is now drawn in the node map as such.
        (2) The bench doc was ordered by topic rather than by time, so the safety rules
        and the pigtail colour-to-pin verification — both of which must precede any
        wiring — sat after the wiring instructions. Restructured into read-once /
        reference / do-in-order parts; the rungs are section 9.
        Board-label trap worth remembering: GPIO 36 and 39 are silkscreened `VP` and `VN`
        (SENSOR_VP/VN) on the DevKitC and every breakout for it, so no terminal is marked
        36 or 39 and the two aliases are adjacent to each other.
        FIRST REAL FIRMWARE BUG, found by the first SELFTEST boot 2026-08-18 and fixed:
        `mcpwm_capture_channel_register_event_callbacks()` returned `ESP_ERR_INVALID_ARG`
        with "user context not in internal RAM", panicking in a reboot loop. Cause:
        `CONFIG_MCPWM_ISR_CACHE_SAFE=y` makes the driver run `esp_ptr_internal()` on
        `user_ctx`, and `sync_engine.c` was passing the channel index as
        `(void *)(intptr_t)i` — the literal addresses 0/1/2, which are not RAM. The
        diagnostic was actively misleading in two ways: index 0 is NULL, which the driver
        waves through, so channel 0 registered and **channel 1 was the first to fail**,
        naming GPIO 35 in the error and making it look like a fault specific to the second
        strobe pin or its wiring. Fix is a `static int s_strobe_ctx[] = {0,1,2}` in DRAM,
        passing `&s_strobe_ctx[i]` and dereferencing in the ISR. It must be non-const: a
        `const` array lands in flash-mapped `.rodata`, which an `IRAM_ATTR` ISR must not
        touch. THE GENERAL LESSON, which applies to every other `arts_embedded` board:
        the integer-cast-as-user_ctx idiom is standard C and standard ESP-IDF practice, and
        it is silently incompatible with IRAM/cache-safe ISRs. Rebuilt clean, no warnings.
        Also renamed `CONFIG_MCPWM_ISR_IRAM_SAFE` → `CONFIG_MCPWM_ISR_CACHE_SAFE` in
        `sdkconfig.defaults` (renamed in ESP-IDF v5.5; the old name still works via the
        rename mechanism but warns on every configure).
        RUNG 5b PASSED 2026-08-18, and it delivered the first hardware timing numbers this
        project has. `capture running: 80000000 Hz (12.500 ns/tick)` — the decision-14
        capture-clock figure is now MEASURED on hardware rather than reasoned about, so the
        80 MHz-vs-160 MHz trap is closed and the published skew scale is correct. Over 418
        intervals: trigger period mean 33 333.000 µs, min = max, **spread 0.000 µs**, i.e.
        trigger jitter below one 12.5 ns tick, which is the instrument floor. 0 dropped
        records. Note 33 333.000 µs vs an ideal 33 333.33 µs is not error but quantization:
        the MCPWM timer is at 1 MHz so the period register is an integer and the true rate
        is 30.0003 Hz, identically on every pulse, cancelling out of every difference.
        FLOATING-INPUT ARTIFACT DISCOVERED HERE, and it is a keeper because it makes a
        broken wire look like a working one. With nothing attached to GPIO 34/35 the
        selftest did NOT report `no captures`; it reported `trigger->exposure` = exactly
        100.000 µs on both channels with 0.000 spread and a SKEW of exactly 0.000 µs.
        Mechanism: GPIO 34–39 are input-only with NO internal pull-ups, so an unwired strobe
        pin floats at high impedance and capacitively follows the trigger edges on the
        adjacent driven pads; the channels arm on the falling edge only, so they timestamp
        the END of the pulse and therefore report `CONFIG_ARTS_SYNC_PULSE_WIDTH_US`. Two
        floating pins coupling from one source agree perfectly, which manufactures a
        beautiful and completely fictitious 0.000 µs skew. THE PULL-UP IS WHAT MAKES
        THIS GO AWAY — it holds the line at 3.3 V so coupled charge cannot swing it — so the
        artifact is confined to rungs before 5d. Two consequences recorded in the bench doc:
        never quote a skew figure taken before the pull-ups are fitted, and at 5d the
        acceptance test is that the delay is NOT equal to the pulse width (a real reading is
        ~50–60 µs from the opto's 18 µs in + 36 µs out). A disconnected strobe reads a
        plausible number rather than `NO_CAPTURE`, which is exactly the failure shape this
        whole box exists to eliminate.
        BLOCKED AT 5c AS OF 2026-08-18, and NOT on anything at the bench. 5a/5b were the
        rungs needing a meter and both pass; every later rung needs the camera CONFIGURED
        (trigger mode on, `Line1` source = `ExposureActive`), which the ESP32 cannot do. On
        this machine nothing can currently talk to a Blackfly: Spinnaker is mirrored under
        `$ARTS_LOCAL/oem_drivers/spinnaker-4.2.0.88/` but NOT installed (no `/opt/spinnaker`,
        no `libSpinnaker` in `/usr/lib`), `flir_ros2_ws` has never been built bare-metal here
        (`$ARTS_LOCAL/middle_overlay_ros2_wss/` holds only `ur_ros2_ws`), and the Docker
        daemon is not running. This is the same gap already logged under the DDS entry in
        Phase 0 — the committed FLIR baselines were taken in Docker and the bare-metal path
        was never exercised on 24.04 — so closing it serves both items at once. Note
        SpinView_QT does not run on Noble, so the configuration path is the ROS driver or the
        Spinnaker CLI samples, not the GUI.
        RISK NOTE CORRECTED in the bench doc while here: it claimed a pigtail
        mis-identification "destroys a camera outright". Not true for ARTS's own hardware —
        with a 3.3 V drive through 560 Ω, pin 1's 3.6 V ceiling is not exceeded and no single
        mis-wire is destructive. The 5 V hazard is real but applies to 5 V boards. What a
        mis-identification actually costs is a non-functional trigger, or — the insidious one
        — tying brown (pin 6, camera power ground) to our ground, which shorts across the
        optocoupler and silently discards the ESU isolation that motivated Line 0.
        BARE-METAL FLIR INSTALL DONE 2026-08-18 to unblock 5c, closing the gap above.
        `flir_host_deps` and `flir_oem_install` ran clean (7 debs skipped as designed:
        SpinVideo/SpinView need jammy's ffmpeg ABI, plus docs). Spinnaker lands in
        `/opt/spinnaker`, NOT `/usr/lib` — `ldconfig` picks it up from there, and the CLI
        samples in `/opt/spinnaker/bin` include `Trigger`, `Trigger_QuickSpin`, `LogicBlock`
        and `StereoGPIO`, which give a ROS-independent way to exercise a hardware trigger.
        `flir_overlay_build` then FAILED TWICE, and both failures had ONE cause with a
        misleading face. First it died on `ament_cmake_clang_format`, then — after that was
        (wrongly) worked around — on `ffmpeg` "has no installation candidate". Neither
        package was the problem. `flir_oem_install` ended with an unconditional
        `rm -rf /var/lib/apt/lists/*`: correct image-shrinking in a Dockerfile, DESTRUCTIVE
        on bare metal where those lists belong to the whole machine. With the lists empty,
        apt truthfully reports that it cannot find perfectly installable packages, and the
        error surfaces in whatever step runs next rather than where the damage was done —
        two functions later, in rosdep. It compounded itself because the same function's
        `apt-get update 2>/dev/null || true` had NO sudo, so on a host it always failed,
        always silently, and never repaired the damage.
        Fixed: the cleanup is gated on `[ -f /.dockerenv ]`, and the update runs under
        `$SUDO` and warns loudly on failure. `dds.sh` already had the right shape — a
        separate `dds_image_install()` that only Dockerfile.base calls — and that is the
        pattern to copy; flir.sh branches rather than splitting because the deb extraction
        genuinely is shared. THE GENERAL LESSON, applying to every module script under
        decision 4: A FUNCTION CONSUMED BY BOTH PATHS MUST CONTAIN NO CONTAINER
        HOUSEKEEPING. Sharing the function is what stops the two paths drifting, and it is
        also exactly what lets a Dockerfile idiom reach into the host.
        A WRONG DIAGNOSIS WORTH RECORDING, because the reasoning was superficially sound and
        the same trap is easy to re-enter. The clang-format failure was first attributed to
        the rolling-repo hazard — rosdep only mints a `ros-jazzy-*` key for a package
        rosdistro lists as released, `apt-cache` showed 43 sibling `ros-jazzy-ament-*`
        packages and no clang-format entry at all, so "released on paper, absent from the
        archive" fit, and even explained why the 2026-08-10 Docker build of the identical
        function had succeeded. It was wrong. `apt-cache` reads the BINARY CACHE under
        `/var/cache/apt/`, which survives deletion of the lists, so it answers plausibly
        while apt itself cannot resolve anything; with lists restored,
        `ros-jazzy-ament-cmake-clang-format` is present at 0.17.5 and installs normally.
        `--skip-keys ament_cmake_clang_format` and `-DBUILD_TESTING=OFF` were added and then
        REVERTED, because carrying a workaround for a phantom is worse than the phantom.
        Rule for next time: `apt-cache` is not evidence about repository contents unless
        `ls /var/lib/apt/lists/*Packages*` is non-empty.
        BUILD RESULT: 4 packages in 43 s — `flir_camera_description`, `flir_camera_msgs`,
        `spinnaker_camera_driver`, `spinnaker_synchronized_camera_driver`. CONFIRMED a second
        time after the workaround was reverted, so the clean build is attributable to the
        apt-lists fix alone and `ament_cmake_clang_format` installs and configures normally.
        rosdep pulls `ffmpeg` and `pocketsphinx-en-us` (~100 MB) onto the host as declared
        dependencies; noted because it surprises, not because it is wrong.
        VERIFIED on the installed overlay: all 4 packages present; `/opt/ros/jazzy` appears
        EXACTLY ONCE in `AMENT_PREFIX_PATH` and last, so the chain is correct; and both
        `camera_driver_node` and `synchronized_camera_driver_node` resolve every shared
        library, with `libSpinnaker.so.4` and the GenICam set coming from `/opt/spinnaker/lib`.
        That last check matters because a Spinnaker that links at build time and fails to load
        at runtime would otherwise only reveal itself when a camera is plugged in.
        `spinnaker_synchronized_camera_driver` ships `follower_example.launch.py`,
        `master_example.launch.py` and `primary_secondary.launch.py` — the first is the
        starting point for the Stage 5 trigger configuration.
        THIRD FINDING, and the one with the longest reach: UPSTREAM SILENTLY DEFEATS THE
        SPINNAKER PIN. `spinnaker_camera_driver/CMakeLists.txt` calls
        `find_package(SPINNAKER QUIET)` and, when that fails, `include(DownloadSpinnaker.cmake)`,
        whose script maps the OS codename to a version — noble → **4.4.0.99** — and fetches it
        from `https://eventvisionresearch.com/`, a THIRD-PARTY HOST, not FLIR. So a machine that
        skips `flir_oem_install` does not fail: it quietly builds against a different Spinnaker
        than `oem_drivers/spinnaker/VERSION` declares, bypassing both the pin and the account
        wall, and every benchmark taken afterwards describes a stack nobody chose. Same failure
        shape as the Ruckig incident — a pin defeated by something arriving through a back door
        — and the same remedy: fail loudly. `flir_overlay_build` now has a PREFLIGHT that
        refuses to run unless `/opt/spinnaker/lib/libSpinnaker.so` and
        `/opt/spinnaker/include/Spinnaker.h` both exist, naming the auto-download in the error
        so nobody re-learns this. Note this is why "the driver built fine" is NOT evidence that
        the OEM install worked.
        5c UNBLOCKED — BOTH CAMERAS LIVE ON BARE METAL 2026-08-18, first time on 24.04.
        `Enumeration` reports Spinnaker 4.2.0.88, 2 cameras on USB Interface 0; `Acquisition`
        grabbed 10 full-resolution 3072×2048 frames from EACH (`DeviceCurrentSpeed:
        SuperSpeed`, `DeviceVersion 1801.0.9.0`). Caveat: that sample opens the cameras
        SEQUENTIALLY, so it proves the data path, not simultaneous bandwidth.
        The pair IS split across two controllers: 24057131 (L) on Bus 002 (PCH, direct USB-A)
        and 24057132 (R) on Bus 006 via the USB-C dongle, both at 5000M. So the dongle does
        work — the earlier `lsusb -t` showing its hub with no camera beneath it was a
        transient, not a power limit. That is the bandwidth wall largely defused on this
        machine without dropping the frame rate.
        SERIAL FORMAT TRAP: sysfs reports the serial in HEX and Spinnaker in DECIMAL, so
        `/sys/.../serial` = `016F152B` is the same camera as `DeviceSerialNumber 24057131`
        (0x016F152B = 24057131, and `016F152C` = 24057132). Anyone matching udev output
        against a driver config will otherwise conclude they are looking at different cameras.
        SIDE EFFECT WORTH ACTING ON: that same table is upstream's statement that FLIR ships
        **4.4.0.99 for 24.04**, whereas ARTS pins 4.2.0.88, which officially supports 20.04 and
        22.04 only and is the reason SpinView_QT and libSpinVideo are broken on noble. A bump to
        4.4.0.99 may simply retire that whole limitation. It is a benchmarked act under the
        pinning policy (bump VERSION, re-mirror from FLIR's site, rebuild, re-run the FLIR
        benchmark, commit the new baseline beside the old) and it is NOT on the Stage 5 critical
        path, so it is recorded here rather than done mid-bring-up.
        DECIDED 2026-08-19 FOR THE PRODUCTION WORKSTATION BUILD-OUT: **install the pin as
        recorded (4.2.0.88) FIRST, then bump as a separate act.** A fresh machine is the only
        real test of the written recipe, and it is worth exactly one clean run — bumping the
        SDK in the same pass changes two variables at once, so any regression could not be
        attributed to either, and the existing baselines would lose their comparison. Order:
        `arts_install.sh` at 4.2.0.88 → reproduce the committed 39.99 Hz FLIR baseline on the
        new host → then bump to 4.4.0.99 as its own change with its own baseline beside the
        old one. The only thing 4.2.0.88 costs on 24.04 is SpinView_QT and libSpinVideo, and
        neither is on any critical path (the CLI samples in `/opt/spinnaker/bin` cover
        enumeration, acquisition and trigger checks).
        Also note `Failed to unshare namespaces: EINVAL` reproduced here (the open host
        issue in docs/host_prerequisites.md), so it is not as non-reproducible as logged.
      - [x] WRITTEN 2026-08-18, not yet run on hardware:
            `arts_driver/config/blackfly_s_arts.yaml` (parameter definitions, deliberately
            ordered) and `cameras.launch.py` extended with `trigger:=false` default. The
            free-running path keeps upstream's `blackfly_s.yaml` untouched so the committed
            40 Hz baselines stay reproducible; triggered mode uses the ARTS file. Uses the
            SINGLE `spinnaker_camera_driver` — the synchronized driver is a later rung.
            Removed a dead line while there: `frame_rate_auto` was being set by
            `cameras.launch.py` but is NOT defined in upstream's file, and the driver skips
            parameters it has no definition for, so it never did anything. The rate was
            always set by `frame_rate_enable` + `frame_rate`.
            RUN ON HARDWARE 2026-08-18 AND **RUNG 5c PASSES**: camera 24057131 acquires at
            `IN: 30.00 Hz`, flat, matching the ESP32 firmware's 30 Hz exactly. So the whole
            configuration chain is validated end to end — including `line1_linesource`, the
            entry that exists only because upstream does not expose it, and the deliberate
            parameter ordering, which applied cleanly with `trigger_mode` last. The camera is
            now genuinely slaved to the sync box.
            THREE THINGS LEARNED, and the first is a correction to this file. `TriggerDelay`
            379 µs is the model's **FLOOR, not a default someone picked** — the camera rejects
            9 with `OutOfRangeException`, so upstream's `value >= 9` comment does not apply to
            BFS-U3-63S4C. That softens the decision-15 amendment's fake-skew warning: two
            untouched cameras both sit at the floor and therefore agree. Setting it explicitly
            and identically still matters, because it is per-camera and settable above the
            floor, and because the 379 µs is a term in every trigger-to-exposure budget.
            Second, `OUT: 0.00 Hz` ALONGSIDE a healthy `IN` IS NOT A FAULT: the driver only
            publishes when something is subscribed (`camera.cpp:827`), so read `IN` for
            acquisition health and subscribe before reading `OUT`. Third, a polarity test
            between `RisingEdge` and `FallingEdge` DOES NOT DISCRIMINATE — a 100 µs pulse
            presents one edge of each per period, so both give exactly 30 Hz. ARTS keeps
            `RisingEdge` because it anchors exposure to the instant the ESP32 loopback
            timestamps; `FallingEdge` would fold `CONFIG_ARTS_SYNC_PULSE_WIDTH_US` into every
            measured delay and move it whenever that width was retuned.
            Also added while there: `line2_selector` + `line2_v33enable` and an `enable_3v3`
            launch argument. Line 2 carries a switchable 3.3 V rail (factory OFF), and
            energizing it is the only STATIC way to confirm the pigtail's colour-to-pin map,
            which nothing had verified. 5c passing confirms it empirically instead — black
            really is the Line 0 opto input, blue really is OPTO_GND — so the diagnostic stays
            defaulted off, but it is the tool to reach for on the next unlabelled pigtail.
      - [x] **RUNG 5d PASSED 2026-08-18, and it answers decision 15's `ExposureActive`
            question by measurement: 729.5 µs, i.e. FIRST-ROW assertion (*Any pixel*).**
            The two candidates differed by three orders of magnitude — ~430 µs for first-row
            versus ~17.4 ms for last-row — so there is no interpretive latitude. Since the
            camera has no `ExposureActiveMode` node to set, the mode is now a MEASURED
            property of firmware 1801.0.9.0 rather than a configured one, which is the
            stronger claim. Stability: absolute delay spread 8.6 µs over 797 samples
            (mean 729.5, min 725.2, max 733.8); exposure width spread 0.5 µs.
            THE FIRST READING WAS WRONG IN A WAY WORTH KEEPING, because it is the exact
            failure shape this box exists to catch. `ARTS_SYNC_STROBE_ACTIVE_LOW` defaulted
            to `y` on the reasoning that an open-collector opto output pulls LOW when
            asserted. It does not here: the pin idles low against the pull-up and
            rises while exposing. So the firmware captured the falling edge and reported
            5687.7 µs — a stable, tight, plausible number that was actually
            TriggerDelay + exposure + slew, i.e. the exposure's END. Nothing about it looked
            wrong; it was caught only by noticing it tracked the exposure time.
            THE FIX IS STRUCTURAL, NOT A FLIPPED FLAG. The firmware now captures BOTH edges
            on the same channel (no extra capture channel, so the three-channel budget of
            decision 14 is untouched; ~30 extra interrupts per second per camera), and every
            record carries exposure START and exposure WIDTH. The width is not needed to date
            a frame — it is there so the polarity CANNOT be silently wrong again: it must
            equal the commanded exposure, and a mislabelled edge makes it come out as the
            trigger period minus the exposure instead. It also exposes a stuck `ExposureAuto`
            immediately, and an L-versus-R difference in it is a real stereo asymmetry rather
            than a timing artifact. `CameraSyncPulse` gains `exposure_width_ticks`.
            **THE STROBE PULL-UP IS 1 kΩ, NOT THE 10 kΩ THIS FILE AND THE BENCH GUIDE
            ORIGINALLY SPECIFIED.** That value was chosen on the same wrong premise as the
            polarity — if the asserted edge is the transistor pulling DOWN, the pull-up only
            governs a lazy recovery edge nobody times, so its value hardly matters. Since the
            line actually RISES at exposure start, the pull-up governs the one edge that
            dates the photons. Measured back to back on 2026-08-18:

            | | 10 kΩ | 1 kΩ |
            |---|---|---|
            | trigger→exposure | 729.5 µs | 711.7 µs |
            | exposure width (camera reports `ExposureTime` 5004 µs) | 4958.3 µs | 4977.2 µs |
            | implied strobe rise delay | 45.7 µs | 26.8 µs |
            | pulses with no strobe | 4–5 % | **0 of 240** |
            | width spread | 0.5 µs | 0.15 µs |

            The dropped strobes are what make this a defect rather than a refinement: a slow
            edge loitering near the input threshold was being missed outright several percent
            of the time, and the 10 kΩ run's "4–5 % no strobe" was initially suspected to be
            the camera skipping triggers. It was the pull-up. Both figures moved by the same
            ~18 µs, which is the signature of a start-edge delay (it adds to the delay and
            subtracts from the width) and confirms the model quantitatively.
            Do not go above 1 kΩ; below buys almost nothing. Fitting the two points gives
            ~2.1 µs per kΩ plus a FIXED ~25 µs that is the optocoupler's own turn-off time
            and is not resistor-tunable.
            THAT RESIDUAL IS SELF-CALIBRATING, WHICH IS THE USEFUL PART: the camera reports
            its true `ExposureTime` in chunk data on the `~/<cam>/meta` topic, so
            `ExposureTime − measured width` yields the rise delay PER CAMERA PER FRAME. The
            bias inside the absolute figure is therefore measurable in-band rather than
            assumed constant, and it is common-mode anyway so it cancels out of the L/R skew.
            Remaining jitter is 8.5 µs peak-to-peak on the absolute delay, which is the
            dominant term and comfortably inside the < 1 ms sync target.
      - [x] **RUNG 5e — FIRST HARDWARE SKEW MEASURED 2026-08-18: |cam0 − cam1| mean 3.4 µs,
            max 10.6 µs, min 0.000 µs, over 720 pulses, 0 dropped records.** Against a
            < 1 ms target and decision 14's own "expect < 100 µs", so the sync box beats its
            spec by a factor of thirty. Supporting figures: cam0 711.64 µs / width 4977.27 µs,
            cam1 709.01 µs / width 4976.55 µs, trigger period spread 0.000 µs.
            STILL PRELIMINARY — this is the SELFTEST console path, not the micro-ROS bag, so
            the 5e acceptance criterion (hardware skew agreeing with the `camera_time`
            software cross-check) is NOT yet discharged, and it was taken on the laptop
            rather than the production workstation.
            THE NUMBER DECOMPOSES, and the decomposition is the part worth publishing:
            - A SYSTEMATIC 2.63 µs, the difference of the two means. Of that, 0.72 µs is
              strobe-side and is directly attributable, because the exposure WIDTHS differ by
              exactly that much and the width difference isolates the rise-delay mismatch.
              The remaining ~1.9 µs is therefore upstream of the strobe — 560 Ω tolerance
              changing opto drive current, cable length, or camera-internal variation.
              Recording the widths is what makes that split possible at all.
            - A RANDOM ±8.6 µs per camera, and it is almost certainly THE SENSOR'S LINE
              PERIOD rather than noise in our chain: ~17 ms readout over 2048 rows is 8.3 µs
              per line, the camera can only begin exposing on a line boundary, and the
              observed 8.5–8.7 µs matches. Two consequences. It is IRREDUCIBLE at this ROI —
              no wiring or firmware change touches it — and shrinking the ROI vertically
              shortens it proportionally, the same lever that shortens the decision-15
              rolling-shutter stagger. It is also INDEPENDENT between cameras, which the data
              confirms: skew spread 10.6 µs sits between the ~0 of a common-mode source and
              the ~12 µs of two independent ±8.6 µs terms.
            Strobe capture is now 718/720 on cam0 and 722/722 on cam1; the two misses
            coincide with the USB acquisition dropouts below, i.e. the camera genuinely did
            not expose, which is the correct behaviour rather than a lost edge.
            USB NOTE, NOT ON THE SKEW PATH: with both cameras streaming full-res 30 Hz the
            drivers periodically drop to 12–15 Hz and recover, and they do so WITHIN 2 ms OF
            EACH OTHER despite being on separate controllers (verified `lsusb -t`: left on
            Bus 002/PCH, right on Bus 006/Thunderbolt dongle). Simultaneity across
            independent controllers rules out per-controller bandwidth and points at the host
            — ~378 MB/s aggregate DMA with no subscriber consuming it. Deliberately not
            chased here: it is the "both cameras, full resolution, full rate, zero drops"
            benchmark that this file already scopes to the production workstation, and the
            skew measurement never touches USB.
            CAUSE IDENTIFIED 2026-08-19, and it is not the host: ARTS was running the two
            cameras as TWO SEPARATE PROCESSES, which upstream explicitly does not support —
            "the FLIR Spinnaker does not support two programs accessing the Spinnaker SDK at
            the same time, even if two different cameras are accessed. Strange things happen,
            in particular with USB3 cameras." A shared SDK is the one resource spanning both
            controllers, so it fits the simultaneity that ruled out bandwidth. The fix is the
            single-container item in the Stage 5 list above; retest the dropouts after it
            rather than treating them as a workstation problem.
      - [x] **MICROROS FIRMWARE BUILT, FLASHED AND PUBLISHING 2026-08-19** (266 KB, 75 % of the
            app partition free), which is what turns the SELFTEST console numbers above into a
            recordable topic. `arts_interfaces` rebuilt first so `CameraSyncPulse` carries
            `exposure_width_ticks`; verified with `ros2 interface show`.
            FOUR TRAPS, and every one of them produces a misdirecting error rather than a
            useful one. They are the reason this took five build attempts, and they are all
            now either fixed in tracked files or documented in the package README.
            (1) `catkin_pkg` MISSING FROM THE ESP-IDF VENV. `micro_ros_espidf_component` ships
            no prebuilt archive — it cross-compiles ~57 ROS 2 packages with colcon into
            `libmicroros.a` — and ESP-IDF's venv is built with
            `include-system-site-packages = false`, so apt's `python3-catkin-pkg` is invisible
            to it. Note this REFINES rather than contradicts the "build firmware in a ROS-free
            shell" rule above: that rule is about not leaking `PYTHONPATH` from a sourced
            overlay, and it is not a claim that the build needs no ROS tooling. Fixed by
            `firmware/requirements.txt` (pinned, `empy` held below 4.0) installed into the venv
            — not into system Python, where it would collide with apt — by
            `camera_sync_oem_install`.
            (2) A CONFIG-GATED `REQUIRES` SILENTLY DOES NOTHING. `main/CMakeLists.txt` must
            depend on the micro-ROS component only in this mode, and the obvious
            `if(CONFIG_ARTS_SYNC_MODE_MICROROS)` never fires: ESP-IDF processes each
            component's CMakeLists TWICE and the early-expansion pass that collects `REQUIRES`
            runs before Kconfig is loaded — verified by printing the symbol, which came out
            `''` then `'y'`. There is no error; the dependency edge is simply absent, and the
            visible symptom is a missing `camera_sync_pulse.h` whose three existing copies the
            error helpfully lists, sending you to audit include paths and the `extra_packages`
            symlink. Now gated on the component's PRESENCE ON DISK, which both passes can see.
            (3) THE TRANSPORT IS COMPILED INTO `libmicroros.a`, AND `CONFIG_MICRO_ROS_ESP_UART_TRANSPORT`
            DOES NOT SELECT IT. That option only gates Kconfig sections; the real switch is
            `-DRMW_UXRCE_TRANSPORT`, which the component's `colcon.meta` hardcodes to `udp`.
            The library was therefore built for UDP while the firmware called
            `rmw_uros_set_custom_transport()`, whose prototype lives behind
            `RMW_UXRCE_TRANSPORT_CUSTOM` in the GENERATED `rmw_microxrcedds_c/config.h` — so
            the error was an implicitly-declared function, pointing at the call site rather
            than at the library's configuration. Fixed by a tracked
            `firmware/app-colcon.meta`; `main.c` now `#error`s if the two ever disagree again,
            and includes `rmw_microxrcedds_c/config.h` explicitly. Two corollaries: the archive
            does NOT rebuild on a meta change (`libmicroros.mk` skips when it exists, and
            `idf.py fullclean` does not touch it — delete `libmicroros.a`, `include/` and
            `micro_ros_src/{build,install}`), and the first build after fetching always needs a
            second invocation because the component globs `include/*` before `libmicroros` has
            created it.
            (4) THE UART TRANSPORT IS NOT IN THE COMPONENT. `esp32_serial_open/close/write/read`
            live in `examples/int32_publisher_custom_transport/main/`, which upstream expects
            each application to copy. ARTS compiles it from the pinned checkout instead, so
            `components/` stays gitignored and nothing third-party is vendored.
            One robustness fix taken while there, because the console is disabled in this build
            and a dead agent session would otherwise be invisible: a failed `rcl_publish` was
            being discarded, which would leave the board triggering cameras while publishing
            nothing for the rest of a recording. 300 consecutive failures now reboot, matching
            how `RCCHECK` already handles init failure.
            AGENT BUILT 2026-08-19, first time on this machine: `oem_ros2_ws` now exists
            (`micro_ros_agent` + `micro_ros_setup`, 58 s). Its `rosdep install` needs
            `clang-tidy`, `libncurses-dev` and `ros-jazzy-micro-ros-msgs` and therefore an
            interactive sudo, so it cannot run unattended. AUDIT THE TRANSITIVE PIN, which is the
            hazard `oem_ros2.repos` already flags: `micro_ros_agent` fetches
            Micro-XRCE-DDS-Agent and spdlog as CMake ExternalProjects, i.e. NOT through the
            manifest. This build resolved them to **agent `7362281` (Release v2.4.3)** and
            **spdlog `eb322062` (1.9.2)**. The agent and the ESP32 client
            (`firmware/components.repos`) must be bumped together — a mismatch fails as a session
            handshake error that reads like a wiring or baud-rate fault.
            **A FIFTH TRAP, AND THE WORST OF THEM, FOUND BY FLASHING 2026-08-19: THE FIRMWARE RAN
            CORRECTLY AND TRANSMITTED NOTHING, BECAUSE UPSTREAM'S TRANSPORT NEVER ROUTES ITS OWN
            PINS.** `esp32_serial_open()` calls
            `uart_set_pin(port, CONFIG_MICROROS_UART_TXD, CONFIG_MICROROS_UART_RXD, ...)` and
            both default to **-1 = `UART_PIN_NO_CHANGE`**, so the transport relies on somebody
            else having muxed UART0 onto GPIO 1/3. The console is what normally does that — and
            `sdkconfig.defaults.microros` deliberately DISABLES the console, because log text
            interleaved with XRCE framing corrupts the session. The two settings therefore
            interact: console off plus pins at -1 gives a client that runs, retries
            `CREATE_SESSION` ten times, fails `rclc_support_init`, reboots, and writes every byte
            into a UART that reaches no pad. Fixed by setting
            `CONFIG_MICROROS_UART_TXD=1` / `RXD=3` explicitly in `sdkconfig.defaults.microros`,
            which is better than re-enabling the console to get the mux as a side effect: the
            transport then owns its pins and survives any later console change.
            THE SYMPTOM NAMES NOTHING, which is why this needed a tool rather than thought. The
            agent just waits; `ros2 topic list` shows nothing; the board is silent. Zero bytes on
            the wire looks IDENTICAL for a failed transport open, a stack overflow during
            micro-ROS init, a hung handshake and an unrouted pin. The discriminator is
            `firmware/sdkconfig.defaults.microros_debug`, added for this: layer it third and the
            console comes back, at the cost of a session that cannot work. With it, the same
            binary immediately showed `XRCE` frames going out followed by
            `rclc_support_init ... failed: 1` — which proved the client, the stack size and the
            transport call were all fine and localized the fault to the pin mux in one run. Keep
            that overlay; it is the only debugger this mode has.
            One diagnostic worth reusing: with the console off the ROM banner is STILL printed on
            every reset, so counting banners over ten seconds distinguishes a reboot loop
            (repeated banners) from a hang or a mute-but-healthy app (exactly one). That is how
            the crash hypotheses were eliminated before the A/B.
            TWO RUNTIME TRAPS FOR ANY CONSUMER OF THIS TOPIC. `/arts_camera_sync/pulse` is
            published BEST-EFFORT, and `ros2 topic echo` requests reliable by default, so it
            prints NOTHING while `ros2 topic hz` reports a healthy 30 Hz — pass
            `--qos-reliability best_effort`. `ros2 bag record` adapts to the offered QoS by
            itself and needs no override. And recording the meta topics requires
            `flir_camera_msgs` ON THE PATH: with only `arts_ros2_ws` sourced, `ros2 topic info`
            shows a publisher while the type reads "invalid", and rosbag2 then records the pulse
            topic perfectly and NO meta topic at all, without erroring. CLOSED 2026-08-19 by
            rebuilding `arts_ros2_ws` against `ur_ros2_ws` + `flir_ros2_ws` + `oem_ros2_ws`;
            `/opt/ros/jazzy` still appears once and last. Keep it that way — a later rebuild
            that sources only UR will silently re-break meta recording.
            SIDE EFFECT OF FETCHING THAT COMPONENT, FOUND AND FIXED 2026-08-20:
            `arts_camera_sync/CMakeLists.txt` did `install(DIRECTORY launch firmware)`, and
            `firmware/` had by then grown a 509 MB fetched micro-ROS component plus a 135 MB
            ESP-IDF build tree — 644 MB of gitignored third-party content being copied into
            `install/` on every colcon build. It did not merely bloat: the copy FAILED
            outright, because the fetched checkouts carry read-only `.git/hooks` and CMake
            cannot set permissions on them, so `arts_camera_sync` failed and took two more
            packages down as "not processed". Fixed with `PATTERN ... EXCLUDE` entries
            mirroring `firmware/.gitignore`; build is now 1.1 s and 240 KB, and the installed
            firmware set is exactly the 14 tracked files. THE GENERAL LESSON for
            `arts_esu` and `arts_dv_tool_driver`, which will both grow a `firmware/` and will
            copy this pattern: an `install(DIRECTORY)` over a directory that a build or a
            fetch writes into is a defect waiting for the directory to fill up, and the
            exclusion list must be kept in step with that directory's `.gitignore`. Note the
            failure surfaced ~2 weeks after the line was written, which is why it was not
            caught when the package was created.
            **5e BAG CROSS-CHECK, BREADBOARD, 2026-08-19.** 34.5 s MCAP of pulse + both meta
            topics (no images): 1004 pulses, 1037 meta/side. Hardware, after casting ticks to
            a signed int — `uint32` subtraction wraps and manufactures an 11-second "skew"
            that is `2^32` ticks: **|cam0−cam1| mean 3.52 µs, max 10.44 µs, min 0.013 µs
            over 1001 paired pulses**, systematic 2.81 µs, cam0 delay 712.15 µs / width
            4977.00 µs, cam1 709.33 µs / 4976.32 µs, trigger period spread 0.000 µs, 3
            missed strobes on cam0. That is the same number as the SELFTEST console (3.4 /
            10.6 µs) to the resolution of a different sample. Software: `camera_time` period
            33332.8 µs either side (vs 33333.000 µs hardware), residual of (L−R) after
            removing the two-clock offset has std 22.8 µs and 87 µs peak-to-peak — larger
            than hardware, as it must be (independent crystals, pairing by `header.stamp`
            rather than `FrameID`, no `FrameID` on `ImageMetaData`). Both are << 1 ms.
            STILL NOT THE COMMITTED BASELINE: breadboard, laptop, single-camera driver not
            the synchronized one. Report: `docs/benchmarks/camera_sync_skew_breadboard_20260819.md`.
      - [ ] Original note, kept for the reasoning: write the ARTS-side DEFINITION file (superset of upstream
            `blackfly_s.yaml` adding `line1_linesource` → `DigitalIOControl/LineSource`, which
            upstream does not expose, so `ExposureActive` cannot otherwise be routed to Line 1).
            THE FILE'S ORDER IS LOAD-BEARING, traced through `camera.cpp` 2026-08-18:
            `line0..3_selector` ALL map to the same node `DigitalIOControl/LineSelector`
            because GenICam configures digital I/O through a selector, so `LineMode`/`LineSource`
            apply to whichever line is currently selected. The write order comes from the
            DEFINITION YAML, not the launch file — `readParameters()` registers the
            on-set-parameters callback (camera.cpp:438) BEFORE `createCameraParameters()`
            (camera.cpp:179), `readParameterDefinitionFile()` builds `parameterList_` in file
            order, and each declaration carrying a launch override fires `parameterChanged()`
            immediately, writing to the camera then and there. So `line1_linesource` MUST sit
            after `line1_selector`, and reordering that file is a functional change that
            silently reconfigures different physical pins.
      - [x] Move both cameras into ONE `ComposableNodeContainer` in
        `arts_driver/launch/cameras.launch.py`, keeping the SINGLE `spinnaker_camera_driver`
        (template: upstream `multiple_cameras.launch.py`; `use_intra_process_comms: True`;
        `trigger_source: Line0`, NOT the examples' `Line3`, whose 3.6 V input ceiling makes
        it a hardware footgun). SUPERSEDES the former "switch to
        `spinnaker_synchronized_camera_driver`" item, which is CANCELLED — see the decision-5
        amendment for why, and note this is also the leading fix for the correlated USB
        dropouts below, since two processes sharing the Spinnaker SDK is unsupported upstream.
        WRITTEN 2026-08-20, container machinery verified WITH NO CAMERAS ATTACHED (the
        container starts, loads `libcamera_driver.so`, instantiates
        `spinnaker_camera_driver::CameraDriver`, and `camera_left` reads
        `blackfly_s_arts.yaml` and reports Spinnaker 4.2.0.88 in-process). The DROPOUT FIX
        ITSELF IS UNVERIFIED — that needs both cameras streaming full-res, and the honest
        test is an A/B against the two-process layout, which is why `composed:=false` exists
        and why it should be deleted once that comparison is a committed benchmark.
        A/B RUN ON HARDWARE 2026-08-21, AND IT CAME OUT INCONCLUSIVE IN THE ONLY HONEST
        SENSE: **the dropouts did not reproduce in EITHER layout** — 5 minutes per arm,
        both cameras full-res hardware-triggered at 30 Hz, split across Bus 002/006 as at
        5e, no subscribers, every 5 s window at 30.00 Hz in both. So the composed container
        is now verified on real hardware at full rate, and the shared-SDK hypothesis
        remains a hypothesis because the defect it explains is currently non-reproducible.
        `composed:=false` therefore STAYS — the deletion plan assumed the control arm
        would exhibit the defect, and its diagnostic value is exactly a future session in
        which the dropouts reappear (re-run the A/B before anything else in that session).
        Same session, from a 34.5 s composed-mode bag: skew |cam0−cam1| mean 3.40 µs /
        max 10.19 µs over 1032 paired pulses, systematic 2.63 µs, period spread 0.000 µs —
        the breadboard baseline's numbers to within sample variation, so composition
        demonstrably does not perturb the measurement path. Report:
        `docs/benchmarks/flir_composed_ab_20260821.md`.
        TWO CORRECTIONS TO WHAT THIS FILE PREVIOUSLY ASSUMED, both found by doing it.
        (1) COMPOSING DOES NOT RENAME ANY TOPIC — see the corrected `~/<cam>/meta` item
        below. (2) A single-threaded `component_container` is the right choice, and not
        merely upstream's habit: each `Camera` spawns its own acquisition thread and calls
        `doPublish` on it (`camera.cpp:193`, `728`, `799`), so the container's executor
        carries only the 1 Hz diagnostics timers and `component_container_mt` would buy
        nothing.
        THE REAL COST OF COMPOSING, MEASURED 2026-08-20: **ONE ABSENT OR SLOW CAMERA NOW
        TAKES DOWN THE OTHER.** `Camera::openDevice` retries a HARDCODED 5 times at 1 s
        (`camera.cpp:279`, `retry < 6` — not a parameter) and then throws, and upstream then
        dies with **SIGSEGV** rather than exiting cleanly. The crash is upstream's, not
        composition's — the standalone node does exactly the same, verified side by side.
        What composition changes is the blast radius: `load_composable_nodes` loads
        SEQUENTIALLY, so the container died while loading `camera_left` and `camera_right`
        was never loaded at all, whereas two processes fail independently. The realistic
        trigger is not an unplugged camera but a LATE one — enumeration more than ~5 s after
        the container starts, or the USB-C dongle appearing without its camera beneath it,
        which has already happened on this laptop. Operationally: confirm both cameras in
        `lsusb` BEFORE launching, and read a dead container as "one was late" rather than as
        a driver fault. Deliberately NOT papered over with `respawn=True`: a genuinely
        missing camera would become an endless crash loop instead of one clear "giving up"
        line, and a mid-session respawn would restart the HEALTHY camera too, re-phasing its
        exposure so a skew baseline measured earlier in the recording would no longer
        describe the rest of it. The right fixes are upstream — make the retry budget a
        parameter, and do not segfault on a failed `configure()` — and they belong in the
        same upstream contribution as the `FrameID` field of the decision-15 amendment.
      - [x] RECOVER THE ESU-TAP PCB SOURCE — DONE as a PATTERN REFERENCE 2026-08-19, which is
        all this item ever needed to be. The board is still under active revision upstream, so
        it is deliberately NOT adopted into `arts_esu/hardware/` yet; what was blocking the
        sync-box PCB was the unanswered question of toolchain and house style, and that is now
        answered. Adoption of the design itself is a Phase 4 item, once the upstream revisions
        settle.
        The design was found in a private GitHub repo and cloned to
        `$ARTS_LOCAL/code_intake/esu_tap/0_incoming/` (symlinked as `reference/esu_tap`), with
        per-part verdicts in that directory's `NOTES.md`. IT ANSWERED THE QUESTION IT WAS ASKED
        FOR: toolchain is **KiCad 9**, revisions are self-contained directories with vendored
        libraries on project-relative paths, and fab exports are release-only — which corrected
        decision 14's fab-export rule rather than the other way round. It also delivered more
        than a house style: a **confirmed safety bug** (v1's power-tied bypass does not complete,
        because `COMMON` is not routed through, so an unpowered board leaves the surgeon's pedal
        INERT — fixed in v2 by a third bypass pole; verify on the physical board before any
        tissue work), a ROS 2 `pedal_controller` package to port, and measured CPC connector
        pinouts. Two divergences to resolve: the firmware is PlatformIO/Humble against ARTS's
        pinned ESP-IDF/Jazzy, and NO fabrication files exist for either revision, so ordering v2
        means generating them from a reviewed commit first.
        STILL OPEN before anything lands in the tree: copyright ownership (MIT, held by an
        individual RA), anonymity of the notice and upstream project name, and confirming ARTS's
        generator/pedal are the models it was built against. See `NOTES.md`.
      - [~] Sync-box PCB: design package authored here → drawn by someone with EDA experience
        → reviewed here pin-by-pin against the connection table → fabricated → committed at
        `arts_camera_sync/hardware/`. Only then does a skew baseline count (decision 14).
        **DESIGN PACKAGE WRITTEN 2026-08-20** at `arts_camera_sync/hardware/arts-camera-sync-v2/
        design_package/`: netlist (12 nets), the pin-by-pin connection table in TWO keys,
        BOM with distributor numbers, layout/fab constraints, and a bring-up procedure whose
        Stage A is a continuity check with the module removed. **LAYOUT DRAWN 2026-09-03,
        APPROVED 2026-09-08** (`arts-camera-sync-v2`, KiCad 9): R1–R6 0402, 80 × 55 mm,
        USB +X / antenna −X (trust 3-D, not the SnapEDA 2-D courtyard), dual `GND` pour,
        B.Cu hops, stitch vias beside hops plus a south row near D1–D6 and a sparse
        open-pour grid, extra DevKitC GND pins (`J3_1`, `J3_7`) on net `GND`, DRC 0/0,
        pin-check vs `01` §2–§4. `+3V3` stays out of the antenna keep-out. KiCad source is
        in `hardware/arts-camera-sync-v2/` (`arts-camera-sync-v2.kicad_pro`). This is the
        first published revision. JLCPCB order queued — 2 layer, 1.6 mm, HASL lead-free,
        green; PCBA **R1–R6 + D1–D6 only**; sockets / Phoenix / DevKitC hand-fit.
        FABRICATED BOARDS RECEIVED 2026-10-20; not yet hand-fitted, continuity-checked or
        baselined. `production/` still empty (commit the exports that were actually
        ordered); no committed skew baseline.
        `hardware/README.md` + `hardware/.gitignore` adopt ESU-TAP's house policy
        (self-contained revisions, release-only fab exports).
        FOUR THINGS THE PACKAGE PINS DOWN THAT WERE NOT DECIDED BEFORE, each because getting
        it wrong yields a board that assembles cleanly and mismeasures:
        (1) **The loopback taps the PAD, not the resistor output.** `JP1` sits on the GPIO 25
        node, never on the far side of the 560 Ω — otherwise the camera's opto LED capacitance
        joins the loopback's edge and absolute trigger-to-exposure latency becomes a function
        of which cameras happen to be plugged in.
        (2) **1 % resistors from one reel, and this is the highest-value line in the BOM.**
        The 560 Ω sets the opto drive current and turn-on delay falls as current rises, so a
        5 % pair can differ 10 % between channels. That is the leading candidate for the
        ~1.9 µs of the 2.81 µs systematic left unexplained after the strobe-side term was
        isolated at 5e. Costs cents; addresses the largest identified contributor.
        (3) **DO NOT length-match the channels.** FR4 is ~6 ps/mm, so 50 mm of mismatch is
        0.024 capture ticks — four orders of magnitude below the term being chased. Written
        down explicitly because "high-precision timing board" otherwise invites the effort.
        (4) **Low-capacitance SOD-323 TVS, not a through-hole body.** The obvious substitution
        has ~100× the capacitance: 1000 pF against the 1 kΩ pull-up is 1 µs on the rising
        edge, i.e. 4 % of the opto's own delay, added per-channel to the edge that dates the
        photons. 12 pF is fine, 1000 pF is not.
        Unused pigtail wires stay CUT AND INSULATED rather than getting a park terminal: brown
        (pin 6) is the camera's non-isolated ground and landing it near our GND shorts across
        the optocoupler, silently discarding the ESU isolation that motivated Line 0. A
        terminal makes that reachable to anyone tidying loose wires.
        Also recorded: the ESU-TAP KiCad project already carries a fabricated-once
        `ESP32-DEVKITC-32E` symbol/footprint/3-D model. PROVENANCE RESOLVED 2026-08-21 by
        reading the files: the symbol embeds `SnapEDA_Link` properties and the STEP carries
        SnapEDA's download naming (`--3DModel-STEP-269445`), so it is a THIRD-PARTY SnapEDA
        (SnapMagic) part, not the RA's own work — his permission is neither needed nor his to
        give for this asset. ARTS therefore downloads the same part from SnapEDA directly and
        vendors it in `hardware/<revision>/libraries/` under SnapEDA's own terms (CC-BY-SA 4.0
        with their Design Exception; verify at download time), with source URL, download date
        and license recorded in the revision README. Attribution to SnapEDA identifies nobody,
        so it is capsule-safe. This is the PATTERN for every third-party EDA/CAD asset,
        including future da Vinci instrument geometry: record provenance and license PER ASSET;
        vendor only what the license permits redistributing; otherwise record a fetch recipe,
        exactly as `oem_drivers/` does for SDKs. And no silkscreen text
        may identify a person, lab or repo, because a board photo in the paper is a tracked
        artifact like any other.
  - [x] Record the `~/<cam>/meta` topics. They are the ONLY route to the device clock, and
        they are what the software skew cross-check reads.
        CORRECTED 2026-08-20 — this item previously said "under the composed layout the topic
        is `<container_node>/<cam>/meta`", AND THAT IS WRONG. Composing changes no topic
        name: `Camera` advertises `"~/" + topicPrefix_ + "meta"` and `topicPrefix_` is EMPTY
        in `spinnaker_camera_driver`, so a composable node named `camera_left` publishes
        `/camera_left/meta` exactly as the standalone process did. The container's name never
        enters the topic. The prefixed layout belongs to the SYNCHRONIZED driver, where one
        node owns several cameras — which is precisely the driver ARTS does not use — so the
        note was importing a property of the abandoned design into the chosen one. Practical
        consequence: the 5e bag recipe, its topic list and its analysis need NO change.
        VERIFIED EMPIRICALLY 2026-08-21 under the composed layout: 34.5 s bag captured
        `/camera_left/meta` and `/camera_right/meta` (1036 messages each) beside 1033
        pulses, unchanged names, unchanged record invocation
        (`docs/benchmarks/flir_composed_ab_20260821.md`). The Phase 3 recorder profiles in
        `arts_record` still need to include these topics when they are written — that is
        tracked under Phase 3, not here.
      - [ ] Measure skew both ways (hardware capture + `camera_time` spread) and commit the
            baseline. Nominal rate is NOT 40 Hz for the pair: two 3072×2048 Bayer8 streams are
            ~503 MB/s against ~400 MB/s usable on one 5 Gbps link, which is why camera 2
            vanished during Stage 1. CONTROLLER MAP MEASURED 2026-08-18, and it corrects an
            earlier note in this file that said "split the pair across the two xHCI
            controllers, Bus 004 is empty". There are THREE controllers and Bus 004 is NOT
            reachable from any USB-A port: Bus 001+002 = Intel PCH `00:14.0` and owns EVERY
            USB-A port, Bus 003+004 = NVIDIA TU106 `01:00.2` exposed only through the GPU's
            VirtualLink USB-C, Bus 005+006 = Thunderbolt 3 `06:00.0`. Each pair is one
            controller with a USB2 and a USB3 root hub, so both cameras land on Bus 002
            whichever side of the laptop is used and share its single 5 Gbps uplink. Splitting
            them needs a PASSIVE USB-C→USB-A adapter, not a hub — a bus-powered dongle was
            tried and its hub enumerated at 10000M on Bus 006 with no camera beneath it,
            i.e. bus power (~3 W per Blackfly) or a USB2-only downstream port.
            DECIDED 2026-08-18 for the bring-up: keep L on Bus 002 (direct USB-A) and R on
            Bus 006 (dongle). The reason is DEBUGGING DISCIPLINE, not throughput — on separate
            controllers a frame missing at rung 5e means a trigger fault, while on a shared
            controller it could equally be a USB overrun, and separating those two costs
            nothing. Sharing Bus 002 is fine for 5c/5d, which trigger one camera at a time.
            THE REPLICATION REQUIREMENT IS A CONTROLLER BUDGET, NOT A DONGLE: state it as one
            USB3 host controller per camera at ~400 MB/s each, or a reduced rate/ROI that fits.
            The dongle is a laptop workaround and must never appear in the build guide as
            design. Do not take the final committed dual-camera benchmark over a
            Thunderbolt-tunnelled controller either — fine for bring-up, but the published
            number comes from the production workstation with its topology recorded.
            THIS DOES NOT GATE THE SKEW MEASUREMENT and that is the point: the hardware number
            comes from the ESP32 capture counter, which never touches USB, and the software
            cross-check reads `camera_time` from the `~/<cam>/meta` topics, which are a few
            hundred bytes per frame. Bring 5c–5e up at 25 Hz (~315 MB/s, MOPS's stereo rate) or
            a reduced ROI, and treat "both cameras, full resolution, full rate, zero drops" as
            the separate benchmark deliverable it is — the production workstation is where that
            gets solved. Reducing the ROI vertically also shortens the decision-15
            rolling-shutter stagger proportionally.
- [~] Exit review: methodology verdict RECORDED 2026-09-08 in
      `docs/modules/pilot_flir_camera.md` §Exit review; corrected template (8 items, UR arm
      next) written there and NOT yet applied — that happens when `deploy/modules/ur.sh` is
      created. DELIBERATELY WRITTEN BEFORE the production workstation and the fabricated PCB
      exist: both are weeks out (Dell shipping, PCB purchase request), and neither can move
      a methodology verdict — the committed PCB skew baseline and the full-rate dual-camera
      number stay open above and under Phase 3, and the review claims neither. Headlines:
      shared-source pattern is right but held only where the code was the ONLY copy (fixed,
      see "Build order"); Stages 0–4 on the ~4-day estimate, Stage 5 ~3 weeks and unestimated
      because reading the pinned driver source rescoped it; the benchmark contract missed
      host/RMW, machine class, bus topology, device node-map defaults, wiring revision and
      subscriber state — each found by needing it later, so the layer-4 header becomes a
      fixed checklist that `analyze_bench.py` enforces.
