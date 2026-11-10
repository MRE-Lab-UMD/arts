# arts_camera_sync — the ARTS stereo sync box

ESP32 firmware that fires every FLIR camera from one hardware edge and timestamps each
camera's `ExposureActive` strobe on a single MCPWM capture counter.

- Architecture rationale: AGENTS.md decision 14.
- Pin maps, electrical limits, driver-side parameters: `docs/modules/pilot_flir_camera.md`
  Stage 5.
- Bench procedure — resistor identification, breadboard and screw-terminal layout, meter
  checks, symptom table: `docs/modules/camera_sync_wiring.md`.

**Why this exists in one sentence:** `spinnaker_synchronized_camera_driver` assigns identical
`header.stamp` values to the frames of one pulse, so stereo skew cannot be measured from
image timestamps at all — this board measures it in hardware instead, at one capture tick.

## Layout

```
firmware/                ESP-IDF project (xtensa toolchain — NOT built by colcon)
  main/arts_sync_config.h  pin map and derived compile-time configuration
  main/Kconfig.projbuild   the "ARTS camera sync" menuconfig options
  main/sync_engine.c       MCPWM trigger generation + capture ISR + record ring
  main/main.c              the three run modes
  sdkconfig.defaults       shared settings
  sdkconfig.defaults.microros  layered on for the micro-ROS build only
hardware/arts-camera-sync-v2/  KiCad 9 board (first published revision)
launch/camera_sync.launch.py   micro-ROS agent
```

The enclosing directory is an ament package only so the launch file and docs travel with the
workspace. `colcon build` does not compile the firmware.

## Three builds, one source

The MICROROS build takes UART0 for its transport, which disables the log console and removes
`printf` debugging. So bring the hardware up under SELFTEST and switch modes only once the
numbers look right.

| Mode | Stage | What it does |
|---|---|---|
| `BLINK` | 5a | 2 Hz software-timed square wave on GPIO 25 so an LED is visibly blinking. No camera connected. Production timing is a 100 µs pulse at 30 Hz — a 0.3 % duty cycle that is invisible and proves nothing. |
| `SELFTEST` | 5b–5d | Real MCPWM trigger and capture; prints trigger-period, per-camera trigger-to-exposure and `|cam0−cam1|` skew statistics to the console every 2 s. |
| `MICROROS` | 5e | Publishes `arts_interfaces/CameraSyncPulse` on `/ecm/stereo_sync/pulse`. Console disabled. |

## Toolchain

ESP-IDF is pinned in `oem_drivers/esp_idf/VERSION` and installed by
`deploy/modules/camera_sync.sh`, which also writes the udev rule that makes the board appear at
the stable path `/dev/arts_camera_sync`:

```bash
./deploy/modules/camera_sync.sh host_deps                       # apt deps, dialout, udev
ARTS_ENABLE_ESP_IDF=1 ./deploy/modules/camera_sync.sh oem_install   # ~2 GB, opt-in
```

The opt-in gate is deliberate: this is the only module whose `oem_install` is skipped by
default, because the toolchain is needed to *build* firmware and not to run a board that has
already been flashed. Never source `export.sh` from `~/.bashrc` — it prepends its own Python
venv to `PATH` and shadows the system Python for everything else in that shell, colcon
included.

**`host_deps` genuinely must run first.** ESP-IDF's installer creates its private Python venv
as its LAST step, after fetching ~2 GB of toolchain, and on Ubuntu 24.04 `python3 -m venv` is
a non-functional stub until `python3.12-venv` is installed (which `python3-venv` in
`host_deps` pulls in). Skipping `host_deps` therefore costs a full download and then fails on
the final line. `oem_install` now checks for a working `ensurepip` up front and refuses early,
and it is idempotent — re-running after the fix skips the clone and redoes only the toolchain
step.

## Build and flash

**Build in a shell with NO ROS overlay sourced.** ESP-IDF's venv is created with
`include-system-site-packages = false`, so it is properly isolated from apt Python — but a
venv still honours `PYTHONPATH`, and a sourced ROS overlay puts `/opt/ros/jazzy` and every
overlay's `site-packages` on it. The visible symptom is harmless (ESP-IDF's installer prints
resolver complaints about `launch-ros` and `generate-parameter-library-py` having unmet deps,
because Debian's apt Python packages carry no pip metadata), and nothing is written into the
ROS trees. But the leak runs both ways, so build under `env -i` or in a fresh terminal:

Source `esp_idf_env.sh`, not ESP-IDF's `export.sh`. ARTS forces `IDF_TOOLS_PATH` into
`$ARTS_LOCAL` (Espressif's default hides 1.5 GB in `~/.espressif`), and `export.sh` reads that
variable — sourced bare, it reports no toolchain installed. The wrapper sets it first.

**`env -i` clears `$ARTS_LOCAL` too — pass it in.** This bit people (and cost time on
2026-08-18): the obvious `env -i … bash` followed by `. $ARTS_LOCAL/…` expands to
`. /oem_drivers/esp_idf_env.sh`, which does not exist, so nothing is sourced and the next
command fails with the misleading `idf.py: command not found`. The whole point of `env -i` is
that it removes variables, and `ARTS_LOCAL` is one of them.

```bash
env -i HOME="$HOME" USER="$USER" ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}" \
      PATH=/usr/bin:/bin:/usr/sbin:/sbin bash
. "$ARTS_LOCAL/oem_drivers/esp_idf_env.sh"
cd ~/arts/arts_ros2_ws/src/arts_embedded/arts_camera_sync/firmware
idf.py set-target esp32
idf.py menuconfig            # "ARTS camera sync" — run mode, camera count, rate
idf.py build flash monitor
```

Selecting the mode without `menuconfig` (scripted builds): `set-target` writes its own
`sdkconfig`, which then ignores any later defaults file, so delete it after `set-target`:
`rm -f sdkconfig && idf.py -DIDF_TARGET=esp32 -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;<file
containing CONFIG_ARTS_SYNC_MODE_BLINK=y>" reconfigure build`, then check the mode line in
`sdkconfig` before flashing (observed 2026-10-27: the first attempt silently built SELFTEST).

`idf.py` must run from the `firmware` directory — it looks for the project's `CMakeLists.txt`
in `$PWD` and otherwise reports `CMakeLists.txt not found in project directory`. To only watch
a board that is already flashed, `idf.py -p /dev/arts_camera_sync monitor` is enough; exit with
`Ctrl+]`.

### The MICROROS build

Everything below was established by doing it on 2026-08-18. The recipe:

```bash
cd firmware
mkdir -p components
vcs import components < components.repos                       # 1
ln -s ../../../../../../arts_interfaces \
      components/micro_ros_espidf_component/extra_packages/    # 2
readlink -e components/micro_ros_espidf_component/extra_packages/arts_interfaces
rm -f sdkconfig                                                # 3
idf.py -DIDF_TARGET=esp32 \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.microros" reconfigure build
grep ARTS_SYNC_MODE_MICROROS=y sdkconfig
```

1. **The component**, pinned in `firmware/components.repos`. `components/` is gitignored, so
   it does not exist in a fresh checkout and `vcs import` refuses a missing path (2026-10-28).
   The agent is pinned separately in
   `middle_overlay_ros2_wss/oem_ros2_ws/oem_ros2.repos` and the two must be bumped together —
   a mismatched client and agent fail with a session handshake error that reads like a wiring
   or baud-rate fault.
2. **ARTS's messages.** `CameraSyncPulse` is not a standard type, so the component has to
   generate it, and it only looks in `extra_packages/`. A symlink is enough. Omitting this
   fails at link time on `arts_interfaces__msg__CameraSyncPulse__get_type_support`. The
   target is relative to `extra_packages/`, six levels below `arts_ros2_ws/src`; the
   `readlink -e` line fails on a dangling link (an earlier five-level version of this recipe
   produced one, found 2026-10-28).
3. **Delete `sdkconfig` first.** `-DSDKCONFIG_DEFAULTS` seeds a *new* `sdkconfig`; it does not
   revise an existing one, so switching modes silently keeps the old mode.

`requirements.txt` in `firmware/` lists Python packages this build needs inside the ESP-IDF
venv — `oem_install` installs them. It is the one place where "build firmware in a ROS-free
shell" needs qualifying: that rule is about not leaking `PYTHONPATH` from a sourced overlay,
and it does not mean the build needs no ROS tooling. This one cross-compiles ~57 ROS packages
with colcon and genuinely needs `catkin_pkg` and friends. See the header of that file.

**Two things are compiled into `libmicroros.a`, and neither is what you would expect to
control it.**

The **transport** comes from `firmware/app-colcon.meta`, NOT from
`CONFIG_MICRO_ROS_ESP_UART_TRANSPORT`. That Kconfig option only gates Kconfig sections; the
real switch is `-DRMW_UXRCE_TRANSPORT`, which the component's own `colcon.meta` hardcodes to
`udp`. With the option set and no override the library is built for UDP while the firmware
calls `rmw_uros_set_custom_transport()`, and the symptom is that function being *implicitly
declared* — because its prototype sits behind `RMW_UXRCE_TRANSPORT_CUSTOM` in the generated
`rmw_microxrcedds_c/config.h`, which is undefined in a UDP build. That points at the call site
rather than at the library. `main.c` now `#error`s on the mismatch instead.

The **UART transport implementation is not in the component at all.**
`esp32_serial_open/close/write/read` live in
`examples/int32_publisher_custom_transport/main/esp32_serial_transport.c`, which upstream
expects each application to copy into its own `main/`. `main/CMakeLists.txt` compiles it from
the pinned checkout instead, so nothing third-party is vendored here.

**`libmicroros.a` does not rebuild itself.** `libmicroros.mk` skips its work whenever the
archive exists, so a change to `app-colcon.meta` has no effect until you delete
`libmicroros.a`, `include/`, and `micro_ros_src/{build,install}` in the component. `idf.py
fullclean` does not do this — it cleans the ESP-IDF build, not the vendored ROS build.

**The first build after fetching the component may need a second invocation.** The
component's `CMakeLists.txt` globs `include/*` to assemble its include path, and on the first
pass that directory does not exist yet because `libmicroros` is what creates it. Observed on
2026-08-18; on 2026-10-28 the `reconfigure build` form above succeeded in one pass from a
fresh fetch. If the first pass fails on missing micro-ROS headers, run `idf.py build` again.

Then on the host:

```bash
ros2 launch arts_camera_sync camera_sync.launch.py
ros2 topic echo --qos-reliability best_effort /ecm/stereo_sync/pulse   # /arts_camera_sync/pulse on firmware built before 2026-11-01
```

## Reading the numbers

Everything is in capture ticks; `tick_period_ns` in the message reports the actual tick
period, read back from the hardware rather than assumed, because the capture timer follows
the MCPWM group clock and that has changed between ESP-IDF versions.

- **Skew** is the difference between two `exposure_delay_ticks` entries. This is the number
  the paper reports. Each entry's absolute value also contains the optocoupler propagation
  delay (18 µs in, 36 µs out), which is common-mode across cameras and therefore cancels in
  the difference — which is exactly why the isolated line costs nothing here.
- **Trigger jitter** is the spread of `trigger_period_ticks`. It bounds every sync claim
  made downstream, so report it alongside the skew.
- **`pulse_index`** ties a trigger edge to the frames it produced, via the driver's
  `ChunkData` `FrameID`. It is the invariant to align on; `header.stamp` is not.
- **`header.stamp`** is only the coarse ROS-time anchor from the micro-ROS session sync, and
  is meaningless while `time_synced` is false.

## Capture channel budget — why the third camera costs the loopback

MCPWM group 0 has exactly three capture channels, and they must all share one timer for
inter-camera differences to be exact. Two cameras plus the trigger loopback fills it. A third
camera therefore means dropping the loopback (`ARTS_SYNC_TRIGGER_LOOPBACK=n`), after which
camera 0 becomes the reference: skew survives intact, and only the absolute
trigger-to-exposure delay is lost. `arts_sync_config.h` fails the build rather than letting
the over-subscription surface as a runtime error.
