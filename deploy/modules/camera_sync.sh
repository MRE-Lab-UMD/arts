#!/usr/bin/env bash
# deploy/modules/camera_sync.sh — single source of truth for the ARTS camera sync box
# (arts_embedded/arts_camera_sync, AGENTS.md decision 14).
#
# Functions:
#   camera_sync_host_deps      — ESP-IDF build prerequisites, dialout group, udev rule
#   camera_sync_oem_install    — ESP-IDF itself at the pinned tag + xtensa toolchains
#   camera_sync_overlay_build  — micro-ROS agent overlay (oem_ros2_ws); the FIRMWARE is not
#                                built here, see the note below
#
# THE FIRMWARE IS NOT BUILT BY THIS SCRIPT, deliberately. It is cross-compiled for xtensa by
# `idf.py` and then flashed over USB to a board that may not be attached, so it cannot be a
# step in an unattended install the way an overlay build can. This script installs everything
# `idf.py` needs; the flashing recipe is in the package README.
#
# Docker note: unlike flir.sh there is no container path yet. Flashing needs a USB serial
# device and the toolchain is ~2 GB, so the sync box is a bare-metal development dependency.
# If a container is ever wanted it should build firmware artifacts only, never flash.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"

# Shared overlay-build sequence; also sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# shellcheck source=../../oem_drivers/esp_idf/VERSION
source "$ARTS_ROOT/oem_drivers/esp_idf/VERSION"   # → VERSION, ESP_IDF_TARGETS

ESP_IDF_DIR="$ARTS_LOCAL/oem_drivers/esp-idf-${VERSION}"

# Espressif's own default is ~/.espressif, which would put ~1.5 GB of xtensa toolchain in a
# hidden directory in $HOME — outside both ARTS trees, recorded by nothing, and invisible to
# anyone auditing what a machine actually has. IDF_TOOLS_PATH overrides it, so the toolchain
# lands in the machine-local tree beside the SDK it belongs to, per the two-tree rule in
# AGENTS.md. Consequence to respect: export.sh reads IDF_TOOLS_PATH too, so sourcing it with
# this unset finds no tools at all. That is what esp_idf_env.sh below exists to prevent.
ESP_IDF_TOOLS_DIR="$ARTS_LOCAL/oem_drivers/esp_idf_tools"
ESP_IDF_ENV_SH="$ARTS_LOCAL/oem_drivers/esp_idf_env.sh"

# ─────────────────────────────────────────────────────────────────────
camera_sync_host_deps() {
    echo "[camera_sync] host_deps ..."

    # ESP-IDF's documented prerequisite set for Linux. Not apt-pinned with `=version` for the
    # rolling-repo reason recorded in the Ruckig decision: these are build tools whose exact
    # Debian revision does not reach any published number.
    $SUDO apt-get update
    $SUDO apt-get install -y --no-install-recommends \
        git wget flex bison gperf ccache cmake ninja-build dfu-util \
        python3 python3-pip python3-venv \
        libffi-dev libssl-dev libusb-1.0-0

    # Flashing writes to a tty. Without this the symptom is a permission error that looks
    # like a broken board.
    local who="${SUDO_USER:-$USER}"
    if [ -n "$who" ] && ! id -nG "$who" 2>/dev/null | grep -qw dialout; then
        $SUDO usermod -aG dialout "$who" \
            && echo "[camera_sync]   added $who to dialout — log out and back in to apply"
    fi

    # A stable device path. The DevKitC enumerates as a generic USB serial bridge, so
    # /dev/ttyUSB0 is whatever was plugged in first — fine while breadboarding, not fine once
    # a recording session depends on the agent finding the right port.
    # Matched by the bridge's SERIAL NUMBER, not by vendor/product: the ESU-TAP board is an
    # ESP32-DevKitC with the same CP2102N (10c4:ea60), and the first rule (any CP210x/CH340)
    # pointed /dev/arts_camera_sync at the ESU-TAP when it was plugged in (2026-11-03), so a
    # `idf.py -p /dev/arts_camera_sync flash` would have overwritten the ESU-TAP's firmware.
    # Another cart's box: ARTS_CAMERA_SYNC_SERIAL=<ID_SERIAL_SHORT from
    # `udevadm info -q property -n /dev/ttyUSBn`>.
    local serial="${ARTS_CAMERA_SYNC_SERIAL:-9072b44464bcf0118aca167148e9de0f}"
    local rules="/etc/udev/rules.d/70-arts-camera-sync.rules"
    local want
    want="# ARTS camera sync box (ESP32-DevKitC-32E, CP2102N), by serial number: other ESP32 boards
# (the ESU-TAP) carry the same bridge. Written by deploy/modules/camera_sync.sh.
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"$serial\", MODE=\"0660\", GROUP=\"dialout\", SYMLINK+=\"arts_camera_sync\""
    if [ "$(cat "$rules" 2>/dev/null)" != "$want" ]; then
        printf '%s\n' "$want" | $SUDO tee "$rules" >/dev/null
        $SUDO udevadm control --reload-rules && $SUDO udevadm trigger --subsystem-match=tty
        $SUDO udevadm settle
        echo "[camera_sync]   udev rule → $rules (serial $serial → /dev/arts_camera_sync)"
    fi

    echo "[camera_sync] host_deps done."
}

# ─────────────────────────────────────────────────────────────────────
camera_sync_oem_install() {
    # ESP-IDF plus its xtensa toolchains is ~2 GB and is only needed to BUILD firmware, not
    # to run the sync box — a flashed board needs nothing but the micro-ROS agent. So this is
    # opt-in: an unattended full install of the whole system must not silently pull 2 GB for
    # a user who never touches embedded code. This is the only module that is gated; every
    # other oem_install is needed to run its hardware.
    if [ "${ARTS_ENABLE_ESP_IDF:-0}" != "1" ] && [ ! -d "$ESP_IDF_DIR/.git" ]; then
        echo "[camera_sync] oem_install SKIPPED (ESP-IDF toolchain, ~2 GB)."
        echo "[camera_sync]   Needed only to build/flash sync-box firmware. To install:"
        echo "[camera_sync]     ARTS_ENABLE_ESP_IDF=1 ./deploy/modules/camera_sync.sh oem_install"
        return 0
    fi

    # ESP-IDF's install.sh ends by building a private Python venv, and it does that AFTER
    # downloading ~2 GB of toolchain. On Ubuntu 24.04 `python3 -m venv` is a stub unless
    # python3.12-venv is present (python3-venv in host_deps pulls it), so without it the
    # install dies at the very last step having spent several minutes. Fail up front instead.
    if ! python3 -c 'import ensurepip' 2>/dev/null; then
        echo "[camera_sync] ERROR: python3 -m venv is not functional (ensurepip missing)."
        echo "[camera_sync]        ESP-IDF's installer would download ~2 GB and THEN fail."
        echo "[camera_sync]        Run host_deps first (needs sudo):"
        echo "[camera_sync]          ./deploy/modules/camera_sync.sh host_deps"
        return 1
    fi

    echo "[camera_sync] oem_install: ESP-IDF $VERSION → $ESP_IDF_DIR"

    if [ ! -d "$ESP_IDF_DIR/.git" ]; then
        mkdir -p "$(dirname "$ESP_IDF_DIR")"
        # ESP-IDF vendors its dependencies as submodules and is large; a shallow clone of the
        # pinned tag keeps it to a few hundred MB instead of several GB of history.
        git clone --depth 1 --branch "$VERSION" --recurse-submodules --shallow-submodules \
            https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    else
        echo "[camera_sync]   already cloned; verifying the pin"
        local have
        have="$(git -C "$ESP_IDF_DIR" describe --tags --exact-match 2>/dev/null || echo unknown)"
        if [ "$have" != "$VERSION" ]; then
            echo "[camera_sync]   WARNING: checkout is at '$have', VERSION pins '$VERSION'."
            echo "[camera_sync]   Delete $ESP_IDF_DIR and re-run to move the pin."
        fi
    fi

    # Installs the xtensa toolchains and a private Python venv, into ARTS_LOCAL rather than
    # Espressif's hidden ~/.espressif default.
    mkdir -p "$ESP_IDF_TOOLS_DIR"
    export IDF_TOOLS_PATH="$ESP_IDF_TOOLS_DIR"
    "$ESP_IDF_DIR/install.sh" "$ESP_IDF_TARGETS"

    # The MICROROS firmware build needs ROS build tooling inside ESP-IDF's OWN venv, because
    # micro_ros_espidf_component ships no prebuilt archive — it cross-compiles ~57 ROS 2
    # packages with colcon into libmicroros.a. That venv is created with
    # include-system-site-packages = false, so it cannot see apt's python3-catkin-pkg and the
    # build dies on `ModuleNotFoundError: No module named 'catkin_pkg'` (observed 2026-08-18).
    # Into the venv rather than system Python on purpose: pip over apt's python3-catkin-pkg is
    # the classic apt/pip conflict, and the venv copy is confined to ARTS_LOCAL. Rationale and
    # the empy<4 pin are in the requirements file itself.
    #
    # Run in a subshell: export.sh prepends the venv to PATH, which must not leak to the rest
    # of this script.
    local fw_reqs="$ARTS_ROOT/arts_ros2_ws/src/arts_embedded/arts_camera_sync/firmware/requirements.txt"
    if [ -f "$fw_reqs" ]; then
        echo "[camera_sync]   installing firmware build deps into the ESP-IDF venv"
        (
            set -eo pipefail
            . "$ESP_IDF_DIR/export.sh" >/dev/null
            set -u
            python -m pip install --no-input -r "$fw_reqs"
        )
    else
        echo "[camera_sync]   WARNING: $fw_reqs missing; the MICROROS build will fail on catkin_pkg"
    fi

    # One file to source, so nobody has to remember IDF_TOOLS_PATH. Sourcing ESP-IDF's own
    # export.sh without it silently finds no toolchain at all.
    cat > "$ESP_IDF_ENV_SH" <<EOF
# Generated by deploy/modules/camera_sync.sh — do not edit; re-run oem_install instead.
# Activates the ARTS-pinned ESP-IDF for firmware work in THIS shell only.
#
#   . $ESP_IDF_ENV_SH
#
# Two reasons this wrapper exists rather than sourcing export.sh directly. IDF_TOOLS_PATH must
# match what oem_install used, or no toolchain is found. And export.sh prepends its own Python
# venv to PATH, shadowing the system Python for everything else in the shell — colcon
# included — so this must never go into a shell rc file, and firmware should be built in a
# shell with no ROS overlay sourced, because a venv still honours PYTHONPATH.
export IDF_TOOLS_PATH="$ESP_IDF_TOOLS_DIR"
. "$ESP_IDF_DIR/export.sh"
EOF

    cat <<EOF

[camera_sync] oem_install done. Activate the toolchain in each new shell with:

    . $ESP_IDF_ENV_SH

Do NOT add that to ~/.bashrc, and do not source ESP-IDF's export.sh directly — it needs
IDF_TOOLS_PATH=$ESP_IDF_TOOLS_DIR, which the wrapper sets.
EOF
}

# ─────────────────────────────────────────────────────────────────────
camera_sync_overlay_build() {
    # Builds the micro-ROS agent, which is the host side of the sync box. Not available from
    # apt on Jazzy (`ros-jazzy-micro-ros-*` ships only msgs and the diagnostic bridge), so it
    # comes from source via oem_ros2.repos.
    #
    # oem_ros2_ws is SHARED with every other loose OEM wrapper, so this module does not own
    # it (see overlay_owner_module in _common.sh); it delegates to the same generic call as
    # the overlay's build.sh, so `./camera_sync.sh overlay_build` keeps working as a shortcut.
    local ws="${1:-$ARTS_LOCAL/middle_overlay_ros2_wss/oem_ros2_ws}"

    echo "[camera_sync] overlay_build: micro-ROS agent via oem_ros2_ws"
    overlay_build_generic oem_ros2_ws --ws "$ws"
    echo "[camera_sync] overlay_build done: $ws"
}

# ─────────────────────────────────────────────────────────────────────
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    cmd="${1:-}"
    case "$cmd" in
        host_deps)     camera_sync_host_deps ;;
        oem_install)   camera_sync_oem_install ;;
        overlay_build) camera_sync_overlay_build "${2:-}" ;;
        all)           camera_sync_host_deps; camera_sync_oem_install
                       camera_sync_overlay_build "${2:-}" ;;
        *) echo "Usage: $0 {host_deps|oem_install|overlay_build [ws_path]|all}"; exit 1 ;;
    esac
fi
