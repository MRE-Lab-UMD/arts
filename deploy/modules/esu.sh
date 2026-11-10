#!/usr/bin/env bash
# deploy/modules/esu.sh — the ESU-TAP footswitch interposer (arts_embedded/arts_esu, decision 18).
#
# Functions:
#   esu_host_deps — udev rule: /dev/arts_esu for the board, dialout access
#
# The board runs its designer's firmware unmodified (oem_drivers/esu_tap/VERSION); nothing is
# flashed here. Its ESP32-DevKitC has the same CP2102N USB bridge (10c4:ea60) as the camera
# sync box, so each is matched by the bridge's serial number. Another cart's board:
# ARTS_ESU_SERIAL=<ID_SERIAL_SHORT from `udevadm info -q property -n /dev/ttyUSBn`>.
#
# The micro-ROS agent comes from the pinned oem_ros2_ws overlay (oem_ros2.repos), as for the
# sync box; the esu node needs rmw_fastrtps_cpp, which ros-jazzy-ros-base already installs.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# ─────────────────────────────────────────────────────────────────────
esu_host_deps() {
    echo "[esu] host_deps…"
    local serial="${ARTS_ESU_SERIAL:-322a115f7abcf0119c84157148e9de0f}"
    local rules="/etc/udev/rules.d/70-arts-esu.rules"
    local want
    want="# ARTS ESU-TAP board (ESP32-DevKitC-32E, CP2102N), by serial number: the camera sync box
# carries the same bridge. Written by deploy/modules/esu.sh.
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"$serial\", MODE=\"0660\", GROUP=\"dialout\", SYMLINK+=\"arts_esu\""
    if [ "$(cat "$rules" 2>/dev/null)" != "$want" ]; then
        printf '%s\n' "$want" | $SUDO tee "$rules" >/dev/null
        $SUDO udevadm control --reload-rules && $SUDO udevadm trigger --subsystem-match=tty
        $SUDO udevadm settle
        echo "[esu]   udev rule → $rules (serial $serial → /dev/arts_esu)"
    fi
    echo "[esu] host_deps done."
}
