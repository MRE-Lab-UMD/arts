#!/usr/bin/env bash
# deploy/modules/head_sensor.sh — the GOOVIS HMD's head-presence sensor (decision 18
# /console/head_sensor; node arts_teleop/scripts/head_sensor.py).
#
# Functions:
#   head_sensor_host_deps  — udev rule giving the plugdev group read access to the HMD's hidraw
#
# No OEM install and no overlay: the HMD reports presence as a plain USB HID report (vendor
# 880a, product 3501; the LAST byte is 1 with a head in front of the optics), read from
# /dev/hidraw*. Without the rule the node is root-only (crw-------) and logs "cannot open".
#
# Docker note: a container reads it only with the hidraw device passed in (--device
# /dev/hidrawN) and the plugdev GID; the rule itself is host-side, like every udev rule.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# ─────────────────────────────────────────────────────────────────────
head_sensor_host_deps() {
    echo "[head_sensor] host_deps…"
    local rules="/etc/udev/rules.d/99-arts-goovis-head-sensor.rules"
    if [ ! -f "$rules" ]; then
        $SUDO tee "$rules" >/dev/null <<'RULE'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="880a", ATTRS{idProduct}=="3501", MODE="0660", GROUP="plugdev"
RULE
        $SUDO udevadm control --reload && $SUDO udevadm trigger --subsystem-match=hidraw
        echo "[head_sensor]   udev rule → $rules (group plugdev)"
    fi

    local who="${SUDO_USER:-$USER}"
    if [ -n "$who" ] && ! id -nG "$who" 2>/dev/null | grep -qw plugdev; then
        $SUDO usermod -aG plugdev "$who" \
            && echo "[head_sensor]   added $who to plugdev — log out and back in to apply"
    fi
    echo "[head_sensor] host_deps done."
}
