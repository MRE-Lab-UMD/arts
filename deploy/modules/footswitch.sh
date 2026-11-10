#!/usr/bin/env bash
# deploy/modules/footswitch.sh — the console 3-pedal footswitch (Kinesis Savant Elite2,
# decision 9; node arts_teleop/scripts/footswitch.py, mapping config/input_mapping.yaml).
#
# Functions:
#   footswitch_host_deps   — udev rule giving the plugdev group read access to the pedal
#
# No OEM install and no overlay: the pedal is a plain USB HID keyboard and the node reads its
# evdev interface directly. Without the rule the device is root:input 0660 and the node logs
# "cannot open ... (udev rule installed?)".
#
# Docker note: a container reads the pedal only with the event device passed in
# (--device /dev/input/eventN, or the by-id path) and the plugdev GID; the rule itself is
# host-side, like every udev rule.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# ─────────────────────────────────────────────────────────────────────
footswitch_host_deps() {
    echo "[footswitch] host_deps…"
    local rules="/etc/udev/rules.d/99-arts-savant-pedal.rules"
    if [ ! -f "$rules" ]; then
        $SUDO tee "$rules" >/dev/null <<'EOF'
SUBSYSTEM=="input", ATTRS{idVendor}=="29ea", ATTRS{idProduct}=="0100", MODE="0660", GROUP="plugdev"
EOF
        $SUDO udevadm control --reload && $SUDO udevadm trigger --subsystem-match=input
        echo "[footswitch]   udev rule → $rules (group plugdev)"
    fi

    local who="${SUDO_USER:-$USER}"
    if [ -n "$who" ] && ! id -nG "$who" 2>/dev/null | grep -qw plugdev; then
        $SUDO usermod -aG plugdev "$who" \
            && echo "[footswitch]   added $who to plugdev — log out and back in to apply"
    fi
    echo "[footswitch] host_deps done."
}
