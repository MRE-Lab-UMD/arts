#!/usr/bin/env bash
# deploy/modules/dv_tool_driver.sh — the da Vinci X adapters' Dynamixel buses
# (arts_embedded/arts_dv_tool_driver, node disc_drive; ports in config/disc_buses.yaml).
#
# Functions:
#   dv_tool_driver_host_deps — udev rule: FTDI latency timer 1 ms, dialout access
#
# The adapters are FTDI FT232H USB-serial boards. The ftdi_sio driver batches received bytes
# for latency_timer ms (default 16), so every bus read waits that long: disc_drive ran at
# 62.5 Hz instead of 200 Hz (measured 2026-11-01). Collaborator A set it to 1 ms by hand, and
# the setting is lost on every replug; this rule applies it at every plug. Each board is
# told apart by its FTDI serial number (/dev/serial/by-id), never by /dev/ttyUSBn order.
#
# dynamixel_sdk itself comes from the pinned oem_ros2_ws overlay (oem_ros2.repos).
#
# Docker note: the rule is host-side, like every udev rule; the container gets the ports by
# the c 188:* cgroup rule (compose.yaml, disc_drive).
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# ─────────────────────────────────────────────────────────────────────
dv_tool_driver_host_deps() {
    echo "[dv_tool_driver] host_deps…"
    local rules="/etc/udev/rules.d/99-arts-dynamixel-ftdi.rules"
    # DRIVER (the event device's own driver), NOT DRIVERS: udev requires every parent key in a
    # rule (DRIVERS, ATTRS, ...) to match ONE device in the chain, and ftdi_sio is the driver
    # of the USB interface while idVendor/idProduct live on the USB device above it. The first
    # version used DRIVERS and never matched (latency stayed 16 ms, 2026-11-01).
    local want
    want='# FT232H (0403:6014), the Dynamixel buses of the adapters: 1 ms latency, dialout group.
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTR{latency_timer}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0660", GROUP="dialout"'
    if [ "$(cat "$rules" 2>/dev/null)" != "$want" ]; then
        printf '%s\n' "$want" | $SUDO tee "$rules" >/dev/null
        $SUDO udevadm control --reload
        echo "[dv_tool_driver]   udev rule → $rules"
    fi
    # Apply to the boards plugged in now; udevadm settle so the check below sees the result.
    $SUDO udevadm trigger --action=add --subsystem-match=usb-serial
    $SUDO udevadm settle
    local f
    for f in /sys/bus/usb-serial/devices/*/latency_timer; do
        [ -e "$f" ] && echo "[dv_tool_driver]   $(basename "$(dirname "$f")"): latency_timer $(cat "$f") ms"
    done
    echo "[dv_tool_driver] host_deps done."
}
