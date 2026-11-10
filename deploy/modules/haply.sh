#!/usr/bin/env bash
# deploy/modules/haply.sh — Haply Inverse3 + VerseGrip (decision 9).
#
# Functions:
#   haply_oem_install — fetch the pinned Haply Hub .deb into $ARTS_LOCAL, verify its SHA-256,
#                       install it with apt (Hub GUI, service library, udev rule)
#   haply_host_deps   — the bridge's WebSocket client (arts_driver/haply_bridge)
#
#   ./deploy/modules/haply.sh {oem_install|host_deps}   (as your user; apt asks for sudo)
#
# The download is public, so unlike Spinnaker it is fetched here, but only at the URL and
# hash in oem_drivers/haply/VERSION: a mismatch fails, it never installs "whatever is there".
# Device firmware is flashed from the Hub GUI afterwards and recorded, not pinned.
#
# Docker note: not yet containerized; the Hub is a GUI and the udev rule is host-side.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Sets SUDO (empty as root, "sudo" otherwise).
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# ─────────────────────────────────────────────────────────────────────
haply_oem_install() {
    # shellcheck source=../../oem_drivers/haply/VERSION
    source "$ARTS_ROOT/oem_drivers/haply/VERSION"
    local dir="${ARTS_LOCAL:-$HOME/arts_local}/oem_drivers/haply-$VERSION"
    local deb="$dir/${URL##*/}"
    echo "[haply] oem_install: Hub $VERSION"

    if [ ! -f "$deb" ]; then
        mkdir -p "$dir"
        curl -fSL -o "$deb.part" "$URL" && mv "$deb.part" "$deb"
    fi
    if ! echo "$SHA256  $deb" | sha256sum -c --quiet -; then
        echo "[haply] ERROR: $deb does not match the pinned SHA-256 — not installing." >&2
        exit 1
    fi

    # The .deb does not declare libusb-0.1: without it haply-device-manager exits with
    # "libusb-0.1.so.4: cannot open shared object file" (Hub 1.5.0, 24.04, 2026-10-29).
    $SUDO apt-get install -y libusb-0.1-4=2:0.1.12-35build1

    local have
    have=$(dpkg-query -W -f='${Version}' haply-hub 2>/dev/null || true)
    if [ "$have" = "$VERSION" ]; then
        echo "[haply]   haply-hub $VERSION already installed"
    else
        $SUDO apt-get install -y "$deb"
    fi
    echo "[haply] oem_install done: $(dpkg-query -W -f='${Package} ${Version}' haply-hub)"
}

# ─────────────────────────────────────────────────────────────────────
haply_host_deps() {
    echo "[haply] host_deps…"
    # websocket-client, the synchronous client the bridge's receive thread uses.
    $SUDO apt-get install -y python3-websocket=1.7.0-1
    echo "[haply] host_deps done."
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    case "${1:-}" in
        oem_install) haply_oem_install ;;
        host_deps) haply_host_deps ;;
        *) echo "usage: $0 {oem_install|host_deps}" >&2; exit 2 ;;
    esac
fi
