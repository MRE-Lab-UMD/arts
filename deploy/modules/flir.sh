#!/usr/bin/env bash
# deploy/modules/flir.sh — single source of truth for the FLIR camera module.
# Consumed by BOTH arts_install.sh (bare metal) and Dockerfile.flir (container).
#
# Functions:
#   flir_host_deps      — udev, usbfs (host only, needs sudo). The CycloneDDS receive
#                         buffer used to live here and now belongs to dds.sh.
#   flir_oem_install    — Spinnaker SDK debs (non-interactive, works in Docker)
#   flir_overlay_build  — flir_ros2_ws from flir_ros2.repos
#
# Deb search order (flir_oem_install):
#   1. /tmp/spinnaker/  (Docker staging — set by build_flir.sh)
#   2. the directory under $ARTS_LOCAL/oem_drivers/spinnaker-<ver>/ that holds
#      libspinnaker_<ver>_amd64.deb (flir_deb_dir; the tarball's folder name differs
#      between releases, so it is found by the deb, not assumed)
#
# Version: oem_drivers/spinnaker/VERSION pins 4.2.0.88; ARTS_SPINNAKER_VERSION overrides it
# explicitly and every function here then says so (flir_version_notice). Cameras are optional:
# ARTS_ENABLE_FLIR=0 ./arts_install.sh leaves this module out (mock and sim need no SDK).
#
# Ubuntu 24.04 note: Spinnaker 4.2.0.88 officially supports 20.04 and 22.04 only.
# In practice the split is clean, because the components link differently:
#   libSpinnaker  → libc, libstdc++, libgomp, libusb   … all present on noble, works
#   libSpinVideo  → libavcodec.so.58, libavformat.so.58, libavutil.so.56,
#                   libswscale.so.5                     … jammy-only ffmpeg ABI, broken
#   SpinView_QT   → Qt5 + libSpinVideo                  … broken by association
# The ROS 2 driver and every CLI sample tool use libSpinnaker only, so the ARTS
# data path is unaffected. SpinVideo is a video-file writer ARTS does not use
# (we record MCAP), so those two packages are skipped rather than shimmed with
# foreign-ABI ffmpeg libraries. See docs/modules/pilot_flir_camera.md.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"

# SUDO (empty as root inside a Docker build, "sudo" for a user on bare metal) is set by
# _common.sh below.

# The shared overlay-build sequence. Dockerfile.flir stages this file beside flir.sh
# (build_flir.sh copies it), so the image and the host run the identical function.
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# shellcheck source=../../oem_drivers/spinnaker/VERSION
source "$ARTS_ROOT/oem_drivers/spinnaker/VERSION"   # → VERSION, PINNED_VERSION, DOWNLOAD_URL

# The directory holding the SDK debs for $VERSION, or nothing. Matching the exact deb name
# means a mirror of a different release can never be installed under the pin's name.
# --mirror-only skips the Docker staging dir (build_flir.sh, which fills it, runs on the host).
flir_deb_dir() {
    if [ "${1:-}" != --mirror-only ] && ls /tmp/spinnaker/libspinnaker_*_amd64.deb &>/dev/null; then
        echo /tmp/spinnaker; return 0
    fi
    local root="$ARTS_LOCAL/oem_drivers/spinnaker-${VERSION}" deb
    [ -d "$root" ] || return 0
    deb=$(find "$root" -maxdepth 3 -name "libspinnaker_${VERSION}_amd64.deb" -print -quit)
    [ -n "$deb" ] && dirname "$deb"
    return 0
}

# Loud, every time: an SDK other than the pin is a deviation the user must be able to see in
# any install log they attach to a result.
flir_version_notice() {
    [ "$VERSION" = "$PINNED_VERSION" ] && return 0
    echo "[flir] ****************************************************************"
    echo "[flir]  Spinnaker $VERSION (ARTS_SPINNAKER_VERSION), NOT the pinned $PINNED_VERSION."
    echo "[flir]  Unbenchmarked: every ARTS camera result used $PINNED_VERSION. State this"
    echo "[flir]  version beside any camera number measured on this install."
    echo "[flir] ****************************************************************"
}

flir_sdk_missing_msg() {
    echo ""
    echo "================================================================"
    echo "  ERROR: Spinnaker SDK $VERSION debs not found."
    echo "================================================================"
    echo "  Looked for libspinnaker_${VERSION}_amd64.deb under"
    echo "    $ARTS_LOCAL/oem_drivers/spinnaker-${VERSION}/"
    echo ""
    echo "  The SDK is needed ONLY for the FLIR stereo cameras. Pick one:"
    echo ""
    echo "  a) No cameras (mock robot, simulation, arms without the endoscope):"
    echo "       ARTS_ENABLE_FLIR=0 ./arts_install.sh ..."
    echo ""
    echo "  b) Cameras, pinned SDK: download Spinnaker $PINNED_VERSION for Ubuntu 22.04"
    echo "     (amd64; free Teledyne account; under 'All Versions' if not the latest)"
    echo "       $DOWNLOAD_URL"
    echo "     and extract the tarball into"
    echo "       $ARTS_LOCAL/oem_drivers/spinnaker-${PINNED_VERSION}/"
    echo ""
    echo "  c) Cameras, but $PINNED_VERSION is no longer offered: extract the release you"
    echo "     can get into $ARTS_LOCAL/oem_drivers/spinnaker-<its version>/ and run"
    echo "       ARTS_SPINNAKER_VERSION=<its version> ./arts_install.sh ..."
    echo "     (unbenchmarked: ARTS's camera results used $PINNED_VERSION)"
    echo "================================================================"
}

# ─────────────────────────────────────────────────────────────────────
flir_host_deps() {
    echo "[flir] host_deps ..."

    # USB3 Vision buffer (Spinnaker recommends ≥ 1000 MB; kernel default 16 MB).
    # Persist for future boots AND apply now — usbcore is built in on Ubuntu, so the
    # modprobe.d file alone does nothing until the next reboot, and a benchmark run
    # before that reboot silently drops frames at 40 fps.
    local usbfs="/etc/modprobe.d/arts-usbfs.conf"
    if ! grep -qs "usbfs_memory_mb=1000" "$usbfs" 2>/dev/null; then
        echo "options usbcore usbfs_memory_mb=1000" | $SUDO tee "$usbfs" >/dev/null
        echo "[flir]   usbfs_memory_mb=1000 → $usbfs (persisted)"
    fi
    local usbfs_now="/sys/module/usbcore/parameters/usbfs_memory_mb"
    if [ -w "$usbfs_now" ] || [ -e "$usbfs_now" ]; then
        if [ "$(cat "$usbfs_now" 2>/dev/null)" != "1000" ]; then
            echo 1000 | $SUDO tee "$usbfs_now" >/dev/null \
                && echo "[flir]   usbfs_memory_mb=1000 applied to the running kernel"
        fi
    fi

    # The CycloneDDS receive buffer moved to deploy/modules/dds.sh, which owns the
    # middleware for both paths. It is checked rather than set here because it is not
    # FLIR's to own, but a too-small rmem drops frames at 40 fps while looking like a
    # camera problem, so narrowing an install to `--modules flir` should not fail quietly.
    local rmem_min=10485760
    if [ "$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)" -lt "$rmem_min" ]; then
        echo "[flir]   WARNING: net.core.rmem_max is below 10 MB. CycloneDDS will be"
        echo "[flir]            silently capped and frames will drop under load."
        echo "[flir]            Run the dds module: ./arts_install.sh --modules dds"
    fi

    # Spinnaker udev rules. The SDK ships these in configure_spinnaker.sh, a postinst
    # helper that flir_oem_install deliberately bypasses (dpkg-deb -x skips maintainer
    # scripts to avoid the interactive EULA in Docker/CI). So ARTS writes them itself
    # rather than telling the user to run a vendor script by hand — otherwise a fresh
    # install has root-only cameras and the bare-metal path fails in a way the Docker
    # path, which runs privileged, does not.
    local udev_f="/etc/udev/rules.d/40-flir-spinnaker.rules"
    local grp="flirimaging"
    if [ ! -f "$udev_f" ]; then
        getent group "$grp" >/dev/null || $SUDO groupadd -f "$grp"
        # 1e10 = Point Grey Research (Blackfly S and current FLIR USB3 cameras),
        # 1724 = the legacy Point Grey vendor ID, both per the SDK's own script.
        printf 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", GROUP="%s"\nSUBSYSTEM=="usb", ATTRS{idVendor}=="1724", GROUP="%s"\n' \
            "$grp" "$grp" | $SUDO tee "$udev_f" >/dev/null
        $SUDO udevadm control --reload-rules >/dev/null 2>&1
        $SUDO udevadm trigger >/dev/null 2>&1
        echo "[flir]   udev rules → $udev_f (group $grp)"
    fi
    # Group membership applies at next login; say so rather than let it look applied.
    local who="${SUDO_USER:-$USER}"
    if [ -n "$who" ] && [ "$who" != "root" ] && ! id -nG "$who" 2>/dev/null | grep -qw "$grp"; then
        $SUDO usermod -aG "$grp" "$who" \
            && echo "[flir]   added $who to $grp — log out and back in for it to apply"
    fi

    echo "[flir] host_deps done."
}

# ─────────────────────────────────────────────────────────────────────
flir_oem_install() {
    local deb_dir
    deb_dir=$(flir_deb_dir)
    if [ -z "$deb_dir" ]; then
        flir_sdk_missing_msg
        exit 1
    fi
    flir_version_notice

    echo "[flir] oem_install: Spinnaker SDK $VERSION from $deb_dir"

    # Runtime deps of libSpinnaker only (see the ABI note at the top of this
    # file). The jammy ffmpeg and Qt5 sets the vendor README lists are for
    # SpinVideo and SpinView_QT, which are skipped below.
    $SUDO apt-get update && $SUDO apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        libgomp1

    # Shrinking the layer by dropping the package lists is correct in an image and
    # DESTRUCTIVE on a host: bare metal shares one /var/lib/apt/lists with the whole
    # machine, so an unconditional cleanup here deletes the user's package lists and every
    # later apt operation fails with "Package X is not available, but is referred to by
    # another package" / "has no installation candidate". That is a confusing symptom
    # because it names a package that is in fact perfectly installable, and it surfaces in
    # whatever step runs next — here it was rosdep resolving ffmpeg, two functions later.
    # Hit for real on 2026-08-18. dds.sh avoids this by keeping a separate
    # dds_image_install(); the deb extraction below is genuinely shared, so this function
    # branches instead of being split in two.
    if [ -f /.dockerenv ]; then
        $SUDO rm -rf /var/lib/apt/lists/*
    fi

    # dpkg-deb -x extracts without running preinst scripts, bypassing the
    # interactive EULA prompt that blocks in Docker / CI.  The user accepted
    # the EULA when downloading the SDK from FLIR's website.
    local skipped=0
    for deb in "$deb_dir"/*.deb; do
        case "$(basename "$deb")" in
            libspinvideo*|spinview-qt*|spinnaker-doc*)
                skipped=$((skipped + 1))
                continue
                ;;
        esac
        $SUDO dpkg-deb -x "$deb" /
    done
    $SUDO ldconfig

    echo "[flir]   skipped $skipped deb(s): SpinVideo/SpinView (jammy ffmpeg ABI), docs"
    echo "[flir] oem_install done."
}

# ─────────────────────────────────────────────────────────────────────
flir_overlay_build() {
    # $1 = workspace root (bare metal: $ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws
    #                       Docker:     /opt/flir_ros2_ws)
    local ws="${1:-$ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws}"
    local repos="$ARTS_ROOT/middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos"

    echo "[flir] overlay_build: $ws (repos: $repos)"

    # PREFLIGHT — refuse to build without the pinned SDK, because upstream's fallback is to
    # go and get an UNPINNED one. spinnaker_camera_driver/CMakeLists.txt does
    # find_package(SPINNAKER QUIET) and, on failure, include(cmake/DownloadSpinnaker.cmake),
    # whose script maps the OS codename to a version: noble → 4.4.0.99, fetched from
    # https://eventvisionresearch.com/ — a third-party host, not FLIR. So a machine that
    # skipped flir_oem_install does not fail; it QUIETLY BUILDS AGAINST A DIFFERENT SPINNAKER
    # than oem_drivers/spinnaker/VERSION claims, sidestepping both the pin and the account
    # wall, and every benchmark taken afterwards describes a stack nobody chose. Exactly the
    # failure shape as the Ruckig incident: a pinned version defeated by something arriving
    # through a back door. Failing loudly here is the only way the pin means anything.
    if [ ! -f /opt/spinnaker/lib/libSpinnaker.so ] || [ ! -f /opt/spinnaker/include/Spinnaker.h ]; then
        echo ""
        echo "================================================================"
        echo "  ERROR: Spinnaker SDK $VERSION is not installed."
        echo "================================================================"
        echo "  Expected /opt/spinnaker/lib/libSpinnaker.so and"
        echo "           /opt/spinnaker/include/Spinnaker.h"
        echo ""
        echo "  Run this first:   ./deploy/modules/flir.sh oem_install"
        echo ""
        echo "  Not doing so does NOT fail the build: upstream would download"
        echo "  Spinnaker 4.4.0.99 from a third-party mirror instead, silently"
        echo "  ignoring the pinned $VERSION."
        echo "================================================================"
        exit 1
    fi
    echo "[flir]   preflight: Spinnaker present at /opt/spinnaker ($VERSION)"
    flir_version_notice

    # The build sequence itself is the shared one in _common.sh — this function contributes
    # only the preflight above. It used to carry its own copy, which is how the `set -u`
    # ordering fix and the audible apt-get update lived here for weeks without reaching the
    # other five overlays (docs/repo_structure_triage.md item 3).
    overlay_build_generic flir_ros2_ws --ws "$ws" --repos "$repos"

    echo "[flir] overlay_build done: $ws"
}

# ─────────────────────────────────────────────────────────────────────
# CLI interface: source flir.sh && flir_oem_install
#           or:  ./flir.sh oem_install
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    cmd="${1:-}"
    case "$cmd" in
        host_deps)     flir_host_deps ;;
        oem_install)   flir_oem_install ;;
        overlay_build) flir_overlay_build "${2:-}" ;;
        all)           flir_host_deps; flir_oem_install; flir_overlay_build "${2:-}" ;;
        *) echo "Usage: $0 {host_deps|oem_install|overlay_build [ws_path]|all}"; exit 1 ;;
    esac
fi
