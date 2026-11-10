#!/usr/bin/env bash
# deploy/modules/dds.sh — single source of truth for the ARTS middleware layer.
# Consumed by BOTH arts_install.sh (bare metal) and Dockerfile.base (container).
#
# Functions:
#   dds_host_deps    — kernel buffers, the RMW package, the config file, and the
#                      environment that selects them (host only, needs sudo)
#   dds_image_install — the same RMW package and config inside an image (no sudo,
#                      no sysctl: a container shares the host kernel and cannot
#                      set net.core.*, which is precisely why dds_host_deps runs
#                      on the host in the Docker path too)
#
# NOT A HARDWARE MODULE. Everything else in this directory corresponds to a physical
# component with a compose profile. This one is here anyway because deploy/modules/*.sh
# is the hook arts_install.sh already walks, and <name>_host_deps is already run in every
# install path — which is exactly the semantics the middleware needs. The alternative was
# a second orchestration mechanism for one script.
#
# WHY THIS EXISTS AT ALL
# ----------------------
# Dockerfile.base has installed CycloneDDS and set RMW_IMPLEMENTATION since the image was
# first built, so every containerised run has used it. Nothing did the equivalent on the
# host. Found 2026-08-12: bare metal had only rmw_fastrtps_cpp installed and
# RMW_IMPLEMENTATION unset, so anything run natively silently used Fast DDS. Decision 4
# makes both paths first-class and the benchmark ladder compares across them, so two
# different middlewares is not a cosmetic inconsistency — it invalidates the comparison.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ROS_DISTRO_ARTS="${ROS_DISTRO_ARTS:-jazzy}"

# The RMW implementation, named once. Both paths install this package and set
# RMW_IMPLEMENTATION to the matching library name.
DDS_RMW_PACKAGE="ros-${ROS_DISTRO_ARTS}-rmw-cyclonedds-cpp"
DDS_RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
DDS_CONFIG_DEST="/etc/cyclonedds/cyclonedds.xml"

# Deliberately NOT pinned with apt `=version`, for the same reason ruckig is not:
# packages.ros.org is a rolling repository rather than an archive, so an exact version
# string stops being installable once a newer build lands and the pin turns into a
# fresh-machine build failure. See the Ruckig decision record in
# open_source_software/ruckig/VERSION.

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# ─────────────────────────────────────────────────────────────────────
dds_host_deps() {
    echo "[dds] host_deps ..."

    # Kernel receive buffer. CycloneDDS asks the socket for SocketReceiveBufferSize and
    # the kernel silently caps the request at net.core.rmem_max WITHOUT reporting it, so
    # an untuned host looks configured and drops frames under image load. Persisted and
    # applied now, because a benchmark run before the next reboot would otherwise measure
    # the default 208 KiB.
    #
    # This is also why dds_host_deps runs in the Docker path: containers share the host
    # kernel, and network_mode: host means they share these sysctls too.
    local sysctl_f="/etc/sysctl.d/10-cyclonedds.conf"
    if [ ! -f "$sysctl_f" ]; then
        printf 'net.core.rmem_max=26214400\nnet.core.rmem_default=26214400\n' \
            | $SUDO tee "$sysctl_f" >/dev/null
        echo "[dds]   rmem_max/default → $sysctl_f"
    fi
    if [ "$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)" -lt 26214400 ]; then
        $SUDO sysctl --system >/dev/null 2>&1 || true
    fi

    # The config file, from the one copy in the repo.
    $SUDO install -D -m 0644 "$ARTS_ROOT/deploy/cyclonedds.xml" "$DDS_CONFIG_DEST"
    echo "[dds]   config → $DDS_CONFIG_DEST"

    # The RMW package, and the environment that selects it. Only meaningful when ROS is
    # installed natively; a Docker-only host has neither the package nor anything to run.
    if [ ! -d "/opt/ros/$ROS_DISTRO_ARTS" ]; then
        echo "[dds]   no /opt/ros/$ROS_DISTRO_ARTS — skipping the native RMW"
        echo "[dds]   (expected on a Docker-only host; the image carries its own)"
        echo "[dds] host_deps done."
        return 0
    fi

    if ! dpkg -s "$DDS_RMW_PACKAGE" >/dev/null 2>&1; then
        $SUDO apt-get update
        $SUDO apt-get install -y "$DDS_RMW_PACKAGE"
        echo "[dds]   installed $DDS_RMW_PACKAGE"
    fi

    # /etc/profile.d rather than ~/.bashrc: this has to hold for every user and, more to
    # the point, for non-interactive shells. Note profile.d is read by LOGIN shells only,
    # so a service unit or a bare `env -i` shell still needs the variables passed in --
    # arts_controllers/test/run_servo_smoke_test.sh is one such caller.
    local profile_f="/etc/profile.d/arts-dds.sh"
    $SUDO tee "$profile_f" >/dev/null <<EOF
# Written by deploy/modules/dds.sh. ARTS runs CycloneDDS on both the bare-metal and the
# Docker path; see deploy/cyclonedds.xml for why, and for the settings the committed
# benchmarks were measured against.
export RMW_IMPLEMENTATION=$DDS_RMW_IMPLEMENTATION
export CYCLONEDDS_URI=file://$DDS_CONFIG_DEST
EOF
    $SUDO chmod 0644 "$profile_f"
    echo "[dds]   RMW_IMPLEMENTATION + CYCLONEDDS_URI → $profile_f"
    echo "[dds]   (log out and back in, or source it, for it to apply to this session)"

    echo "[dds] host_deps done."
}

# ─────────────────────────────────────────────────────────────────────
# Image-side equivalent. Called from Dockerfile.base so the package name and the config
# file cannot drift from what the host installs.
dds_image_install() {
    echo "[dds] image_install ..."
    apt-get update
    apt-get install -y --no-install-recommends "$DDS_RMW_PACKAGE"
    rm -rf /var/lib/apt/lists/*
    install -D -m 0644 "$ARTS_ROOT/deploy/cyclonedds.xml" "$DDS_CONFIG_DEST"
    echo "[dds] image_install done."
}
