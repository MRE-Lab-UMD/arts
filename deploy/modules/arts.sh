#!/usr/bin/env bash
# deploy/modules/arts.sh — the top of the chain: the pinned control libraries (coal →
# pinocchio → proxsuite) and the arts_ros2_ws overlay itself.
# Consumed by BOTH arts_install.sh (bare metal) and Dockerfile.arts (container).
#
# Functions:
#   arts_build_deps          — apt packages the control libraries and arts_ros2_ws compile
#                              against, pinned with `=version` (Ubuntu archive, not the
#                              rolling packages.ros.org). Until 2026-11-01 these were declared
#                              nowhere (AGENTS.md Phase 0, "known gap"); the workstation had
#                              them only because someone installed them by hand.
#   arts_control_libs_build  — open_source_software/{coal,pinocchio,proxsuite}/build.sh, each
#                              in its own `env -i` shell, skipping a library already installed
#   arts_ws_build            — arts_ros2_ws against the middle overlays present in
#                              $ARTS_LOCAL, with the control-library prefixes prepended to
#                              CMAKE_PREFIX_PATH (the sourcing rules in AGENTS.md)
#
#   ./deploy/modules/arts.sh {build_deps|control_libs|ws}
#
# NOT A HARDWARE MODULE, like dds.sh. It lives here because deploy/modules/*.sh is the one
# place both install paths read from, and arts_install.sh's docker branch finds
# deploy/docker/build_arts.sh through the module name. It defines no arts_host_deps on
# purpose: a Docker-only host must stay minimal, and these are build dependencies.
#
# CONTAINER HOUSEKEEPING IS GATED on /.dockerenv (hard rule 4): inside an image the
# control-library source and build trees are deleted after install; on a host they are the
# incremental-rebuild state and stay.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"

# Sets SUDO (empty as root, "sudo" otherwise) and ARTS_MINIMAL_PATH.
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

# Build order matters: pinocchio links coal (see pinocchio/build.sh).
ARTS_CONTROL_LIBS="coal pinocchio proxsuite"

# ─────────────────────────────────────────────────────────────────────
arts_build_deps() {
    echo "[arts] build_deps ..."
    # What each is for, so a later reader can drop one with confidence:
    #   libeigen3-dev        pinocchio, proxsuite, arts_controllers (find_package Eigen3 3.4)
    #   libboost-*-dev       coal (chrono, thread, date_time, filesystem, serialization) and
    #                        pinocchio (filesystem, serialization, system)
    #   libassimp-dev        coal mesh loading
    #   libqhull-dev         coal, built with COAL_HAS_QHULL=ON
    #   libopencv-dev        arts_display (also a rosdep key there; pinned here so rosdep
    #                        finds it installed instead of taking whatever is current)
    # Versions are the noble archive candidates on 2026-11-01, identical on the workstation
    # and in ros:jazzy-ros-base-noble. git, cmake and a compiler come with the base image
    # and with any ROS development host.
    $SUDO apt-get install -y --no-install-recommends \
        libeigen3-dev=3.4.0-4build0.1 \
        libboost-dev=1.83.0.1ubuntu2 \
        libboost-chrono-dev=1.83.0.1ubuntu2 \
        libboost-date-time-dev=1.83.0.1ubuntu2 \
        libboost-filesystem-dev=1.83.0.1ubuntu2 \
        libboost-serialization-dev=1.83.0.1ubuntu2 \
        libboost-system-dev=1.83.0.1ubuntu2 \
        libboost-thread-dev=1.83.0.1ubuntu2 \
        libassimp-dev=5.3.1+ds-2build1 \
        libqhull-dev=2020.2-6build1 \
        libopencv-dev=4.6.0+dfsg-13.1ubuntu1
    echo "[arts] build_deps done."
}

# ─────────────────────────────────────────────────────────────────────
arts_control_libs_build() {
    local lib prefix
    for lib in $ARTS_CONTROL_LIBS; do
        prefix="$ARTS_LOCAL/open_source_software/$lib"
        if [ -d "$prefix/install" ]; then
            echo "[$lib] already installed — skipping"
            continue
        fi
        echo "============================================================"
        echo " Building open_source_software/$lib"
        echo "============================================================"
        env -i HOME="${HOME:-/root}" USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" \
          PATH="$ARTS_MINIMAL_PATH" ARTS_LOCAL="$ARTS_LOCAL" \
          ${ARTS_BUILD_JOBS:+ARTS_BUILD_JOBS="$ARTS_BUILD_JOBS"} \
          bash "$ARTS_ROOT/open_source_software/$lib/build.sh"
        # Image-only: the clone and its build tree are hundreds of MB and nothing reads them
        # after install. pinocchio's RUNPATH points at coal's install/lib, which stays.
        if [ -f /.dockerenv ]; then
            rm -rf "$prefix/src"
        fi
    done
}

# ─────────────────────────────────────────────────────────────────────
arts_ws_build() {
    echo "============================================================"
    echo " Building arts_ros2_ws (top overlay)"
    echo "============================================================"
    env -i HOME="${HOME:-/root}" USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" \
      PATH="$ARTS_MINIMAL_PATH" ARTS_LOCAL="$ARTS_LOCAL" ARTS_ROOT="$ARTS_ROOT" \
      ARTS_CONTROL_LIBS="$ARTS_CONTROL_LIBS" \
      bash -c '
        set -eo pipefail   # ROS setup files read unset vars; -u goes on after sourcing
        found=0
        skip_keys=""
        for ws in ur_ros2_ws flir_ros2_ws slicer_ros2_ws atracsys_ros2_ws isaac_ros2_ws oem_ros2_ws; do
          setup="$ARTS_LOCAL/middle_overlay_ros2_wss/$ws/install/setup.bash"
          if [ -f "$setup" ]; then
            echo "[arts]   underlay: $ws"
            # shellcheck disable=SC1090
            source "$setup"
            found=1
          elif [ "$ws" = flir_ros2_ws ]; then
            # No-camera install (ARTS_ENABLE_FLIR=0). arts_driver exec-depends on
            # spinnaker_camera_driver, and rosdep would satisfy that key with the apt binary
            # ros-jazzy-spinnaker-camera-driver, which ships its own libSpinnaker.so.4.4.0.99 into
            # /opt/ros/jazzy/lib (3.0.5-1noble.20260912, checked 2026-11-09 with dpkg -c): an
            # unpinned SDK through the back door flir_overlay_build refuses. Skip the key; only
            # cameras.launch.py (ecm_stereo:=true) needs the package.
            echo "[arts]   no flir_ros2_ws: skipping rosdep key spinnaker_camera_driver (no cameras)"
            skip_keys="spinnaker_camera_driver"
          fi
        done
        # Each middle overlay chains to /opt/ros/jazzy on its own, so sourcing Jazzy
        # as well would duplicate AMENT_PREFIX_PATH entries. Source it only as the
        # fallback for a build with no middle overlay present at all.
        if [ "$found" = 0 ]; then
          source /opt/ros/jazzy/setup.bash
        fi
        set -u

        cd "$ARTS_ROOT/arts_ros2_ws"
        rosdep update --rosdistro=jazzy 2>/dev/null || true
        rosdep install --from-paths src --ignore-src -y ${skip_keys:+--skip-keys "$skip_keys"}

        # The from-source non-ROS libraries live outside any ament prefix, so sourcing
        # overlays does not reveal them. Prepend them explicitly, ahead of the underlay,
        # so that find_package() prefers the pinned build over any apt copy that shares
        # the name -- apt ships pinocchio 4.1.0 and coal 3.0.3 (measured 2026-11-01), both
        # of which would otherwise win simply by being in /opt/ros/jazzy.
        for lib in $ARTS_CONTROL_LIBS; do
          prefix="$ARTS_LOCAL/open_source_software/$lib/install"
          if [ -d "$prefix" ]; then
            CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
          fi
        done
        export CMAKE_PREFIX_PATH

        # Bounded by memory as well as cores: one package at a time (colcon would build several
        # at once, each with its own jobs), arts_controllers includes the heavy pinocchio
        # templates. Unbounded, a 24-core / 31 GB machine ran out of memory (2026-11-08,
        # Docker build). ARTS_BUILD_JOBS overrides the total.
        mem_gb=$(( $(grep MemAvailable /proc/meminfo | tr -s " " | cut -d" " -f2) / 1048576 ))
        jobs=${ARTS_BUILD_JOBS:-$(( mem_gb / 3 > 0 ? mem_gb / 3 : 1 ))}
        [ "$jobs" -gt "$(nproc)" ] && jobs=$(nproc)
        echo "colcon: $jobs jobs in total ($(nproc) cores, ${mem_gb} GB available)"
        MAKEFLAGS="-j$jobs" colcon build --symlink-install --executor sequential \
          --cmake-args -DCMAKE_BUILD_TYPE=Release

        # Image-only: logs. build/ stays, because --symlink-install points the Python
        # packages and the test binaries (task_qp_test) into it.
        if [ -f /.dockerenv ]; then
          rm -rf log
        fi
        echo ">>> Finished building arts_ros2_ws"
        echo "    source $ARTS_ROOT/arts_ros2_ws/install/setup.bash"
      '
}

# ─────────────────────────────────────────────────────────────────────
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        build_deps)   arts_build_deps ;;
        control_libs) arts_control_libs_build ;;
        ws)           arts_ws_build ;;
        *) echo "Usage: $0 {build_deps|control_libs|ws}"; exit 1 ;;
    esac
fi
