#!/usr/bin/env bash
set -euo pipefail
# Build oem_ros2_ws — delegates to deploy/modules/_common.sh::overlay_build_generic.
#
# This overlay has NO owning module on purpose: it collects loose OEM wrappers that ship
# without a workspace of their own (micro-ROS agent for arts_camera_sync today; Haply, Zivid,
# RealSense later), so no single deploy/modules/<mod>.sh may claim it. Module functions that
# need it (camera_sync_overlay_build) delegate to the same generic call as this file.
# An empty `repositories: {}` manifest is a valid placeholder and builds nothing.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"
overlay_build_generic oem_ros2_ws
