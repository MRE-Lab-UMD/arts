#!/usr/bin/env bash
set -euo pipefail
# Build atracsys_ros2_ws — delegates to deploy/modules/_common.sh::overlay_build_generic.
# Requires the fusionTrack SDK already installed via oem_drivers/fusiontrack/install.sh.
# Pins in atracsys_ros2.repos were NOT re-verified for Jazzy; do that at activation.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"
overlay_build_generic atracsys_ros2_ws
