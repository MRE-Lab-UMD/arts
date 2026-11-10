#!/usr/bin/env bash
set -euo pipefail
# Build isaac_ros2_ws — delegates to deploy/modules/_common.sh::overlay_build_generic.
# Pins in isaac_ros2.repos were NOT re-verified for Jazzy; do that at activation.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"
overlay_build_generic isaac_ros2_ws
