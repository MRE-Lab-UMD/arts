#!/usr/bin/env bash
set -euo pipefail
# Build flir_ros2_ws — delegates to deploy/modules/flir.sh::flir_overlay_build(), which adds
# the pinned-Spinnaker preflight and then runs the shared overlay_build_generic.
#
# arts_install.sh reaches the same function through overlay_build_dispatch (flir is the
# overlay's owning module in _common.sh), so this file is the manual one-overlay entry point.
# Workspace output: $ARTS_LOCAL/middle_overlay_ros2_wss/flir_ros2_ws/

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/flir.sh
source "$ARTS_ROOT/deploy/modules/flir.sh"
flir_overlay_build
