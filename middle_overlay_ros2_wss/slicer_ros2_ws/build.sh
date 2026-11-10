#!/usr/bin/env bash
set -euo pipefail
# Build slicer_ros2_ws — delegates to deploy/modules/_common.sh::overlay_build_generic.
#
# Two deviations from the plain sequence, both preserved from the original script and both
# to be re-examined when this overlay is activated (Phase 5): rosdep is skipped because
# SlicerROS2's dependencies are Slicer's own superbuild tree, which rosdep cannot resolve,
# and Slicer_DIR points colcon at that tree (open_source_software/slicer, pinned v5.8.1).

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

SLICER_DIR="$ARTS_LOCAL/open_source_software/Slicer-Superbuild-Release/build/Slicer-build"

overlay_build_generic slicer_ros2_ws \
    --no-rosdep \
    --cmake-arg "-DSlicer_DIR=$SLICER_DIR"
