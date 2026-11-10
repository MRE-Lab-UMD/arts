#!/usr/bin/env bash
set -euo pipefail
# Build 3D Slicer Superbuild from a pinned commit.
# Called by: arts_install.sh or deploy/modules/slicer.sh
# Output:   $ARTS_LOCAL/open_source_software/Slicer-Superbuild-Release/

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/VERSION"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
BUILD_DIR="$ARTS_LOCAL/open_source_software/Slicer-Superbuild-Release"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d "Slicer" ]; then
    git clone "$GIT_URL" Slicer
fi

cd Slicer
git fetch origin
git checkout "$GIT_REF"
cd ..

mkdir -p build && cd build
# TODO: fill in the verified Slicer cmake flags from the reference install
cmake ../Slicer \
    -DCMAKE_BUILD_TYPE=Release \
    -DSlicer_USE_PYTHONQT=ON
make -j"$(nproc)"

echo "Slicer superbuild at $BUILD_DIR/build"
echo "Pass -DSlicer_DIR=.../Slicer-build when building slicer_ros2_ws"
