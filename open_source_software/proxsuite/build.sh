#!/usr/bin/env bash
set -euo pipefail
# Build proxsuite (ProxQP) from a pinned tag.
# Called by: arts_install.sh (opt-in, ARTS_ENABLE_PROXSUITE=1, until the RCM task layer is
#            verified on a real arm)
# Output:    $ARTS_LOCAL/open_source_software/proxsuite/install
#
# Used by arts_controllers' task layer (decision 17): one small dense QP per control cycle,
# the RCM as an equality constraint, joint limits as bounds. Header-only for C++, so the
# install is just headers + CMake config.
#
# BUILD_WITH_VECTORIZATION_SUPPORT is OFF. ON needs SIMDe (libsimde-dev from apt, an
# unpinned host package) and buys nothing for a 6-9 variable problem; the dense solver is
# plain Eigen without it.
#
# The CMake macros (jrl-cmakemodules) are a git submodule, so the tag pins them too; the
# clone must be recursive or configure fails with a missing cmake/base.cmake.
#
# Deliberately NOT apt's ros-jazzy-proxsuite (0.6.5, rolling repo): same reasoning as coal
# and pinocchio, and arts_controllers asserts the version at configure time.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/VERSION"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
BUILD_DIR="$ARTS_LOCAL/open_source_software/proxsuite"
JOBS="${ARTS_BUILD_JOBS:-$(nproc)}"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d "src/.git" ]; then
    rm -rf src
    git clone --recurse-submodules "$GIT_URL" src
fi

cd src
git fetch --tags origin
git checkout --force "$GIT_REF"
git submodule update --init --recursive

mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$BUILD_DIR/install" \
    -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARK=OFF \
    -DBUILD_DOCUMENTATION=OFF
make -j"$JOBS"
make install

echo "proxsuite $GIT_REF installed to $BUILD_DIR/install"
