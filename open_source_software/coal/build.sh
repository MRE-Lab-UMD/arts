#!/usr/bin/env bash
set -euo pipefail
# Build coal from a pinned tag.
# Called by: arts_install.sh
# Output:    $ARTS_LOCAL/open_source_software/coal/install
#
# coal is the renamed hpp-fcl, an FCL fork. pinocchio's collision support links it, so
# THIS MUST BE BUILT BEFORE pinocchio and pinocchio must be pointed at the same install
# prefix — see open_source_software/pinocchio/build.sh.
#
# Deliberately NOT taken from apt (ros-jazzy-coal exists at 3.0.3). AGENTS.md's pinning
# policy wants a fixed tag for anything whose numbers we publish, and packages.ros.org is
# a rolling repo rather than an archive. The coal API changing between two builds a day
# apart is the incident that created that policy in the first place.
#
# COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL is ON and is NOT optional here. coal is the
# renamed hpp-fcl, but pinocchio v3.9.0 still does `add_project_dependency(hpp-fcl
# REQUIRED)`, so without the shim its configure step fails with "could not find hpp-fcl"
# even though coal is installed and is the very library it wants. The shim installs
# hpp-fclConfig.cmake alongside coalConfig.cmake, both pointing at this build. It is off
# by default upstream.
#
# Consequence worth knowing: this prefix now answers to BOTH names, so installing apt's
# ros-jazzy-hpp-fcl (2.4.5) would put a genuinely different library in /opt/ros/jazzy
# under a name we also claim. That is the Ruckig shadowing shape again, which is why
# arts_controllers asserts the coal version at configure time.
#
# Python bindings are OFF: nothing in the control path uses them and they dominate the
# build time. Turning them on is a full rebuild, so if the deferred instrument-ID work
# ends up wanting them, flip BUILD_PYTHON_INTERFACE and rebuild both coal and pinocchio.
# Octree support is OFF because ARTS collision geometry is capsules and convex meshes;
# it would otherwise need liboctomap-dev.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/VERSION"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
BUILD_DIR="$ARTS_LOCAL/open_source_software/coal"
# Parallel jobs: the cores, but never more than the memory allows (~2 GB per compile job
# for this heavily templated C++). One job per core on a 24-core, 31 GB workstation ran the
# machine out of memory inside the Docker build and the OOM killer took the desktop session
# down (2026-11-08). ARTS_BUILD_JOBS overrides.
MEM_GB=$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)
MEM_JOBS=$(( MEM_GB / 2 > 0 ? MEM_GB / 2 : 1 ))
JOBS="${ARTS_BUILD_JOBS:-$(( $(nproc) < MEM_JOBS ? $(nproc) : MEM_JOBS ))}"
echo "building with $JOBS parallel jobs ($(nproc) cores, ${MEM_GB} GB available)"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d "src/.git" ]; then
    rm -rf src
    # Submodules matter: coal vendors its CMake macros in cmake/ as a submodule, and the
    # configure step fails in a confusing way when it is missing.
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
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DCOAL_HAS_QHULL=ON \
    -DCOAL_HAS_OCTOMAP=OFF \
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON
make -j"$JOBS"
make install

echo "coal $GIT_REF installed to $BUILD_DIR/install"
