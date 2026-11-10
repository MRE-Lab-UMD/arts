#!/usr/bin/env bash
set -euo pipefail
# Build pinocchio from a pinned tag.
# Called by: arts_install.sh
# Output:    $ARTS_LOCAL/open_source_software/pinocchio/install
#
# REQUIRES coal TO BE BUILT FIRST. pinocchio's collision support links coal, and the
# version pinned here needs coal >= 3.0.0. Earlier revisions of this script did not wire
# the two together at all, so pinocchio would silently configure without collision
# support and Phase 2 would have discovered it much later.
#
# Deliberately NOT taken from apt. ros-jazzy-pinocchio is at 4.0.0 — a MAJOR version
# ahead of this pin, with breaking API changes — which is the opposite of the situation
# that sent Ruckig to apt (frozen at one version across every distro, nothing to gain
# from a newer one). Here the library is moving, it produces numbers we publish, and
# apt's pinocchio would drag apt's coal into /opt/ros/jazzy where it would shadow the
# pinned coal. See open_source_software/ruckig/VERSION for the contrasting case.
#
# Python bindings are OFF; see the note in coal/build.sh for the reasoning and the cost
# of changing your mind.
#
# CMAKE_INSTALL_RPATH_USE_LINK_PATH is ON so libpinocchio records where libcoal lives.
# It is not optional and the reason is easy to miss: the linker emits RUNPATH rather than
# RPATH, and RUNPATH is NOT consulted for a dependency's own dependencies. So a consumer
# that lists coal's directory in its own RUNPATH still fails to load pinocchio, because
# the loader resolves libcoal using pinocchio's search path and not the consumer's. The
# symptom is a library that links cleanly and then cannot be dlopen'd.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/VERSION"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
BUILD_DIR="$ARTS_LOCAL/open_source_software/pinocchio"
COAL_PREFIX="$ARTS_LOCAL/open_source_software/coal/install"
# Parallel jobs: the cores, but never more than the memory allows (~4 GB per compile job
# for this heavily templated C++). One job per core on a 24-core, 31 GB workstation ran the
# machine out of memory inside the Docker build and the OOM killer took the desktop session
# down (2026-11-08). ARTS_BUILD_JOBS overrides.
MEM_GB=$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)
MEM_JOBS=$(( MEM_GB / 4 > 0 ? MEM_GB / 4 : 1 ))
JOBS="${ARTS_BUILD_JOBS:-$(( $(nproc) < MEM_JOBS ? $(nproc) : MEM_JOBS ))}"
echo "building with $JOBS parallel jobs ($(nproc) cores, ${MEM_GB} GB available)"

if [ ! -f "$COAL_PREFIX/lib/cmake/coal/coalConfig.cmake" ]; then
    echo "ERROR: coal is not installed at $COAL_PREFIX" >&2
    echo "       Run open_source_software/coal/build.sh first." >&2
    exit 1
fi

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
# CMAKE_PREFIX_PATH rather than a coal_DIR hint, so coal's own transitive dependencies
# resolve from the same prefix.
#
# /opt/ros/jazzy is on the path DELIBERATELY and must stay ahead of /usr. URDF support
# needs urdfdom, and there are two on this machine: liburdfdom-headers-dev 1.0.5 under
# /usr, and ros-jazzy-urdfdom 4.0.2 (headers 1.1.3) under /opt/ros/jazzy. The controller
# links the ROS one at runtime because the underlay is always on its prefix path, so
# pinocchio has to be built against the same one or the two disagree on urdf_model types
# across a library boundary. This is the one case where reaching into the underlay from
# a from-source build is correct rather than sloppy.
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$BUILD_DIR/install" \
    -DCMAKE_PREFIX_PATH="$COAL_PREFIX;/opt/ros/jazzy" \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_COLLISION_SUPPORT=ON
make -j"$JOBS"
make install

echo "pinocchio $GIT_REF installed to $BUILD_DIR/install (collision support via coal)"
