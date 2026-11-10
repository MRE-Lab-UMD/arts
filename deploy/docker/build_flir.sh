#!/usr/bin/env bash
set -euo pipefail
# Build the arts-flir:jazzy Docker image.
#
# Stages the module script, VERSION pin, .repos manifest, and Spinnaker debs
# into deploy/docker/.flir-staging/ (the Dockerfile.flir COPY source).
# Docker cannot access paths outside its build context, so everything the
# Dockerfile needs is copied here first and cleaned up after.
#
# Usage (from repo root):
#   ./deploy/docker/build_flir.sh
#
# Requires:
#   - arts-base:jazzy image already built
#   - Spinnaker SDK debs under $ARTS_LOCAL/oem_drivers/spinnaker-<ver>/
#     (<ver> = the pin, or ARTS_SPINNAKER_VERSION; see oem_drivers/spinnaker/VERSION)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
export ARTS_ROOT ARTS_LOCAL
# The same VERSION handling, deb finder and missing-SDK message as the bare-metal path.
# shellcheck source=../modules/flir.sh
source "$ARTS_ROOT/deploy/modules/flir.sh"   # → VERSION, flir_deb_dir, flir_sdk_missing_msg
STAGING="$SCRIPT_DIR/.flir-staging"

DEB_SRC=$(flir_deb_dir --mirror-only)
if [ -z "$DEB_SRC" ]; then
    flir_sdk_missing_msg
    exit 1
fi
flir_version_notice

cleanup() { rm -rf "$STAGING"; }
trap cleanup EXIT

echo "[FLIR] Staging build context into .flir-staging/ ..."
rm -rf "$STAGING"

mkdir -p "$STAGING/deploy/modules"
mkdir -p "$STAGING/oem_drivers/spinnaker"
mkdir -p "$STAGING/middle_overlay_ros2_wss/flir_ros2_ws"
mkdir -p "$STAGING/spinnaker"

cp "$ARTS_ROOT/deploy/modules/flir.sh" \
   "$STAGING/deploy/modules/flir.sh"
# flir.sh sources the shared overlay-build sequence; the image must run the identical file.
cp "$ARTS_ROOT/deploy/modules/_common.sh" \
   "$STAGING/deploy/modules/_common.sh"
cp "$ARTS_ROOT/oem_drivers/spinnaker/VERSION" \
   "$STAGING/oem_drivers/spinnaker/VERSION"
# The image's copy states the version actually installed, so an override is visible in it.
if [ "$VERSION" != "$PINNED_VERSION" ]; then
    echo "VERSION=$VERSION   # ARTS_SPINNAKER_VERSION override at image build" \
        >> "$STAGING/oem_drivers/spinnaker/VERSION"
fi
cp "$ARTS_ROOT/middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos" \
   "$STAGING/middle_overlay_ros2_wss/flir_ros2_ws/flir_ros2.repos"
cp "$DEB_SRC"/*.deb "$STAGING/spinnaker/"

echo "[FLIR] Building arts-flir:jazzy ..."
docker build -t arts-flir:jazzy -f "$SCRIPT_DIR/Dockerfile.flir" "$SCRIPT_DIR"

echo "[FLIR] Done. Test with:"
echo "  docker run --rm -it --privileged \\"
echo "    -v /dev/bus/usb:/dev/bus/usb \\"
echo "    arts-flir:jazzy \\"
echo "    Enumeration"
echo ""
echo "  (SpinView_QT is unavailable on Ubuntu 24.04 — it needs libSpinVideo,"
echo "   which links the jammy ffmpeg ABI. Enumeration and Acquisition use"
echo "   libSpinnaker only. See deploy/modules/flir.sh.)"
