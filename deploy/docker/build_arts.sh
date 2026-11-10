#!/usr/bin/env bash
set -euo pipefail
# Build the arts:jazzy Docker image (Dockerfile.arts): control libraries, ur_ros2_ws,
# oem_ros2_ws and arts_ros2_ws on top of arts-flir:jazzy.
#
# Stages the module scripts, VERSION/build.sh recipes, .repos manifests and the
# arts_ros2_ws source into deploy/docker/.arts-staging/ (the Dockerfile.arts COPY source)
# and cleans up after. Same pattern as build_flir.sh.
#
# Usage (from repo root):
#   ./deploy/docker/build_arts.sh
#
# Requires arts-flir:jazzy (./deploy/docker/build_flir.sh), which requires arts-base:jazzy.
# arts_install.sh --path docker|both builds all three in that order.
#
# The arts_ros2_ws source is staged from the WORKING TREE (tracked + untracked-but-not-
# ignored files), not from HEAD, so the image builds what a bare-metal build would build
# right now. Ignored trees — the sync-box firmware build output and the fetched ESP-IDF
# micro-ROS component, which carry hundreds of package.xml files — stay out.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAGING="$SCRIPT_DIR/.arts-staging"
BASE_IMAGE="${ARTS_BASE_IMAGE:-arts-flir:jazzy}"

if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "[arts] ERROR: base image $BASE_IMAGE not found."
    echo "[arts]        Build it first: ./deploy/docker/build_base.sh && ./deploy/docker/build_flir.sh"
    exit 1
fi

cleanup() { rm -rf "$STAGING"; }
trap cleanup EXIT

echo "[arts] Staging build context into .arts-staging/ ..."
rm -rf "$STAGING"
mkdir -p "$STAGING/deploy/modules" "$STAGING/open_source_software" \
         "$STAGING/middle_overlay_ros2_wss"

for m in _common arts ur haply; do
    cp "$ARTS_ROOT/deploy/modules/$m.sh" "$STAGING/deploy/modules/$m.sh"
done
for lib in coal pinocchio proxsuite; do
    mkdir -p "$STAGING/open_source_software/$lib"
    cp "$ARTS_ROOT/open_source_software/$lib/VERSION" \
       "$ARTS_ROOT/open_source_software/$lib/build.sh" \
       "$STAGING/open_source_software/$lib/"
done
for ws in ur_ros2_ws oem_ros2_ws; do
    mkdir -p "$STAGING/middle_overlay_ros2_wss/$ws"
    cp "$ARTS_ROOT/middle_overlay_ros2_wss/$ws/"{build.sh,*.repos} \
       "$STAGING/middle_overlay_ros2_wss/$ws/"
done

# arts_ros2_ws/src, working tree, minus ignored files. NUL-separated so odd file names
# survive; --ignore-failed-read skips a tracked file deleted in the working tree.
( cd "$ARTS_ROOT" && git ls-files -z -co --exclude-standard -- arts_ros2_ws/src \
    | tar --null --ignore-failed-read -T - -cf - ) | tar -xf - -C "$STAGING"
echo "[arts]   arts_ros2_ws: $(find "$STAGING/arts_ros2_ws/src" -name package.xml | wc -l) packages," \
     "git $(git -C "$ARTS_ROOT" rev-parse --short HEAD)$(git -C "$ARTS_ROOT" diff --quiet -- arts_ros2_ws/src || echo ' + uncommitted changes')"

echo "[arts] Building arts:jazzy FROM $BASE_IMAGE ..."
docker build -t arts:jazzy --build-arg ARTS_BASE_IMAGE="$BASE_IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile.arts" "$SCRIPT_DIR"

echo "[arts] Done. Smoke test (mock hardware, no arm needed):"
echo "  docker compose -f deploy/docker/compose.yaml run --rm arts \\"
echo "    src/arts_controllers/test/run_servo_smoke_test.sh"
