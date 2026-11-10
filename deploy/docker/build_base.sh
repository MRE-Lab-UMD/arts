#!/usr/bin/env bash
set -euo pipefail
# Build the arts-base:jazzy Docker image.
#
# Stages the middleware module script and the shared CycloneDDS config into
# deploy/docker/.base-staging/ (the Dockerfile.base COPY source). Docker cannot access
# paths outside its build context, and those two files deliberately live outside this
# directory because the bare-metal path uses the same copies -- see deploy/modules/dds.sh.
#
# Usage (from repo root):
#   ./deploy/docker/build_base.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAGING="$SCRIPT_DIR/.base-staging"

cleanup() { rm -rf "$STAGING"; }
trap cleanup EXIT

echo "[base] Staging build context into .base-staging/ ..."
rm -rf "$STAGING"
mkdir -p "$STAGING/deploy/modules"

cp "$ARTS_ROOT/deploy/modules/dds.sh"    "$STAGING/deploy/modules/dds.sh"
cp "$ARTS_ROOT/deploy/cyclonedds.xml"    "$STAGING/deploy/cyclonedds.xml"

echo "[base] Building arts-base:jazzy ..."
docker build -t arts-base:jazzy -f "$SCRIPT_DIR/Dockerfile.base" "$SCRIPT_DIR"

echo "[base] Done. Verify the middleware actually took:"
echo "  docker run --rm arts-base:jazzy bash -lc \\"
echo "    'echo \$RMW_IMPLEMENTATION && ros2 doctor --report | grep -A2 MIDDLEWARE'"
