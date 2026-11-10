#!/usr/bin/env bash
# oem_drivers/esp_idf/install.sh — thin wrapper. Real implementation lives in
# deploy/modules/camera_sync.sh so the bare-metal and Docker paths cannot drift
# (AGENTS.md decision 4). Same pattern as oem_drivers/spinnaker/install.sh.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# shellcheck source=../../deploy/modules/camera_sync.sh
source "$ARTS_ROOT/deploy/modules/camera_sync.sh"
camera_sync_oem_install
