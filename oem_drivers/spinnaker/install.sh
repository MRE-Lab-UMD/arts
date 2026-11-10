#!/usr/bin/env bash
set -euo pipefail
# Install FLIR Spinnaker SDK — delegates to deploy/modules/flir.sh.
#
# This is a convenience wrapper so a user can run:
#   sudo ./oem_drivers/spinnaker/install.sh
# without knowing the module-script machinery.
#
# For the SDK download, see the error message or oem_drivers/spinnaker/VERSION.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/flir.sh
source "$ARTS_ROOT/deploy/modules/flir.sh"
flir_oem_install
