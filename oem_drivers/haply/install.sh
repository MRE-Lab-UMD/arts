#!/usr/bin/env bash
set -euo pipefail
# Install the Haply Hub — delegates to deploy/modules/haply.sh.
#
# Convenience wrapper so a user can run ./oem_drivers/haply/install.sh without knowing the
# module-script machinery. Pin: oem_drivers/haply/VERSION.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# shellcheck source=../../deploy/modules/haply.sh
source "$ARTS_ROOT/deploy/modules/haply.sh"
haply_oem_install
