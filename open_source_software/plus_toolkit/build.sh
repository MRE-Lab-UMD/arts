#!/usr/bin/env bash
set -euo pipefail
# Build Plus Toolkit from a pinned commit.
# Called by: arts_install.sh or deploy/modules/plus_toolkit.sh
# Output:   $ARTS_LOCAL/open_source_software/PlusBuild-bin/

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/VERSION"

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
BUILD_DIR="$ARTS_LOCAL/open_source_software/PlusBuild-bin"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "TODO: implement Plus Toolkit build (see reference/v1.4_ref/Software/PlusBuild-bin and"
echo "reference/install_readme.md). Pin: $GIT_REF from $GIT_URL"
exit 1
