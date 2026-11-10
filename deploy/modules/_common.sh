#!/usr/bin/env bash
# deploy/modules/_common.sh — the ONE overlay-build sequence, shared by every middle overlay.
#
# Why this file exists (AGENTS.md, "Build order", known defect; docs/repo_structure_triage.md
# item 3): decision 4 promises that bare metal and Docker run the SAME functions so the two
# install paths cannot drift. Until 2026-09-08 that held for FLIR only — five `build.sh`
# scripts each re-implemented `env -i` / source Jazzy / `vcs import` / `rosdep` / `colcon`
# independently, and `arts_install.sh` never called any `<mod>_overlay_build`. The FLIR pilot
# showed what that costs: fixes learned in one copy (the `set -u`-after-source ordering, the
# empty-clone purge, the audible `apt-get update`) never reached the others.
#
# Now: every `middle_overlay_ros2_wss/<ws>/build.sh` and every `<mod>_overlay_build` is a thin
# delegate to `overlay_build_generic`, and `arts_install.sh` goes through `overlay_build_dispatch`.
# Per-overlay differences are ARGUMENTS to the generic function, never a second copy of it.
#
#   overlay_build_generic <ws_name> [options]
#       --ws PATH            workspace root (default $ARTS_LOCAL/middle_overlay_ros2_wss/<ws_name>;
#                            Dockerfiles pass e.g. /opt/flir_ros2_ws)
#       --repos FILE         .repos manifest (default middle_overlay_ros2_wss/<ws_name>/<stem>.repos,
#                            stem = <ws_name> minus the trailing `_ws`)
#       --exclude REL_PATH   package dir under src/ to exclude. Writes COLCON_IGNORE + AMENT_IGNORE
#                            and adds its basename to rosdep --skip-keys. All three are required and
#                            not interchangeable (colcon reads the first, rosdep's crawler the second,
#                            and --skip-keys stops a metapackage exec_depend pulling the prebuilt
#                            copy in through apt). Repeatable.
#       --skip-key KEY       extra rosdep --skip-keys entry. Repeatable.
#       --cmake-arg ARG      extra `colcon build --cmake-args` entry. Repeatable.
#       --no-rosdep          skip rosdep entirely (slicer_ros2_ws: its dependencies are Slicer's
#                            own build tree, which rosdep cannot resolve)
#
#   overlay_build_dispatch <ws_name>
#       What arts_install.sh calls. Prefers the owning module's `<mod>_overlay_build` when one
#       exists (see overlay_owner_module), so any preflight the module adds — flir's refusal to
#       build without the pinned SDK — is on the installer's path too. Falls back to the
#       overlay's build.sh, which is itself a delegate to overlay_build_generic.
#
# Shell requirements, both learned the hard way: this file is bash (Docker's default RUN
# shell is dash and dies on `pipefail`; every module Dockerfile sets SHELL to bash), and the
# inner build shell runs `set -eo pipefail`, sources Jazzy, THEN `set -u` — Jazzy's setup.bash
# reads $AMENT_TRACE_SETUP_FILES without a default and `env -i` guarantees it is unset.
#
# CONTAINER HOUSEKEEPING IS GATED, NOT SHARED. The only image-only step here (dropping build/
# and log/ to shrink the layer) runs when /.dockerenv exists. A function consumed by both paths
# must not carry Dockerfile idioms unconditionally — flir_oem_install once deleted the host's
# apt lists that way (AGENTS.md, FLIR Stage 5, 2026-08-18).

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
ARTS_MINIMAL_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Root inside a Docker build, an ordinary user on bare metal; `sudo` is usually absent from
# an image, so escalate only when needed.
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# ─────────────────────────────────────────────────────────────────────
# Which module owns which overlay. The TABLE that triage item 4 asked for: one place where
# "module" maps onto a workspace, readable without parsing anything. Overlays with no owner
# yet (slicer, atracsys, isaac) are built by their build.sh delegate; give them an owner
# here when their deploy/modules/<mod>.sh is written. ur got its owner 2026-11-01 (ur.sh
# exists since 2026-10-29; its build.sh already delegated to ur_overlay_build, so the
# function run is unchanged, only the route is shorter). oem_ros2_ws is deliberately unowned —
# it is shared by every loose OEM wrapper (micro-ROS agent today, Haply/Zivid later), so no
# single module may claim it; camera_sync_overlay_build delegates here rather than owning.
overlay_owner_module() {
    case "$1" in
        flir_ros2_ws) echo flir ;;
        ur_ros2_ws)   echo ur ;;
        *) echo "" ;;
    esac
}

# ─────────────────────────────────────────────────────────────────────
overlay_build_generic() {
    local ws_name="${1:-}"
    [ -n "$ws_name" ] || { echo "overlay_build_generic: workspace name required" >&2; return 2; }
    shift

    local ws="$ARTS_LOCAL/middle_overlay_ros2_wss/$ws_name"
    local repos="$ARTS_ROOT/middle_overlay_ros2_wss/$ws_name/${ws_name%_ws}.repos"
    local -a excludes=() skip_keys=() cmake_args=()
    local rosdep=1

    while [ $# -gt 0 ]; do
        case "$1" in
            --ws)        ws="$2"; shift 2 ;;
            --repos)     repos="$2"; shift 2 ;;
            --exclude)   excludes+=("$2"); skip_keys+=("$(basename "$2")"); shift 2 ;;
            --skip-key)  skip_keys+=("$2"); shift 2 ;;
            --cmake-arg) cmake_args+=("$2"); shift 2 ;;
            --no-rosdep) rosdep=0; shift ;;
            *) echo "overlay_build_generic: unknown option '$1'" >&2; return 2 ;;
        esac
    done

    if [ ! -f "$repos" ]; then
        echo "[$ws_name] ERROR: manifest not found: $repos" >&2
        return 1
    fi
    # An overlay that is not activated yet keeps `repositories: {}`; that is a valid state,
    # not a failure, so the full-chain install does not stop on it.
    if grep -qE '^\s*repositories:\s*\{\s*\}\s*$' "$repos"; then
        echo "[$ws_name] $(basename "$repos") is an empty placeholder — nothing to build yet."
        return 0
    fi

    echo "[$ws_name] overlay_build: $ws"
    echo "[$ws_name]   manifest: $repos"
    [ ${#excludes[@]} -gt 0 ]   && echo "[$ws_name]   exclude:  ${excludes[*]}"
    [ ${#skip_keys[@]} -gt 0 ]  && echo "[$ws_name]   skip-keys: ${skip_keys[*]}"
    [ ${#cmake_args[@]} -gt 0 ] && echo "[$ws_name]   cmake:    ${cmake_args[*]}"
    [ "$rosdep" = 0 ]           && echo "[$ws_name]   rosdep:   skipped"

    mkdir -p "$ws/src"

    # Everything the inner shell needs crosses the `env -i` boundary as VARIABLES, so no path
    # is ever spliced into a code string (the old scripts did that with single quotes, which
    # breaks on the first path containing one). Lists are newline-separated.
    env -i HOME="${HOME:-/root}" USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" \
           PATH="$ARTS_MINIMAL_PATH" \
           OB_NAME="$ws_name" OB_WS="$ws" OB_REPOS="$repos" OB_SUDO="$SUDO" \
           OB_ROSDEP="$rosdep" \
           OB_EXCLUDES="$(printf '%s\n' "${excludes[@]+"${excludes[@]}"}")" \
           OB_SKIP_KEYS="$(printf '%s\n' "${skip_keys[@]+"${skip_keys[@]}"}")" \
           OB_CMAKE_ARGS="$(printf '%s\n' "${cmake_args[@]+"${cmake_args[@]}"}")" \
      bash -c '
        set -eo pipefail
        # Jazzy setup.bash reads $AMENT_TRACE_SETUP_FILES with no default; -u only AFTER it.
        source /opt/ros/jazzy/setup.bash
        set -u
        cd "$OB_WS"

        # A failed import (no network, interrupted clone) leaves an EMPTY directory behind,
        # and --skip-existing then skips it forever — the package silently never appears and
        # colcon reports success with one package missing. Clear empties first.
        find src -mindepth 1 -maxdepth 1 -type d -empty -delete 2>/dev/null || true
        vcs import src --skip-existing --input "$OB_REPOS"

        # --skip-existing also skips a clone whose pin has since been BUMPED: the old
        # checkout stays and builds, and the .repos file lies about it (caught 2026-10-29,
        # UR driver 3.8.0 → 3.10.0). Move every existing clone to its pin, and say so; git
        # refuses, and the build stops, if the clone has local edits.
        python3 -c "import sys, yaml
for name, r in yaml.safe_load(open(sys.argv[1]))[\"repositories\"].items():
    print(name, r[\"version\"])" "$OB_REPOS" |
        while read -r name version; do
          have=$(git -C "src/$name" rev-parse HEAD)
          want=$(git -C "src/$name" rev-parse -q --verify "$version^{commit}" 2>/dev/null) || {
            git -C "src/$name" fetch -q --tags origin
            want=$(git -C "src/$name" rev-parse "$version^{commit}")
          }
          if [ "$have" != "$want" ]; then
            git -C "src/$name" checkout -q "$want"
            echo "[$OB_NAME]   $name moved ${have:0:8} → ${want:0:8} ($version)"
          fi
        done

        while IFS= read -r rel; do
          [ -n "$rel" ] || continue
          if [ -d "src/$rel" ]; then
            touch "src/$rel/COLCON_IGNORE" "src/$rel/AMENT_IGNORE"
            echo "[$OB_NAME]   excluded src/$rel (COLCON_IGNORE + AMENT_IGNORE)"
          fi
        done <<< "$OB_EXCLUDES"

        if [ "$OB_ROSDEP" = 1 ]; then
          # Needs sudo on bare metal, and the failure must be AUDIBLE: the original form was
          # `apt-get update 2>/dev/null || true` with no sudo, so on a host it always failed
          # silently, and an empty apt list then surfaced as rosdep naming a perfectly
          # installable package as unavailable (2026-08-18).
          $OB_SUDO apt-get update \
            || echo "[$OB_NAME]   WARNING: apt-get update failed; rosdep will search a stale or empty package list."
          rosdep update --rosdistro=jazzy 2>/dev/null || true
          skip=""
          while IFS= read -r k; do
            [ -n "$k" ] && skip="${skip:+$skip }$k"
          done <<< "$OB_SKIP_KEYS"
          if [ -n "$skip" ]; then
            rosdep install --ignore-src --from-paths src -y --skip-keys "$skip"
          else
            rosdep install --ignore-src --from-paths src -y
          fi
        fi

        cmake=(-DCMAKE_BUILD_TYPE=Release)
        while IFS= read -r a; do
          [ -n "$a" ] && cmake+=("$a")
        done <<< "$OB_CMAKE_ARGS"
        colcon build --cmake-args "${cmake[@]}"

        # Image-only: drop intermediates to keep the layer small. On a host they are the
        # incremental-rebuild state and stay.
        if [ -f /.dockerenv ]; then
          rm -rf build log
        fi

        echo ">>> Finished building $OB_NAME at $OB_WS"
      '
}

# ─────────────────────────────────────────────────────────────────────
overlay_build_dispatch() {
    local ws_name="${1:-}"
    [ -n "$ws_name" ] || { echo "overlay_build_dispatch: workspace name required" >&2; return 2; }

    local mod
    mod="$(overlay_owner_module "$ws_name")"
    if [ -n "$mod" ] && [ -f "$ARTS_ROOT/deploy/modules/$mod.sh" ]; then
        echo "[$ws_name] via module function ${mod}_overlay_build"
        (
            # shellcheck source=/dev/null
            source "$ARTS_ROOT/deploy/modules/$mod.sh"
            if declare -f "${mod}_overlay_build" >/dev/null 2>&1; then
                "${mod}_overlay_build"
            else
                echo "[$ws_name] ERROR: $mod.sh defines no ${mod}_overlay_build" >&2
                exit 1
            fi
        )
        return
    fi

    local script="$ARTS_ROOT/middle_overlay_ros2_wss/$ws_name/build.sh"
    if [ ! -f "$script" ]; then
        echo "[$ws_name] ERROR: no owning module and no $script" >&2
        return 1
    fi
    echo "[$ws_name] via $script"
    bash "$script"
}
