#!/usr/bin/env bash
# arts_install.sh — thin orchestrator over deploy/modules/*.sh
#
# Build order (each middle overlay in its own env -i shell, all through
# deploy/modules/_common.sh::overlay_build_generic):
#   /opt/ros/jazzy  → ur_ros2_ws → flir_ros2_ws → [slicer_ros2_ws]
#                   → [atracsys_ros2_ws] → [isaac_ros2_ws] → oem_ros2_ws
#                   → coal → pinocchio → proxsuite
#                   → arts_ros2_ws → example_app_ros2_ws
# [bracketed] steps are opt-in: ARTS_ENABLE_SLICER / _ATRACSYS / _ISAAC=1.
# flir_ros2_ws (the stereo cameras) is opt-out: ARTS_ENABLE_FLIR=0 skips it and its
# account-walled Spinnaker SDK. The mock robot and simulation need neither.
#
# Install paths (AGENTS.md decision 4 — both are first-class, neither optional):
#   bare    build everything natively into $ARTS_LOCAL; no Docker required
#   docker  build per-module images; the host stays minimal
#   both    what the development machine runs
# Chosen once and remembered in $ARTS_LOCAL/install_path.  host_deps() always
# runs on the host in every path: udev rules, usbfs, and sysctls are kernel-side
# and a container cannot set them.
#
# Usage:
#   ./arts_install.sh                 # full chain (asks for the path on first run)
#   ./arts_install.sh --path docker   # force a path (bare|docker|both)
#   ./arts_install.sh --modules flir  # only these modules (default: all present)
#   ./arts_install.sh --overlay ur    # one middle overlay
#   ./arts_install.sh --arts-only     # rebuild arts_ros2_ws only
#   ./arts_install.sh --prereqs       # host prerequisite walkthrough only
#   ./arts_install.sh --skip-prereqs  # full chain, no prerequisite walkthrough
#   ARTS_ENABLE_FLIR=0 ./arts_install.sh          # no cameras: no Spinnaker SDK needed
#   ARTS_SPINNAKER_VERSION=x.y.z.w ./arts_install.sh   # a non-pinned SDK (reported loudly)
#
set -euo pipefail

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
MINIMAL_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

export ARTS_ROOT ARTS_LOCAL

# The one overlay-build sequence (overlay_build_generic / overlay_build_dispatch). Every
# middle overlay — installer, build.sh, module function, Dockerfile — goes through it.
# shellcheck source=deploy/modules/_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

usage() {
    sed -n '2,31p' "$0" | sed 's/^# \?//'
}

# Machine-dependent choices (GPU driver branch, RT kernel, isolcpus map, account-walled
# SDKs) cannot be pinned, so they are surfaced interactively instead of guessed at.
run_prereqs() {
    "$ARTS_ROOT/deploy/prereqs.sh" "$@"
}

PATH_FILE="$ARTS_LOCAL/install_path"

# Precedence: --path flag, then $ARTS_INSTALL_PATH, then the remembered choice,
# then ask.  Keeping the answer makes re-runs idempotent and CI non-interactive.
resolve_install_path() {
    if [ -n "$INSTALL_PATH" ]; then
        :
    elif [ -n "${ARTS_INSTALL_PATH:-}" ]; then
        INSTALL_PATH="$ARTS_INSTALL_PATH"
    elif [ -f "$PATH_FILE" ]; then
        INSTALL_PATH="$(cat "$PATH_FILE")"
        echo "Install path: $INSTALL_PATH (remembered — override with --path)"
    else
        echo
        echo "ARTS supports two install paths, and both are fully supported."
        echo
        echo "  bare    build natively into $ARTS_LOCAL; no Docker needed"
        echo "  docker  build per-module images; host stays minimal"
        echo "  both    bare metal plus images (the development machine)"
        echo
        local reply
        read -r -p "Choose [bare/docker/both]: " reply
        INSTALL_PATH="${reply:-bare}"
    fi

    case "$INSTALL_PATH" in
        bare|docker|both) ;;
        *) echo "ERROR: --path must be bare, docker, or both (got '$INSTALL_PATH')"; exit 1 ;;
    esac

    mkdir -p "$ARTS_LOCAL"
    echo "$INSTALL_PATH" > "$PATH_FILE"
}

path_has() {
    [ "$INSTALL_PATH" = "both" ] || [ "$INSTALL_PATH" = "$1" ]
}

# Modules are the hardware units with a deploy/modules/<name>.sh script.
# Default is every script present; --modules narrows it. Files starting with `_`
# (_common.sh) are shared libraries, not modules.
flir_enabled() { [ "${ARTS_ENABLE_FLIR:-1}" != 0 ]; }

selected_modules() {
    if [ -n "$MODULES" ]; then
        tr ',' ' ' <<< "$MODULES"
        return
    fi
    local m
    for m in "$ARTS_ROOT"/deploy/modules/[!_]*.sh; do
        [ -f "$m" ] || continue
        m=$(basename "$m" .sh)
        [ "$m" = flir ] && ! flir_enabled && continue
        echo "$m"
    done
}

# Before anything long runs: the FLIR module needs the account-walled SDK, so say so now
# rather than after the host packages and (Docker) the base image, and offer the no-camera
# install. Same finder and message as flir_oem_install / build_flir.sh.
flir_sdk_preflight() {
    selected_modules | grep -qx flir || return 0
    (
        # shellcheck source=deploy/modules/flir.sh
        source "$ARTS_ROOT/deploy/modules/flir.sh"
        if [ -z "$(flir_deb_dir --mirror-only)" ]; then
            flir_sdk_missing_msg
            exit 1
        fi
        flir_version_notice
    )
}

build_docker_images() {
    if ! command -v docker >/dev/null 2>&1; then
        echo "ERROR: docker path selected but docker is not installed."
        echo "       Run ./arts_install.sh --prereqs first."
        exit 1
    fi

    echo "============================================================"
    echo " Building arts-base:jazzy"
    echo "============================================================"
    bash "$ARTS_ROOT/deploy/docker/build_base.sh"

    # arts goes LAST whatever the module order: arts:jazzy is built FROM arts-flir:jazzy
    # (the full overlay chain, as on bare metal), so it needs that image to exist. Without
    # FLIR it is built FROM arts-base:jazzy instead (no Spinnaker, no flir_ros2_ws).
    if ! selected_modules | grep -qx flir; then
        export ARTS_BASE_IMAGE="${ARTS_BASE_IMAGE:-arts-base:jazzy}"
    fi
    local mod builder
    for mod in $(selected_modules | grep -vx arts) $(selected_modules | grep -x arts); do
        builder="$ARTS_ROOT/deploy/docker/build_${mod}.sh"
        if [ -f "$builder" ]; then
            echo "============================================================"
            echo " Building image for module: $mod"
            echo "============================================================"
            bash "$builder"
        else
            echo "[$mod] no deploy/docker/build_${mod}.sh yet — skipping image"
        fi
    done
}

# One entry point per overlay, and it is the SAME one the Dockerfiles use: dispatch prefers
# the owning module's <mod>_overlay_build (flir → flir_overlay_build, with its Spinnaker
# preflight) and otherwise runs the overlay's build.sh, itself a delegate to
# overlay_build_generic. Until 2026-09-08 this function ran build.sh unconditionally and
# never reached a module function — see AGENTS.md "Build order", known defect.
build_middle_overlay() {
    local name="$1"
    echo "============================================================"
    echo " Building middle overlay: $name"
    echo " Artifacts → $ARTS_LOCAL/middle_overlay_ros2_wss/$name"
    echo "============================================================"
    overlay_build_dispatch "$name"
}

# Overlays the default full install builds. slicer, atracsys and isaac are opt-in
# (ARTS_ENABLE_<NAME>=1) until their pins are verified on Jazzy; `--overlay <name>`
# still builds any of them explicitly.
default_overlays() {
    echo ur_ros2_ws
    flir_enabled && echo flir_ros2_ws
    [ "${ARTS_ENABLE_SLICER:-0}" = 1 ]   && echo slicer_ros2_ws
    [ "${ARTS_ENABLE_ATRACSYS:-0}" = 1 ] && echo atracsys_ros2_ws
    [ "${ARTS_ENABLE_ISAAC:-0}" = 1 ]    && echo isaac_ros2_ws
    echo oem_ros2_ws
}

# coal before pinocchio (pinocchio links it); arts_controllers asserts all three versions at
# configure time. proxsuite is the task-layer QP (decision 17, the RCM) and is required since
# 2026-11-01; built without SIMD, so it needs no libsimde-dev (open_source_software/proxsuite).
# Both bodies live in deploy/modules/arts.sh since 2026-11-01, because Dockerfile.arts runs the
# very same functions (decision 4); they were inline here before and had no container twin.
build_control_libs() {
    (
        # shellcheck source=deploy/modules/arts.sh
        source "$ARTS_ROOT/deploy/modules/arts.sh"
        arts_build_deps
        arts_control_libs_build
    )
}

build_arts_ws() {
    (
        # shellcheck source=deploy/modules/arts.sh
        source "$ARTS_ROOT/deploy/modules/arts.sh"
        arts_ws_build
    )
}

build_example_app() {
    echo "============================================================"
    echo " Building example_app_ros2_ws"
    echo "============================================================"
    env -i HOME="$HOME" USER="$USER" LOGNAME="$LOGNAME" PATH="$MINIMAL_PATH" \
      ARTS_ROOT="$ARTS_ROOT" \
      bash -c '
        set -eo pipefail   # ROS setup files read unset vars; -u goes on after sourcing
        source "$ARTS_ROOT/arts_ros2_ws/install/setup.bash"
        set -u
        cd "$ARTS_ROOT/example_app_ros2_ws"
        rosdep install --from-paths src --ignore-src -y
        # --base-paths src: the workspace root carries COLCON_IGNORE (so a colcon run from
        # the repo root skips it), and colcon honours that marker here too — without this
        # it finds 0 packages and reports success.
        [ -n "$(colcon list --base-paths src --names-only)" ] \
          || { echo "ERROR: colcon finds no packages in example_app_ros2_ws/src"; exit 1; }
        colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
        echo ">>> Finished building example_app_ros2_ws"
      '
}

MODE="full"
OVERLAY=""
SKIP_PREREQS=0
INSTALL_PATH=""
MODULES=""
while [ $# -gt 0 ]; do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --arts-only) MODE="arts"; shift ;;
        --overlay) MODE="overlay"; OVERLAY="${2:-}"; shift 2 ;;
        --prereqs) MODE="prereqs"; shift ;;
        --skip-prereqs) SKIP_PREREQS=1; shift ;;
        --path) INSTALL_PATH="${2:-}"; shift 2 ;;
        --modules) MODULES="${2:-}"; shift 2 ;;
        *) echo "Unknown arg: $1"; usage; exit 1 ;;
    esac
done

mkdir -p "$ARTS_LOCAL"/{oem_drivers,open_source_software,middle_overlay_ros2_wss}

case "$MODE" in
    prereqs)
        run_prereqs
        ;;
    overlay)
        [ -n "$OVERLAY" ] || { echo "--overlay needs a name (ur_ros2_ws, …)"; exit 1; }
        case "$OVERLAY" in
            ur) OVERLAY=ur_ros2_ws ;;
            flir|camera) OVERLAY=flir_ros2_ws ;;
            slicer) OVERLAY=slicer_ros2_ws ;;
            atracsys) OVERLAY=atracsys_ros2_ws ;;
            isaac) OVERLAY=isaac_ros2_ws ;;
            oem) OVERLAY=oem_ros2_ws ;;
        esac
        build_middle_overlay "$OVERLAY"
        ;;
    arts)
        build_arts_ws
        ;;
    full)
        echo "ARTS_ROOT=$ARTS_ROOT"
        echo "ARTS_LOCAL=$ARTS_LOCAL"

        # ── Host prerequisites the user must choose (GPU driver, RT kernel, isolcpus,
        #    account-walled SDKs) — see docs/host_prerequisites.md ──
        if [ "$SKIP_PREREQS" = 0 ]; then
            echo
            run_prereqs
        fi

        resolve_install_path
        echo
        echo "Install path: $INSTALL_PATH"
        echo "Modules:      $(selected_modules | tr '\n' ' ')"
        flir_enabled || echo "FLIR:         off (ARTS_ENABLE_FLIR=0): no stereo cameras in this install"
        echo
        flir_sdk_preflight

        # ── Module host deps — ALWAYS on the host, in every path.  udev rules,
        #    usbfs_memory_mb and sysctls are kernel-side; containers share the
        #    host kernel and cannot set them. ──
        for modname in $(selected_modules); do
            mod="$ARTS_ROOT/deploy/modules/${modname}.sh"
            [ -f "$mod" ] || { echo "WARNING: no module script for '$modname'"; continue; }
            echo "============================================================"
            echo " Module: $modname — host_deps"
            echo "============================================================"
            (
                # shellcheck source=/dev/null
                source "$mod"
                if declare -f "${modname}_host_deps" >/dev/null 2>&1; then
                    "${modname}_host_deps"
                fi
            )
        done

        # ── Bare metal: OEM SDKs onto the host, then every overlay natively ──
        if path_has bare; then
            for modname in $(selected_modules); do
                mod="$ARTS_ROOT/deploy/modules/${modname}.sh"
                [ -f "$mod" ] || continue
                echo "============================================================"
                echo " Module: $modname — oem_install (bare metal)"
                echo "============================================================"
                (
                    # shellcheck source=/dev/null
                    source "$mod"
                    if declare -f "${modname}_oem_install" >/dev/null 2>&1; then
                        "${modname}_oem_install"
                    fi
                )
            done

            for ws in $(default_overlays); do
                build_middle_overlay "$ws"
            done

            build_control_libs
            build_arts_ws
            build_example_app
        fi

        # ── Docker: each module Dockerfile RUNs the same <mod>_oem_install and
        #    <mod>_overlay_build (→ overlay_build_generic) that the bare-metal
        #    branch above just ran, so the two paths cannot drift. Verifiable:
        #    `rg overlay_build deploy/docker/Dockerfile.*` must name the same
        #    functions overlay_build_dispatch resolves to. ──
        if path_has docker; then
            build_docker_images
        fi

        echo
        echo "ALL DONE ($INSTALL_PATH)."
        if path_has bare; then
            echo "  source $ARTS_ROOT/arts_ros2_ws/install/setup.bash"
        fi
        if path_has docker; then
            selected_modules | grep -qx flir \
                && echo "  docker compose -f deploy/docker/compose.yaml --profile flir up"
            echo "  docker compose -f deploy/docker/compose.yaml run --rm arts   # shell in arts:jazzy"
        fi
        ;;
esac
