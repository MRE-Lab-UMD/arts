#!/usr/bin/env bash
set -euo pipefail
# Build ur_ros2_ws — delegates to deploy/modules/_common.sh::overlay_build_generic.
# Recipe lives in the git repo (ur_ros2.repos); artifacts go to $ARTS_LOCAL.
#
# ur_moveit_config is deliberately NOT built. ARTS has no MoveIt in its architecture
# (ros2_control chainable controllers + pinocchio + proxsuite), and building this one
# package pulls the whole MoveIt stack — 45 apt packages — for something we never load.
# See AGENTS.md decision 11 for the architectural argument, which is the real reason.
#
# A historical note, since this comment used to give a different rationale: the trigger
# for investigating was that MoveIt installed ros-jazzy-ruckig 0.9.2 into /opt/ros/jazzy,
# where it would have shadowed the then-pinned from-source Ruckig 0.14.0 in the 500 Hz
# smoother. That specific hazard is gone — ARTS now uses apt's 0.9.2 deliberately
# (open_source_software/ruckig/VERSION), so the MoveIt copy is the copy we want. The
# exclusion stands on decision 11 alone, and arts_controllers asserts the version at
# configure time regardless.
#
# `--exclude` applies THREE markers, and they are not interchangeable: colcon reads
# COLCON_IGNORE, rosdep's crawler reads AMENT_IGNORE (without it rosdep still installs the
# MoveIt dependency tree for a package colcon never builds), and rosdep --skip-keys stops
# the `ur` metapackage's exec_depend from pulling the prebuilt ros-jazzy-ur-moveit-config
# in through the back door.
#
# Reversing this is a one-line change plus a rebuild — MoveIt belongs in an application
# overlay if it is ever wanted, not in a middle overlay.

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ARTS_ROOT

# The exclusion itself is applied by ur_overlay_build in deploy/modules/ur.sh, so the
# installer, this script and a Dockerfile run one function.
# shellcheck source=../../deploy/modules/ur.sh
source "$ARTS_ROOT/deploy/modules/ur.sh"
ur_overlay_build
