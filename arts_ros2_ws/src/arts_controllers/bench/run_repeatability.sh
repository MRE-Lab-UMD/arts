#!/usr/bin/env bash
# Pose repeatability session: record the bag and run repeatability_real_arm.py against an arm
# that is ALREADY up (instrument engaged and zeroed, External Control playing; no cannula, the
# RCM may stay off), then analyze. The poses are taught first, outside this script:
#   python3 repeatability_real_arm.py teach POSES.yaml --tracker-topic /atracsys/<tool>/measured_cp
# The tracker node runs in its OWN shell from ~/atracsys_ws (never sourced here); its /atracsys/*
# topics are recorded by name pattern.
#
#   run_repeatability.sh OUT_DIR run POSES.yaml --tracker-topic /atracsys/<tool>/measured_cp [--cycles N]
#
# RUN rule: sources arts_ros2_ws/install only.
set -eo pipefail

OUT=${1:?usage: run_repeatability.sh OUT_DIR run POSES.yaml --tracker-topic T [--cycles N]}
shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; [ -e "$OUT/bag" ] && { echo "run_repeatability: $OUT/bag exists" >&2; exit 2; }

source "$HERE/../../../install/setup.bash"
[ -f /etc/profile.d/arts-dds.sh ] && source /etc/profile.d/arts-dds.sh
set -u
A="${ARTS_ARM:-psm_left}"
{ date -Is; git -C "$HERE" rev-parse HEAD; git -C "$HERE" status --short -- . ; echo "arm: $A; args: $*"
  cat "$HERE/repeatability_protocol.yaml"; [ -f "${2:-}" ] && { echo "--- $2"; cat "$2"; }; } > "$OUT/run_info.txt"

# The tracker topics, the controller's kinematic tip and RCM state, the stop events, and what
# is needed to reconstruct anything else.
setsid ros2 bag record -s mcap -o "$OUT/bag" -e \
  "^/(atracsys/.*|$A/(bench_event|tool_tip_state|tool_tip_target|rcm_state|joint_states|tool_joint_states|joint_command|status)|tf|tf_static)$" \
  > "$OUT/record.log" 2>&1 &
REC=$(ps -o pgid= -p $! | tr -d ' ')
# SIGTERM: a background job of a non-interactive shell ignores SIGINT (run_e8.sh).
trap 'kill -TERM -- "-$REC" 2>/dev/null || true; sleep 3' EXIT
sleep 2
python3 "$HERE/repeatability_real_arm.py" "$@" 2>&1 | tee "$OUT/motion.log"
kill -TERM -- "-$REC" 2>/dev/null || true
sleep 3
trap - EXIT
python3 "$HERE/repeatability_analyze.py" "$OUT/bag" --arm "$A" --json "$OUT/results.json" \
  2>&1 | tee "$OUT/analysis.txt"
