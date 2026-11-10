#!/usr/bin/env bash
# RCM-drift trial: record the bag and run rcm_motion_real_arm.py drift against an arm that is
# ALREADY up (instrument engaged and zeroed, External Control playing, the RCM ON in free air,
# the shaft marker aligned onto it with rcm_motion_real_arm.py wobble + pivot_monitor.py), then
# analyze. The tracker node runs in its OWN shell from ~/atracsys_ws (never sourced here); its
# /atracsys/* topics are recorded by name pattern.
#
#   run_rcm_drift.sh OUT_DIR [--minutes 5] [--seed N] [other drift arguments]
#
# One trial per OUT_DIR; use a new seed per trial. RUN rule: sources arts_ros2_ws/install only.
set -eo pipefail

OUT=${1:?usage: run_rcm_drift.sh OUT_DIR [--minutes 5] [--seed N]}
shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; [ -e "$OUT/bag" ] && { echo "run_rcm_drift: $OUT/bag exists" >&2; exit 2; }

source "$HERE/../../../install/setup.bash"
[ -f /etc/profile.d/arts-dds.sh ] && source /etc/profile.d/arts-dds.sh
set -u
A="${ARTS_ARM:-psm_left}"
{ date -Is; git -C "$HERE" rev-parse HEAD; git -C "$HERE" status --short -- . ; echo "arm: $A; args: $*"; } \
  > "$OUT/run_info.txt"

setsid ros2 bag record -s mcap -o "$OUT/bag" -e \
  "^/(atracsys/.*|$A/(bench_event|tool_tip_state|tool_tip_target|rcm_state|joint_states|tool_joint_states|joint_command|status)|tf|tf_static)$" \
  > "$OUT/record.log" 2>&1 &
REC=$(ps -o pgid= -p $! | tr -d ' ')
# SIGTERM: a background job of a non-interactive shell ignores SIGINT (run_e8.sh).
trap 'kill -TERM -- "-$REC" 2>/dev/null || true; sleep 3' EXIT
sleep 2
python3 "$HERE/rcm_motion_real_arm.py" drift "$@" 2>&1 | tee "$OUT/motion.log"
sleep 1
kill -TERM -- "-$REC" 2>/dev/null || true
sleep 3
trap - EXIT
python3 "$HERE/rcm_drift_analyze.py" "$OUT/bag" --arm "$A" --json "$OUT/results.json" \
  2>&1 | tee "$OUT/analysis.txt"
