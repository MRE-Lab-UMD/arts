#!/usr/bin/env bash
# E8 session: record the bag and run e8_hold_real_arm.py against an arm that is ALREADY up
# (ur_arm.launch.py arm:=psm_left, External Control playing). Stop the pendant program first, then
# the arm launch, when it is done.
#
#   run_e8.sh OUT_DIR [e8_hold_real_arm.py arguments...]
#
# RUN rule: sources arts_ros2_ws/install only.
set -eo pipefail

OUT=${1:?usage: run_e8.sh OUT_DIR [e8_hold_real_arm.py args...]}
shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; [ -e "$OUT/bag" ] && { echo "run_e8: $OUT/bag exists" >&2; exit 2; }

source "$HERE/../../../install/setup.bash"
[ -f /etc/profile.d/arts-dds.sh ] && source /etc/profile.d/arts-dds.sh
set -u
{ date -Is; git -C "$HERE" rev-parse HEAD; echo "args: $*"; } > "$OUT/run_info.txt"

# The UR's own measured tool pose (a driver-internal topic, recorded raw) and the arm's status
# are the measurement; joint states and the commands are kept to reconstruct anything else.
# The arm is ARTS_ARM (default psm_left, decision 18 names).
A="${ARTS_ARM:-psm_left}"
setsid ros2 bag record -s mcap -o "$OUT/bag" "/$A/tcp_pose_broadcaster/pose" "/$A/joint_states" \
  "/$A/status" "/$A/tool_tip_target" "/$A/tool_tip_state" "/$A/joint_command" \
  > "$OUT/record.log" 2>&1 &
REC=$(ps -o pgid= -p $! | tr -d ' ')
# SIGTERM: a background job of a non-interactive shell ignores SIGINT, which left the E8
# and A/B bags unclosed on 2026-10-30 (run_e5.sh has the same note).
trap 'kill -TERM -- "-$REC" 2>/dev/null || true; sleep 3' EXIT
sleep 2
python3 "$HERE/e8_hold_real_arm.py" "$@" 2>&1 | tee "$OUT/motion.log"
