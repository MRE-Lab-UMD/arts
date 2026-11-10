#!/usr/bin/env bash
# Bring up a mock UR5e as ARTS_ARM (default psm_left, decision 18), run servo_smoke_test.py
# against it, and tear everything down.
#
#   arts_ros2_ws/src/arts_controllers/test/run_servo_smoke_test.sh [TEST_SCRIPT]
#
# TEST_SCRIPT defaults to servo_smoke_test.py; branch_singularity_test.py runs the same way.
# ARTS_WRIST=<instrument file> runs the arm with the instrument wrist and the fake instrument
# (wrist_mock_test.py).
# A TEST_SCRIPT given as a path (containing a /) is used as is, for tests in other packages.
#
# Two things here are not incidental.
#
# ISOLATION. Each run gets its own ROS_DOMAIN_ID. Without it a leftover controller_manager
# from an earlier run is discovered by the new spawner and answers on the same topics, and
# the test then measures a mixture of two stacks. That happened on 2026-08-12 and produced
# a full run of confidently wrong results -- three bringups alive at once, the spawner
# reporting "can not be configured from 'active' state" against somebody else's controller.
#
# TEARDOWN. The bringup is started with setsid so it owns a process group, and the trap
# kills that group. Killing the `ros2 launch` pid alone orphans ros2_control_node and
# robot_state_publisher, which is how the mess above accumulated.
set -eo pipefail

ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG="${LOG:-/tmp/arts_servo_smoke.log}"

source "$WORKSPACE/install/setup.bash"
set -u

# MIDDLEWARE. deploy/modules/dds.sh puts RMW_IMPLEMENTATION and CYCLONEDDS_URI in
# /etc/profile.d, which LOGIN shells read and this one does not. Left to itself the wrapper
# would fall back to Fast DDS and test a different stack than production runs on -- the same
# bare-metal/container middleware split that dds.sh exists to close. Sourced only when the
# caller has not already chosen, so an explicit RMW still wins for A/B runs.
if [ -z "${RMW_IMPLEMENTATION:-}" ] && [ -f /etc/profile.d/arts-dds.sh ]; then
  source /etc/profile.d/arts-dds.sh
fi
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset: RMW default>}"

export LD_LIBRARY_PATH="$ARTS_LOCAL/open_source_software/pinocchio/install/lib:$ARTS_LOCAL/open_source_software/coal/install/lib:${LD_LIBRARY_PATH:-}"
# 1..101 keeps it inside the range every RMW supports.
export ROS_DOMAIN_ID=$(( (RANDOM % 100) + 1 ))
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID   log=$LOG"

# SERVO_CONFIG (optional) swaps the controller config, e.g. to run a test in rate mode.
# ARTS_TIP (optional) starts the controller with that tool_tips.yaml preset (tip:=).
export ARTS_ARM="${ARTS_ARM:-psm_left}"
setsid ros2 launch arts_driver ur_arm.launch.py arm:="$ARTS_ARM" use_fake_hardware:=true \
  ${SERVO_CONFIG:+servo_config:=$SERVO_CONFIG} ${ARTS_TIP:+tip:=$ARTS_TIP} \
  ${ARTS_WRIST:+wrist_instrument:=$ARTS_WRIST} > "$LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')

cleanup() {
  kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
}
trap cleanup EXIT

# Readiness is read from the launch log rather than from `ros2 control list_controllers`,
# which returns nothing here: the ros2cli daemon caches a discovery context per domain and
# does not pick up a domain created seconds ago inside an `env -i` shell. The spawner
# already reports the transition, so ask it instead of fighting the CLI.
echo "waiting for the controller to activate ..."
for _ in $(seq 1 40); do
  if grep -aq "Configured and activated servo_controller" "$LOG"; then
    break
  fi
  sleep 1
done

grep -aq "Configured and activated servo_controller" "$LOG" || {
  echo "FAIL: the servo controller never became active. Last 40 lines of $LOG:"
  tail -40 "$LOG"
  exit 1
}

echo
TEST="${1:-servo_smoke_test.py}"
[[ "$TEST" == */* ]] || TEST="$(dirname "${BASH_SOURCE[0]}")/$TEST"
python3 "$TEST"
