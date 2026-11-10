#!/usr/bin/env bash
# Teleop stack for a real arm that is ALREADY up (ur_arm.launch.py arm:=psm_left, External
# Control playing): MTM bridge, footswitch, teleop follower, and a state recording. Names per
# decision 18; the follower's arm and mtm come from config/teleop.yaml (or the overrides). Everything
# stops by itself after MINUTES; the arm then holds (the controller's 0.1 s reference
# timeout). The arm launch and the pendant are left to the operator: stop the pendant program
# first, then the arm launch.
#
#   run_teleop_real.sh OUT_DIR [MINUTES=10] [--ros-args overrides for the follower...]
#
# e.g. ... run_teleop_real.sh ~/arts_local/teleop_left_20261030 10 \
#        -p "operator_to_base:=[0.0,1.0,0.0, ...]"
#
# The follower runs from config/teleop.yaml (deadman mode unless overridden); extra
# arguments are appended to its --ros-args, so a measured operator_to_base can be tried
# before it is committed to the file. RUN rule: sources arts_ros2_ws/install only.
#
# With an instrument: ARTS_TOOL=<file in arts_dv_tool_driver/config/instruments/> (and
# ARTS_MTM, default mtm_left, for its grip) adds the jaw follower (gate teleop: the jaw follows
# the VerseGrip lever only while the follower is engaged; close the lever before the pedal) and
# records the instrument topics. disc_drive for that arm, with the instrument engaged and
# zeroed (gamepad LB, LB+X), must already be running; the script refuses otherwise.
#
# With the right tool arm too: ARTS_RIGHT_TOOL=<instrument file> starts a second follower
# (teleop.yaml block teleop_follower_right: psm_right <- mtm_right) and a right jaw follower,
# and records psm_right. disc_drive for psm_right must be running, engaged and zeroed.
#
# With the camera arm: ARTS_CAMERA=1 starts camera_teleop.py (both MTMs move the ECM while the
# middle pedal is held; the follower disengages on it) with its follower-matching settings
# (ARTS_CAMERA_ARGS, extra --ros-args) and records /ecm. The ECM's External Control must play.
#
# Handle hold: ARTS_MTM_HOLD=1 runs mtm_hold.py (holds both Inverse3s while the head is out). OFF by
# default: its PC-side spring (300 N/m, 5 N s/m) went unstable on the real handles 2026-11-06 (a
# small pull grew into a shake that spread to the other device).
#
# With the ESU: ARTS_ESU=1 starts the ESU-TAP board's agent + esu node (arts_esu esu.launch.py)
# and records /esu. Energy is by the physical ESU pedals: each press is an /esu/state event
# with source footswitch, stamped by the esu node on arrival; that stamp, not the bag's receive
# time, is the energy timing (decision 18, 2026-11-03). Software energy stays disabled.
set -eo pipefail

OUT=${1:?usage: run_teleop_real.sh OUT_DIR [MINUTES] [follower --ros-args ...]}
MINUTES=${2:-10}
shift 2 || shift $#
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; [ -e "$OUT/bag" ] && { echo "run_teleop_real: $OUT/bag exists" >&2; exit 2; }

source "$HERE/../../../../install/setup.bash"
[ -f /etc/profile.d/arts-dds.sh ] && source /etc/profile.d/arts-dds.sh
set -u
SHARE_T=$(ros2 pkg prefix arts_teleop)/share/arts_teleop
SHARE_D=$(ros2 pkg prefix arts_driver)/share/arts_driver

{ date -Is; git -C "$HERE" rev-parse HEAD; echo "minutes: $MINUTES"; echo "follower extra: $*"; } \
  > "$OUT/run_info.txt"

PGIDS=()
start() {  # start NAME CMD... in its own process group, log to OUT/NAME.log
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  PGIDS+=("$(ps -o pgid= -p $! | tr -d ' ')")
}
cleanup() {
  for g in "${PGIDS[@]}"; do kill -INT -- "-$g" 2>/dev/null || true; done
  sleep 3
  for g in "${PGIDS[@]}"; do kill -TERM -- "-$g" 2>/dev/null || true; done
}
trap cleanup EXIT

start bridge ros2 run arts_driver haply_bridge --ros-args --params-file "$SHARE_D/config/haply.yaml"
start footswitch ros2 run arts_teleop footswitch.py --ros-args \
  -p mapping_file:="$SHARE_T/config/input_mapping.yaml"
# Recorded: the handle, the pedal, every command, and what the arm did. The software-path
# latency (handle frame -> first /joint_states motion) comes from this bag.
A="${ARTS_ARM:-psm_left}"
TOOL_TOPICS=()
if [ -n "${ARTS_TOOL:-}" ]; then
  timeout 5 ros2 topic echo --once "/$A/tool_joint_states" > /dev/null 2>&1 || {
    echo "run_teleop_real: ARTS_TOOL set but no /$A/tool_joint_states: start disc_drive with" \
         "instrument_file:=$ARTS_TOOL, engage and zero first" >&2; exit 2; }
  TOOL_TOPICS=("/$A/tool_joint_states" "/$A/tool_actuator_states" "/$A/tool_joint_target"
               "/$A/tool_jaw_target" /mtm_right/stylus_state)
  echo "tool: $ARTS_TOOL (grip ${ARTS_MTM:-mtm_left})" >> "$OUT/run_info.txt"
fi
RIGHT_TOPICS=()
if [ -n "${ARTS_RIGHT_TOOL:-}" ]; then
  timeout 5 ros2 topic echo --once /psm_right/tool_joint_states > /dev/null 2>&1 || {
    echo "run_teleop_real: ARTS_RIGHT_TOOL set but no /psm_right/tool_joint_states" >&2; exit 2; }
  RIGHT_TOPICS=(/psm_right/tool_tip_target /psm_right/tool_tip_state /psm_right/joint_command
                /psm_right/joint_states /psm_right/rcm_state /psm_right/teleop_state /psm_right/tool_joint_states
                /psm_right/tool_actuator_states /psm_right/tool_jaw_target /mtm_right/cursor_velocity)
fi
ESU_TOPICS=()
CAM_TOPICS=()
if [ -n "${ARTS_CAMERA:-}" ]; then
  start camera ros2 run arts_teleop camera_teleop.py --ros-args \
    -p mapping_file:="$SHARE_T/config/input_mapping.yaml" ${ARTS_CAMERA_ARGS:-}
  CAM_TOPICS=(/ecm/tool_tip_target /ecm/tool_tip_state /ecm/joint_states /ecm/rcm_state /ecm/joint_command /ecm/camera_teleop_state
              /console/head_sensor)
fi
# The handles held while the head is out (mtm_hold.py, the da Vinci's held masters); off by
# default until the hold is stable on the real handles (see the header).
HOLD_TOPICS=()
if [ "${ARTS_MTM_HOLD:-0}" != 0 ]; then
  start hold ros2 run arts_teleop mtm_hold.py
  HOLD_TOPICS=(/mtm_left/cursor_force /mtm_right/cursor_force)
  [ -n "${ARTS_CAMERA:-}" ] || HOLD_TOPICS+=(/console/head_sensor)
fi
if [ -n "${ARTS_ESU:-}" ]; then
  start esu ros2 launch arts_esu esu.launch.py
  timeout 15 bash -c 'until grep -aq "\[esu\]: board " "$0"; do sleep 0.5; done' "$OUT/esu.log" || {
    echo "run_teleop_real: ARTS_ESU set but the esu node found no board (/dev/arts_esu?)" >&2
    tail "$OUT/esu.log" >&2; exit 2; }
  ESU_TOPICS=(/esu/state /esu/device_state /esu/command /esu/info /esu/activation_state)
  grep -a "\[esu\]: board " "$OUT/esu.log" | head -1 | sed 's/.*\[esu\]: /esu: /' >> "$OUT/run_info.txt"
fi
start record ros2 bag record -s mcap -o "$OUT/bag" /mtm_left/cursor /mtm_right/cursor \
  /mtm_left/cursor_velocity /mtm_left/joint_states /mtm_left/stylus_state /console/footswitch_teleop \
  "/$A/tool_tip_target" "/$A/tool_tip_state" "/$A/joint_command" "/$A/joint_states" "/$A/rcm_state" \
  "/$A/status" "/$A/teleop_state" /tf /tf_static /console/episode_mark /ecm/stereo_sync/pulse \
  /arts/collision_hold /collision_guard/state "${TOOL_TOPICS[@]}" "${ESU_TOPICS[@]}" \
  "${CAM_TOPICS[@]}" "${RIGHT_TOPICS[@]}" "${HOLD_TOPICS[@]}"
sleep 2
start follower ros2 run arts_teleop teleop_follower.py --ros-args \
  --params-file "$SHARE_T/config/teleop.yaml" -p mapping_file:="$SHARE_T/config/input_mapping.yaml" "$@"

if [ -n "${ARTS_RIGHT_TOOL:-}" ]; then
  start follower_right ros2 run arts_teleop teleop_follower.py --ros-args -r __node:=teleop_follower_right \
    --params-file "$SHARE_T/config/teleop.yaml" -p mapping_file:="$SHARE_T/config/input_mapping.yaml"
  start jaw_right python3 "$(ros2 pkg prefix arts_teleop)/lib/arts_teleop/jaw_follower.py" --ros-args \
    -r __node:=jaw_follower_right -p arm:=psm_right -p mtm:=mtm_right -p gate:=teleop \
    -p instrument_file:="$ARTS_RIGHT_TOOL"
fi
if [ -n "${ARTS_TOOL:-}" ]; then
  start jaw python3 "$(ros2 pkg prefix arts_teleop)/lib/arts_teleop/jaw_follower.py" --ros-args \
    -p arm:="$A" -p mtm:="${ARTS_MTM:-mtm_left}" -p gate:=teleop -p instrument_file:="$ARTS_TOOL"
fi

sleep 3
grep -aq "connected" "$OUT/bridge.log" || { echo "run_teleop_real: bridge not connected"; tail "$OUT/bridge.log"; exit 1; }
grep -aq "mtm_left -> " "$OUT/follower.log" || { echo "run_teleop_real: follower not up"; tail "$OUT/follower.log"; exit 1; }
echo "run_teleop_real: teleop LIVE for $MINUTES min -- arm moves only while the clutch pedal is held"
sleep $(( MINUTES * 60 ))
echo "run_teleop_real: time up, stopping (the arm holds)"
