#!/usr/bin/env bash
# E5: servo loop health of one UR5e under the Cartesian servo controller, 10 minutes.
#
#   run_e5.sh OUT_DIR [--fake]         (real psm_right by default; ARTS_ARM picks another)
#
# Brings up the arm (its own calibration, loop SCHED_FIFO on an isolated core,
# non-blocking read), records <arm>/joint_states and /diagnostics to MCAP, and drives the arm with first_motion_real_arm.py
# (10 s hold, 30 mm out and back on x/y/z at 5 mm/s) repeated until 600 s have passed. Then
# analyze_e5.py. On a real arm the External Control program must be started on the pendant
# when asked, and STOPPED on the pendant before this script tears the driver down.
#
# RUN rule: sources arts_ros2_ws/install only, in a clean environment.
set -eo pipefail

OUT=${1:?usage: run_e5.sh OUT_DIR [--fake]}
FAKE=${2:-}
DURATION=${E5_DURATION:-600}   # E5_DURATION only to shorten a --fake check
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
mkdir -p "$OUT"; [ -e "$OUT/bag" ] && { echo "run_e5: $OUT/bag exists" >&2; exit 2; }

source "$HERE/../../../install/setup.bash"
[ -f /etc/profile.d/arts-dds.sh ] && source /etc/profile.d/arts-dds.sh
set -u
export LD_LIBRARY_PATH="$ARTS_LOCAL/open_source_software/pinocchio/install/lib:$ARTS_LOCAL/open_source_software/coal/install/lib:${LD_LIBRARY_PATH:-}"

export ARTS_ARM="${ARTS_ARM:-psm_right}"   # decision 18 role name; E5 2026-10-29 ran the right UR5e
if [ "$FAKE" = --fake ]; then
  export ROS_DOMAIN_ID=$(( (RANDOM % 100) + 1 ))
  ARM_ARGS=(arm:="$ARTS_ARM" use_fake_hardware:=true)
else
  ARM_ARGS=(arm:="$ARTS_ARM" headless_mode:=false)
fi

{ date -Is; uname -r; cat /sys/devices/system/cpu/isolated; git -C "$HERE" rev-parse HEAD;
  echo "args: ${ARM_ARGS[*]}"; } > "$OUT/run_info.txt"

setsid ros2 launch arts_driver ur_arm.launch.py "${ARM_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PGID=$(ps -o pgid= -p $! | tr -d ' ')
REC_PGID=""
cleanup() {
  [ -n "$REC_PGID" ] && kill -TERM -- "-$REC_PGID" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true
}
trap cleanup EXIT

echo "run_e5: waiting for the controller ..."
for _ in $(seq 1 40); do
  grep -aq "Configured and activated servo_controller" "$OUT/launch.log" && break
  sleep 1
done
grep -aq "Configured and activated servo_controller" "$OUT/launch.log" \
  || { echo "run_e5: controller never activated"; tail -20 "$OUT/launch.log"; exit 1; }
if grep -aq "calibration parameters of the connected robot don't match" "$OUT/launch.log"; then
  echo "run_e5: calibration mismatch -- refusing to benchmark nominal kinematics"; exit 1
fi

if [ "$FAKE" != --fake ]; then
  echo "run_e5: START the External Control program on the pendant now (hand on the e-stop)."
  until grep -aq "Robot connected to reverse interface" "$OUT/launch.log"; do sleep 1; done
fi
sleep 2

setsid ros2 bag record -s mcap -o "$OUT/bag" "/$ARTS_ARM/joint_states" /diagnostics \
  "/$ARTS_ARM/tool_tip_target" "/$ARTS_ARM/joint_command" > "$OUT/record.log" 2>&1 &
REC_PGID=$(ps -o pgid= -p $! | tr -d ' ')
sleep 3

end=$(( $(date +%s) + DURATION ))
n=0
while [ "$(date +%s)" -lt "$end" ]; do
  n=$((n + 1))
  echo "run_e5: motion cycle $n ($(( end - $(date +%s) )) s left)"
  python3 "$HERE/../test/first_motion_real_arm.py" --step 0.03 --speed 0.005 \
    >> "$OUT/motion.log" 2>&1 || { echo "run_e5: motion cycle $n ABORTED (motion.log)"; break; }
done

# SIGTERM, not SIGINT: without job control bash starts background jobs with SIGINT
# ignored, so an INT never reaches the recorder and the bag is left unclosed (2026-10-29).
kill -TERM -- "-$REC_PGID"; REC_PGID=""
for _ in $(seq 1 30); do [ -f "$OUT/bag/metadata.yaml" ] && break; sleep 1; done
[ -f "$OUT/bag/metadata.yaml" ] || { echo "run_e5: recorder did not close the bag"; exit 1; }
if [ "$FAKE" != --fake ]; then
  # Wait for the pendant, not for a key press: stopping the driver first leaves the pendant
  # with "Error connecting to remote" (2026-10-29).
  echo "run_e5: STOP the External Control program on the pendant now."
  for _ in $(seq 1 300); do
    grep -aqiE "Connection to reverse interface dropped|reverse interface.*(closed|dropped)" \
      "$OUT/launch.log" && break
    sleep 1
  done
fi
python3 "$HERE/analyze_e5.py" "$OUT/bag" --topic "/$ARTS_ARM/joint_states" --launch-log "$OUT/launch.log" --json "$OUT/e5_result.json" \
  | tee "$OUT/analysis.txt"
