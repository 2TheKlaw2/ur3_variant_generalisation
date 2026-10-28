#!/usr/bin/env bash
# Safe recording-only wrapper. No robot or gripper movement commands.
set -Eeo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$HERE/.." && pwd)"
WS="${HOME}/ros2_ws"
RAW="${PROJECT}/demonstrations/raw"
LOGS="${PROJECT}/demonstrations/logs"

if [[ $# -ne 1 ]]; then
  echo 'Usage: bash scripts/record_v1.sh v1_reach|v1_transport|v1_assembly_full|v1_insertion|v1_rotation [optional _NNN suffix]' >&2
  exit 2
fi
NAME="$1"
if [[ ! "$NAME" =~ ^v1_(reach|transport|assembly_full|insertion|rotation)(_[0-9]{3})?$ ]]; then
  echo "ERROR: Invalid recording name: $NAME" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

if [[ "$NAME" =~ _[0-9]{3}$ ]]; then
  OUT="$RAW/$NAME.csv"
else
  for i in $(seq 1 999); do
    printf -v INDEX '%03d' "$i"
    CANDIDATE="${NAME}_${INDEX}"
    if [[ ! -e "$RAW/$CANDIDATE.csv" && ! -e "$LOGS/$CANDIDATE.log" ]]; then
      NAME="$CANDIDATE"
      break
    fi
  done
  if [[ ! "$NAME" =~ _[0-9]{3}$ ]]; then
    echo 'ERROR: No available recording number in 001–999' >&2
    exit 1
  fi
  OUT="$RAW/$NAME.csv"
fi

mkdir -p "$RAW" "$LOGS"
LOG="$LOGS/$NAME.log"
if [[ -e "$OUT" || -e "$LOG" ]]; then
  echo "ERROR: Output or log exists; will not overwrite: $OUT / $LOG" >&2
  exit 1
fi

RECORDER="${WS}/install/ur3_lfd/lib/ur3_lfd/demo_recorder"
PARAMS="${WS}/src/ur3_lfd/config/real.yaml"
if [[ ! -x "$RECORDER" || ! -f "$PARAMS" ]]; then
  echo "ERROR: Required existing ur3_lfd recorder/config missing. Check ROS build." >&2
  exit 1
fi
if pgrep -x demo_recorder >/dev/null; then
  echo 'ERROR: Another demo_recorder is already running. Stop it first.' >&2
  exit 1
fi

REC_PID=""
RECORDING=false
SAVED=false
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ "$RECORDING" == true && "$SAVED" == false ]]; then
    echo 'Interrupted during recording; attempting to stop and save...' >&2
    timeout 12s ros2 service call /stop_recording std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  fi
  if [[ -n "$REC_PID" ]]; then
    kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Reserve log to prevent accidental reuse before starting the recorder.
( set -C; : > "$LOG" ) || { echo "ERROR: Log already exists: $LOG" >&2; exit 1; }

"$RECORDER" --ros-args \
  --params-file "$PARAMS" \
  -p "recorder.output_file:=$OUT" \
  -p 'recorder.base_frame:=base' \
  -p 'recorder.tool_frame:=gripper_tcp' \
  -p 'recorder.auto_start:=false' \
  -p 'recorder.exit_after_stop:=false' \
  >"$LOG" 2>&1 &
REC_PID=$!

echo "Recording name: $NAME"
echo "Output: $OUT"
echo "TF: base -> gripper_tcp  |  50 Hz"
echo 'Recording only: this script DOES NOT enable freedrive or move the robot.'
echo 'Use the approved pendant/controller freedrive procedure first.'

echo 'Waiting for recorder start and stop services...'
READY=false
for i in $(seq 1 60); do
  if ! kill -0 "$REC_PID" 2>/dev/null; then
    break
  fi
  SERVICES="$(ros2 service list 2>/dev/null || true)"
  if grep -Fxq /start_recording <<< "$SERVICES" && grep -Fxq /stop_recording <<< "$SERVICES"; then
    READY=true
    break
  fi
  sleep 0.5
done
if [[ "$READY" != true ]]; then
  echo 'ERROR: Recorder services unavailable; inspect log:' >&2
  tail -n 25 "$LOG" >&2
  exit 1
fi

read -r -p 'Press ENTER to START recording (robot held still at start pose)... ' _
echo 'Starting recording...'
START_REPLY="$(timeout 20s ros2 service call /start_recording std_srvs/srv/Trigger '{}')" || {
  echo 'ERROR: Start service failed.' >&2; exit 1;
}
if [[ "$START_REPLY" != *'success=True'* && "$START_REPLY" != *'success: true'* ]]; then
  echo "ERROR: Start service rejected: $START_REPLY" >&2
  exit 1
fi
RECORDING=true

echo 'RECORDING. Guide the arm with the existing safe freedrive procedure.'
read -r -p 'Press ENTER to STOP and SAVE recording... ' _
echo 'Stopping and saving...'
STOP_REPLY="$(timeout 20s ros2 service call /stop_recording std_srvs/srv/Trigger '{}')" || {
  echo 'ERROR: Stop service failed; check recorder log.' >&2; exit 1;
}
if [[ "$STOP_REPLY" != *'success=True'* && "$STOP_REPLY" != *'success: true'* ]]; then
  echo "ERROR: Stop service rejected: $STOP_REPLY" >&2
  exit 1
fi
RECORDING=false
SAVED=true
if [[ ! -s "$OUT" ]]; then
  echo "ERROR: No CSV created. See: $LOG" >&2; exit 1
fi
LINE_COUNT=$(wc -l < "$OUT")
if (( LINE_COUNT < 12 )); then
  echo "WARNING: only $((LINE_COUNT - 1)) samples in CSV. Verify data before use." >&2
fi
HEADER=$(head -n 1 "$OUT")
if [[ "$HEADER" != time,shoulder_pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3,tcp_x,tcp_y,tcp_z,tcp_qx,tcp_qy,tcp_qz,tcp_qw,gripper_width ]]; then
  echo 'WARNING: Unexpected CSV header; inspect before training.' >&2
fi
printf '\nSAVED: %s (%s samples)\nLOG: %s\n' "$OUT" "$((LINE_COUNT - 1))" "$LOG"
