#!/usr/bin/env bash
# UR3e OFFICIAL DRIVER ONLY; no RG2, MoveIt, RViz, freedrive or user motion nodes.
set -Eeo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

IP="${UR3E_IP:-192.168.0.192}"
if ! [[ "$IP" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
  echo "ERROR: UR3E_IP should be an IPv4 address (got: $IP)" >&2; exit 2
fi
if ! ros2 pkg prefix ur_robot_driver >/dev/null 2>&1; then
  echo 'ERROR: ur_robot_driver is not available in this ROS environment.' >&2
  exit 1
fi

cat <<MSG
UR3e BASELINE CONNECTION TEST
-----------------------------
Robot:       $IP
ROS distro:  $ROS_DISTRO
Launch:      official ur_robot_driver/ur_control.launch.py
Not running: OnRobot, RG2, RViz, MoveIt, Servo, custom robot_controller,
             any demonstration replay or programmed movement.

IMPORTANT: Do not run the old combined start_robot.launch.py at the same time.
With the physical robot ready and network connected, start this launch FIRST.
Then press Play on the External Control program on the pendant.
This script does not send movement commands or activate freedrive.
Use Ctrl+C to end. A stopped External Control program may need Play again.
MSG

args=(
  ur_type:=ur3e
  "robot_ip:=$IP"
  launch_rviz:=false
  headless_mode:=false
)
# Optional: explicitly provide calibration only if verified for THIS physical UR3e.
# Example: UR3E_CALIBRATION_FILE="$HOME/ur3e_calibration.yaml" bash ...
if [[ -n "${UR3E_CALIBRATION_FILE:-}" ]]; then
  if [[ ! -f "$UR3E_CALIBRATION_FILE" ]]; then
    echo "ERROR: calibration file not found: $UR3E_CALIBRATION_FILE" >&2; exit 1
  fi
  echo "Using explicitly selected robot calibration: $UR3E_CALIBRATION_FILE"
  args+=("kinematics_params_file:=$UR3E_CALIBRATION_FILE")
else
  echo 'No robot-specific calibration selected: connection diagnostics ONLY.'
  echo 'Do NOT use an uncalibrated setup for precision pose replay.'
fi

exec ros2 launch ur_robot_driver ur_control.launch.py "${args[@]}"
