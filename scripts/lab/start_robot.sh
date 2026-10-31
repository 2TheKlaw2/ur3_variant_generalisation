#!/usr/bin/env bash
# Real UR3e + RG2 driver, exactly as in the 27-29 Oct 2026 physical sessions. Then press Play on the pendant's
# External Control program. Active controllers afterwards: scaled_joint_trajectory_controller, finger_width_trajectory_controller.
set -Eeo pipefail
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"
CAL="${UR3E_CALIBRATION_FILE:-$HOME/ur3e_calibration.yaml}"
[[ -f "$CAL" ]] || { echo "Missing robot calibration $CAL" >&2; exit 2; }
echo "Starting UR3e + RG2 driver (192.168.0.192, calibration $CAL). Press Play on External Control when it is up."
exec ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 \
  robot_ip:=192.168.0.192 kinematics_parameters_file:="$CAL" launch_rviz:=false
