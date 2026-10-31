#!/usr/bin/env bash
# Freedrive while the ROS driver keeps running (27 Oct 2026 method). Arm must be STATIONARY when you start and stop.
#   bash scripts/lab/freedrive.sh     -> switches to freedrive and keeps it enabled; Ctrl+C ends freedrive and
#                                        switches the arm back to scaled_joint_trajectory_controller.
# The RG2 controller is not touched.
set -Eeo pipefail
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"
read -r -p "Arm stationary, workspace clear, E-stop in reach? Type FREEDRIVE: " a; [[ "$a" == "FREEDRIVE" ]] || exit 1
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate freedrive_mode_controller
restore() {
  echo; echo "Ending freedrive: hold the arm still..."
  sleep 1
  ros2 control switch_controllers --deactivate freedrive_mode_controller --activate scaled_joint_trajectory_controller
  ros2 control list_controllers | grep -E "scaled_joint_trajectory_controller|freedrive_mode_controller|finger_width"
}
trap restore EXIT
echo "FREEDRIVE ON (Ctrl+C to end)."
ros2 topic pub --rate 2 /freedrive_mode_controller/enable_freedrive_mode std_msgs/msg/Bool "{data: true}" >/dev/null
