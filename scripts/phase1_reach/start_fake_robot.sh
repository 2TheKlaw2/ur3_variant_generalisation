#!/usr/bin/env bash
set -eo pipefail
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
CALIB="$HOME/ros2_ws/src/ur3_variant_generalisation/config/ur3e_lab_calibration.yaml"
test -f "$CALIB" || { echo "Missing calibration $CALIB"; exit 1; }
echo 'Starting FAKE UR3e + RG2 in isolated ROS_DOMAIN_ID=73. No real UR driver.'
exec ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e onrobot_type:=rg2 robot_ip:=127.0.0.1 \
  use_fake_hardware:=true initial_joint_controller:=joint_trajectory_controller \
  kinematics_parameters_file:="$CALIB" launch_rviz:=false
