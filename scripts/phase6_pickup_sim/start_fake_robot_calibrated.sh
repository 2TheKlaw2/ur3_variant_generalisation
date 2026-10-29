#!/usr/bin/env bash
# Fake UR3e + RG2 with the LAB CALIBRATION (ROS_DOMAIN_ID=73). No real UR driver.
# Replaces phase1_reach/start_fake_robot.sh for Stage 6: the upstream start_robot.launch.py
# ignores kinematics_parameters_file, so the old script ran the default UR3e kinematics.
set -eo pipefail
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
CALIB="$HOME/ros2_ws/src/ur3_variant_generalisation/config/ur3e_lab_calibration.yaml"
test -f "$CALIB" || { echo "Missing calibration $CALIB"; exit 1; }
echo 'Starting FAKE UR3e + RG2 (lab calibration) in ROS_DOMAIN_ID=73. No real UR driver.'
exec ros2 launch ur3_variant_generalisation fake_robot_calibrated.launch.py \
  ur_type:=ur3e onrobot_type:=rg2 robot_ip:=127.0.0.1 \
  use_fake_hardware:=true initial_joint_controller:=joint_trajectory_controller \
  kinematics_parameters_file:="$CALIB" launch_rviz:=false
