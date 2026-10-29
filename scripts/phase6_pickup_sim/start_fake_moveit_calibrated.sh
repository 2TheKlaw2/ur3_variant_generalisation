#!/usr/bin/env bash
# MoveIt + RViz against the fake robot, with the LAB CALIBRATION (ROS_DOMAIN_ID=73).
# RViz MotionPlanning display shows the planning scene, including V1 once attached.
set -eo pipefail
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
CALIB="$HOME/ros2_ws/src/ur3_variant_generalisation/config/ur3e_lab_calibration.yaml"
test -f "$CALIB" || { echo "Missing calibration $CALIB"; exit 1; }
echo 'Starting MoveIt/RViz (lab calibration) against FAKE robot in ROS_DOMAIN_ID=73.'
exec ros2 launch ur3_variant_generalisation moveit_calibrated.launch.py \
  ur_type:=ur3e onrobot_type:=rg2 use_sim_time:=false \
  launch_rviz:=true launch_servo:=false kinematics_parameters_file:="$CALIB"
