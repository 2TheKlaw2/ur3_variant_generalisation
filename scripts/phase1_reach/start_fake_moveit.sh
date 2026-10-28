#!/usr/bin/env bash
set -eo pipefail
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
CALIB="$HOME/ros2_ws/src/ur3_variant_generalisation/config/ur3e_lab_calibration.yaml"
test -f "$CALIB" || { echo "Missing calibration $CALIB"; exit 1; }
echo 'Starting MoveIt/RViz against FAKE robot in ROS_DOMAIN_ID=73.'
echo 'use_sim_time false: fake ros2_control uses wall-clock, no /clock required.'
echo 'Planner only; separate guarded script controls fake JTC.'
exec ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e onrobot_type:=rg2 use_sim_time:=false \
  launch_rviz:=true launch_servo:=false \
  kinematics_parameters_file:="$CALIB"
