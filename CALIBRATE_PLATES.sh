#!/usr/bin/env bash
# Guided, READ-ONLY calibration of the pickup and placement plates (lab, real robot connected).
# Sends NO motion commands: you move the robot in freedrive and run the RG2 from the pendant.
#
#   bash CALIBRATE_PLATES.sh                      # new session config/calibration/<YYYYmmdd_HHMM>/
#   bash CALIBRATE_PLATES.sh --resume <session>   # continue or redo poses in a session
#   bash CALIBRATE_PLATES.sh --variants 1 2       # only some variants (e.g. a quick re-check)
#
# Saves v1_pickup_V<i>_geometry and v1_place_V<i>_geometry for each variant into the session
# folder (config/saved_poses is never touched), fits both plates (dry run) and checks the limits.
# Apply the result afterwards with:  bash UPDATE_SIM.sh config/calibration/<session>
set -Eeo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} (must be the REAL robot's domain, not the fake-hardware 73)"
if [[ "${ROS_DOMAIN_ID:-0}" == "73" ]]; then
  echo "This is the fake-hardware domain. Calibration needs the real robot's TF. Unset ROS_DOMAIN_ID first." >&2
  exit 2
fi
if ! timeout 10 ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; then
  echo "No /joint_states: start the UR driver for the real robot first (see docs/LAB_REGISTRATION.md)." >&2
  exit 2
fi
cd "$HERE"
python3 scripts/scene/calibrate_plates.py "$@"
