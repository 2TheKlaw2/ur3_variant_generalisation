#!/usr/bin/env bash
# Save new stationary home into NEW project as a timestamped YAML; no robot movement.
# Preserves ~/ros2_ws/src/ur3_lfd/config/home.yaml and final_home.yaml unchanged.
set -Eeo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/../.." && pwd)"
OUT="$PROJECT/config/saved_poses"
mkdir -p "$OUT"
STAMP="$(date +%Y%m%d_%H%M%S)"
NAME="v1_lab_home_${STAMP}"
FRAME="${UR3E_HOME_TOOL_FRAME:-tool0}"
cat <<MSG
New project home measurement (NO motion commanded)
Output: $OUT/$NAME.yaml
TF: base -> $FRAME
Requires: robot stationary; valid /joint_states and TF.
Note: tool0 works with the bare UR driver; gripper_tcp requires the combined RG2 model.
Do not save while guiding the robot or while freedrive is moving the arm.
MSG
read -r -p 'With the robot STATIONARY at the desired new home, press ENTER to capture (Ctrl+C cancels): ' _
exec ros2 run ur3_variant_generalisation save_named_pose "$NAME" --ros-args \
  -p "output_dir:=$OUT" \
  -p 'base_frame:=base' \
  -p "tool_frame:=$FRAME"
