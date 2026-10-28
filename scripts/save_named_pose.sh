#!/usr/bin/env bash
# Capture current stationary joints + gripper_tcp TF to a NEW named YAML, never canonical home.
set -Eeo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd -- "$HERE/.." && pwd)"
if [[ $# -ne 1 || ! "$1" =~ ^v1_[A-Za-z0-9_]+$ ]]; then
  echo 'Usage: bash scripts/save_named_pose.sh v1_grip_pose|v1_insertion_ready|v1_insertion_end|v1_assembled_pose' >&2
  exit 2
fi
NAME="$1"
DIR="$PROJECT/config/saved_poses"
mkdir -p "$DIR"
if [[ -e "$DIR/$NAME.yaml" ]]; then
  echo "ERROR: Pose already exists, refusing overwrite: $DIR/$NAME.yaml" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
ros2 run ur3_variant_generalisation save_named_pose "$NAME" --ros-args \
  -p "output_dir:=$DIR" \
  -p 'base_frame:=base' \
  -p 'tool_frame:=gripper_tcp'
