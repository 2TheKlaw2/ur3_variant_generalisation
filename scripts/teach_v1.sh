#!/usr/bin/env bash
# Interactive menu for recording independent and complete V1 teaching motions.
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
while true; do
  cat <<'MENU'

=== V1 LfD TRAINING MENU — RECORD / SAVE ONLY ===
Use robot-driver and lab-approved freedrive separately.
 1  Record REACH (home -> V1 gripping pose)
 2  SAVE current V1 GRIP pose (arm stationary)
 3  Record TRANSPORT (after gripping -> V1 insertion-ready)
 4  SAVE current V1 INSERTION-READY pose
 5  Record FULL ASSEMBLY (5 mm insert + 10 deg CW) [primary research take]
 6  Record INSERTION ONLY (phase 1)
 7  SAVE current V1 INSERTION-END pose
 8  Record ROTATION ONLY (phase 2)
 9  SAVE current V1 ASSEMBLED pose
 0  Exit menu

For steps 2/4/7/9, hold the robot stationary.
Gripper close/release and freedrive controller switches are MANUAL / separate.
MENU
  read -r -p 'Choose: ' OPTION
  case "$OPTION" in
    1) bash "$HERE/record_v1.sh" v1_reach ;;
    2) bash "$HERE/save_named_pose.sh" v1_grip_pose ;;
    3) bash "$HERE/record_v1.sh" v1_transport ;;
    4) bash "$HERE/save_named_pose.sh" v1_insertion_ready ;;
    5) bash "$HERE/record_v1.sh" v1_assembly_full ;;
    6) bash "$HERE/record_v1.sh" v1_insertion ;;
    7) bash "$HERE/save_named_pose.sh" v1_insertion_end ;;
    8) bash "$HERE/record_v1.sh" v1_rotation ;;
    9) bash "$HERE/save_named_pose.sh" v1_assembled_pose ;;
    0) echo 'Menu closed.'; exit 0 ;;
    *) echo 'Choose 0–9.' ;;
  esac
  echo 'Return to menu. Confirm current robot pose and gripper state before next operation.'
done
