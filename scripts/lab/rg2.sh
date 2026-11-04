#!/usr/bin/env bash
# RG2 open/close through its finger_width controller (works during freedrive; the arm controller is not used).
#   bash scripts/lab/rg2.sh open     -> 40 mm over 3 s
#   bash scripts/lab/rg2.sh grip     -> 8.3 mm over 2 s (reliable for every part, 3 Nov) (stops on contact; 27 Oct: 8.5 mm command read ~10.1 mm)
#   bash scripts/lab/rg2.sh 9.5      -> any width in mm
set -Eeo pipefail
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"
case "${1:-}" in open) mm=40; t=3;; grip) mm=8.3; t=2;; ''|*[!0-9.]*) echo "usage: rg2.sh open|grip|<mm>" >&2; exit 2;; *) mm=$1; t=3;; esac
m=$(python3 -c "print($mm/1000)")
python3 -c "import sys; sys.exit(0 if 0.0 <= $m <= 0.1 else 1)" || { echo "width out of range" >&2; exit 2; }
ros2 action send_goal /finger_width_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [finger_width], points: [{positions: [$m], time_from_start: {sec: $t, nanosec: 0}}]}}" | tail -2
python3 - <<'PY'
import time, rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("rg2_width_read"); got = {}
n.create_subscription(JointState, "/joint_states", lambda m: got.update(w=m.position[m.name.index("finger_width")]) if "finger_width" in m.name else None, 10)
t0 = time.time()
while "w" not in got and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"RG2 width now {got['w']*1000:.2f} mm" if "w" in got else "RG2 width: no finger_width state seen")
n.destroy_node(); rclpy.shutdown()
PY
