#!/usr/bin/env bash
# READ-ONLY connection diagnostics; never sends movement, scripts or controller switches.
set -Eeo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
IP="${UR3E_IP:-192.168.0.192}"
run() { echo; echo "===== $1 ====="; shift; "$@" || true; }
run 'NETWORK ROUTE TO ROBOT (check src IP/interface)' ip -4 route get "$IP"
run 'PING ROBOT (reachability only, not External Control handshake)' ping -c 3 -W 1 "$IP"
run 'ROS CONTROLLERS (read-only)' timeout 8s ros2 control list_controllers
run 'ONE JOINT-STATE SAMPLE' timeout 8s ros2 topic echo /joint_states --once
run 'EXTERNAL CONTROL PROGRAM STATUS' timeout 8s ros2 topic echo /io_and_status_controller/robot_program_running --once
run 'CONTROLLER ACTION NAMES (presence only)' ros2 action list
cat <<'MSG'

Compare the above with the FIRST terminal running the official driver.
If there is a 'Connection to reverse interface dropped' message, check
whether the pendant program stopped, a safety stop occurred, or another
script/controller application interfered. Do not force controller restarts.
MSG
