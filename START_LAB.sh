#!/usr/bin/env bash
# Lab-day readiness check: READ-ONLY (sends no motion, changes nothing). Run it in its own terminal after
# the real robot, RG2 and MoveIt are up. It tells you what is missing and what to run next.
#
#   bash START_LAB.sh            # full check
#   bash START_LAB.sh --offline  # only the laptop/repo checks (no robot needed)
# Checklist: docs/LAB_DAY_CHECKLIST.md
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"; cd "$HERE"
source /opt/ros/humble/setup.bash 2>/dev/null; source "${HOME}/ros2_ws/install/setup.bash" 2>/dev/null
IP="${UR3E_IP:-192.168.0.192}"
ok=0; bad=0
pass() { echo "  [ OK ] $*"; ok=$((ok+1)); }
fail() { echo "  [FAIL] $*"; bad=$((bad+1)); }
info() { echo "  [ .. ] $*"; }

echo "== Laptop / repo"
br="$(git rev-parse --abbrev-ref HEAD 2>/dev/null)"
[[ "$br" == "static-variant-scene" ]] && pass "branch $br ($(git log --oneline -1 | cut -c1-60))" || fail "branch is '$br' (expected static-variant-scene)"
[[ -z "$(git status --porcelain --untracked-files=no)" ]] && pass "no uncommitted changes to tracked files" || info "uncommitted changes present (git status)"
[[ -f "$HOME/ros2_ws/src/rs2_dmp_demo/cdmp/src/cdmps.py" ]] && pass "Victor's CDMP library found" || fail "CDMP library missing at ~/ros2_ws/src/rs2_dmp_demo (needed by UPDATE_SIM.sh)"
python3 -c "import fcl, trimesh, networkx, scipy, yaml" 2>/dev/null && pass "python deps (fcl, trimesh, networkx, scipy, yaml)" || fail "python deps missing: pip install --user python-fcl trimesh networkx"
n=$(ls results/.plan_cache 2>/dev/null | wc -l); [[ $n -ge 5 ]] && pass "plan cache has $n plans" || info "plan cache has $n plans (first RViz/physical run will plan for ~1-3 min)"
latest="$(ls -d config/calibration/*/ 2>/dev/null | grep -v _TEST | tail -1)"
[[ -n "$latest" ]] && info "latest calibration session: $latest" || info "no calibration session yet (run CALIBRATE_PLATES.sh)"
python3 - <<'EOF'
import yaml
r = yaml.safe_load(open("config/scene/scene_registration.yaml"))
print(f"  [ .. ] scene registration layout: {r.get('layout')}  poses_dir: {r.get('poses_dir', '-')}")
if r.get("layout") != "session":
    print("  [ .. ] -> the simulation still uses the PREDICTED plate positions: calibrate before any physical run")
EOF
[[ "${1:-}" == "--offline" ]] && { echo; echo "$ok OK, $bad FAIL (offline checks only)"; exit $(( bad > 0 )); }

echo "== Robot"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
[[ "${ROS_DOMAIN_ID:-0}" == "0" ]] && pass "real-robot domain (0)" || fail "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (real robot needs 0: unset ROS_DOMAIN_ID)"
ping -c 1 -W 2 "$IP" >/dev/null 2>&1 && pass "robot reachable at $IP" || fail "no ping to $IP (cable / static IP 192.168.0.10/24?)"
topics="$(timeout 10 ros2 topic list 2>/dev/null)"
grep -q '^/joint_states$' <<<"$topics" && pass "/joint_states published" || fail "no /joint_states (start the UR + RG2 driver, press Play on External Control)"
hw="$(timeout 10 ros2 control list_hardware_components 2>/dev/null)"
grep -q "URPositionHardwareInterface" <<<"$hw" && pass "real UR hardware interface" || fail "real UR hardware interface not found"
grep -q "RGHardwareInterface" <<<"$hw" && pass "real RG2 hardware interface" || fail "RG2 hardware interface not found"
grep -qi "mock" <<<"$hw" && fail "MOCK hardware present in this domain" || pass "no mock hardware"
ctl="$(timeout 10 ros2 control list_controllers 2>/dev/null)"
grep -q "scaled_joint_trajectory_controller.*active" <<<"$ctl" && pass "scaled_joint_trajectory_controller active" || fail "scaled_joint_trajectory_controller not active"
grep -q "finger_width_trajectory_controller.*active" <<<"$ctl" && pass "RG2 finger_width controller active" || fail "RG2 controller not active"
timeout 10 ros2 service list 2>/dev/null | grep -q '^/check_state_validity$' && pass "MoveIt running (/check_state_validity)" || fail "MoveIt not running for the real robot"
echo
echo "$ok OK, $bad FAIL"
cat <<'MSG'

Next (see docs/LAB_DAY_CHECKLIST.md):
  1. bash CALIBRATE_PLATES.sh                          (read-only, guided)
  2. bash UPDATE_SIM.sh config/calibration/<session>  (offline)
  3. fake hardware, ROS_DOMAIN_ID=73: run_variant_insert_sim.py --variant 1 --level L2 --full --run
  4. python3 scripts/phase6_pickup_sim/planning_scene_tools.py --load --physical
  5. python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1 --until ALIGN_INSERT --execute
MSG
exit $(( bad > 0 ))
