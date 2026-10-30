#!/usr/bin/env bash
# Apply a lab calibration session to the whole simulation and regenerate every plan (offline, no robot).
#
#   bash UPDATE_SIM.sh config/calibration/<session> [--library ~/ros2_ws/src/rs2_dmp_demo]
#
#   1. registration from the session (plates, fixture, heights)  -> config/scene/scene_registration.yaml
#      (the previous file is kept as config/scene/scene_registration.before_<session>.yaml)
#   2. scene validation (holes on slots, prongs on the edge, part/fitting seating, ...)
#   3. Stage 9 INSERT/ROTATE, all levels, all variants              -> results/simulation/stage9_<session>/
#   4. Stage 10 REACH/TRANSPORT CDMP retargeting, tau 2              -> results/simulation/stage10_<session>/
#   5. RELEASE / RETREAT / HOME after assembly                       -> same folder
#   6. results/simulation/stage10_current -> stage10_<session>   (the RViz runner's --full uses it)
# Kinematic simulation only. Old poses, demos and reference results are never overwritten.
set -Eeo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
SESSION="${1:?usage: bash UPDATE_SIM.sh config/calibration/<session> [--library DIR]}"; shift || true
LIB="${HOME}/ros2_ws/src/rs2_dmp_demo"
if [[ "${1:-}" == "--library" ]]; then LIB="$2"; fi
[[ -d "$SESSION" ]] || { echo "No session folder: $SESSION" >&2; exit 2; }
[[ -f "$LIB/cdmp/src/cdmps.py" ]] || { echo "CDMP library not found at $LIB (pass --library)" >&2; exit 2; }
NAME="$(basename "$SESSION")"
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; source "${HOME}/ros2_ws/install/setup.bash" || true; fi
RG2="${RG2_MESH_DIR:-$(ros2 pkg prefix onrobot_description 2>/dev/null)/share/onrobot_description/meshes/rg2/collision}"
[[ -d "$RG2" ]] || { echo "RG2 collision meshes not found ($RG2); set RG2_MESH_DIR" >&2; exit 2; }
THICK_MM="$(python3 -c "import yaml;print(yaml.safe_load(open('$SESSION/notes.yaml'))['fixture_thickness_mm'])")"
if grep -q "self_test: true" "$SESSION/notes.yaml" 2>/dev/null; then
  echo "Refusing: $SESSION is a self-test session (synthetic poses)." >&2; exit 2
fi
LOG="results/simulation/update_${NAME}.log"; mkdir -p results/simulation
echo "UPDATE_SIM: session $NAME, fixture ${THICK_MM} mm, log $LOG"

step() { echo; echo "==== $1"; }
step "1/6 registration"
cp config/scene/scene_registration.yaml "config/scene/scene_registration.before_${NAME}.yaml"
python3 scripts/scene/compute_scene_registration.py --session "$SESSION" \
  --fixture-thickness "$(python3 -c "print($THICK_MM/1000)")" > "$LOG" 2>&1
python3 - <<EOF
import yaml; r = yaml.safe_load(open("config/scene/scene_registration.yaml"))
for k in ("pickup_plate", "placement_plate"):
    print(f"  {k}: xyz {r[k]['xyz_m']}, yaw {r[k]['yaw_deg']} deg, fit RMS {r[k].get('fit_rms_mm')} mm")
print("  prong check:", r.get("prong_check"))
EOF
step "2/6 scene validation"
python3 scripts/scene/validate_scene.py --rg2-mesh-dir "$RG2" --out "results/simulation/scene_check_${NAME}" 2>&1 \
  | tee -a "$LOG" | grep -E "FAIL|checks" || true
step "3/6 Stage 9 INSERT/ROTATE (all levels)"
python3 scripts/phase9_variants/run_variants_offline.py --out "results/simulation/stage9_${NAME}" 2>&1 \
  | tee -a "$LOG" | grep -E "^V[1-5] " || true
step "4/6 Stage 10 REACH/TRANSPORT retargeting"
python3 scripts/phase10_full_sequence/retarget_reach_transport.py --library "$LIB" --rg2-mesh-dir "$RG2" --tau 2 \
  --out "results/simulation/stage10_${NAME}" 2>&1 | tee -a "$LOG" | grep -E "^V[1-5]" || true
step "5/6 RELEASE / RETREAT / HOME"
python3 scripts/phase10_full_sequence/plan_release_retreat.py --rg2-mesh-dir "$RG2" \
  --stage9 "results/simulation/stage9_${NAME}" --out "results/simulation/stage10_${NAME}" 2>&1 \
  | tee -a "$LOG" | grep -E "^V[1-5]" || true
step "6/6 point the RViz runner at the new plans"
ln -sfn "stage10_${NAME}" results/simulation/stage10_current
echo "  results/simulation/stage10_current -> stage10_${NAME}"
echo
echo "Done. Restart terminals 1-2 (fake robot + MoveIt/RViz) so RViz loads the new plate positions."
echo "Next (fake hardware, ROS_DOMAIN_ID=73):"
echo "  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 1 --level L2 --full --run --speed 3"
echo "Undo: cp config/scene/scene_registration.before_${NAME}.yaml config/scene/scene_registration.yaml"
