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
#   6. results/simulation/stage10_current -> stage10_<session>   (the RViz runner's --full uses it); pre-plans V1-V5 L2
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
# run STEP_FILTER cmd...: show the filtered lines; STOP if the program crashed or exited non-zero.
# validate_scene.py exits 1 when a check FAILs (expected, reported); only a traceback stops it.
run() {
  local filt="$1" soft="$2"; shift 2
  local tmp; tmp="$(mktemp)"
  set +e; "$@" > "$tmp" 2>&1; local rc=$?; set -e
  cat "$tmp" >> "$LOG"; grep -E "$filt" "$tmp" || true
  if grep -q "^Traceback" "$tmp" || { [[ $rc -ne 0 ]] && [[ "$soft" != "soft" ]]; }; then
    echo "STOPPED: this step failed (exit $rc). Last lines:" >&2; tail -15 "$tmp" >&2; rm -f "$tmp"; exit 3
  fi
  rm -f "$tmp"
}
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
run "FAIL|checks" soft python3 scripts/scene/validate_scene.py --rg2-mesh-dir "$RG2" --out "results/simulation/scene_check_${NAME}"
step "3/6 Stage 9 INSERT/ROTATE (all levels)"
run "^V[1-5] " hard python3 scripts/phase9_variants/run_variants_offline.py --out "results/simulation/stage9_${NAME}"
step "4/6 Stage 10 REACH/TRANSPORT retargeting"
run "^V[1-5]" hard python3 scripts/phase10_full_sequence/retarget_reach_transport.py --library "$LIB" --rg2-mesh-dir "$RG2" \
  --tau 2 --out "results/simulation/stage10_${NAME}"
step "5/6 RELEASE / RETREAT / HOME"
run "^V[1-5]" hard python3 scripts/phase10_full_sequence/plan_release_retreat.py --rg2-mesh-dir "$RG2" \
  --stage9 "results/simulation/stage9_${NAME}" --out "results/simulation/stage10_${NAME}"
step "6/6 point the RViz runner at the new plans and pre-plan its INSERT/ROTATE (so RViz runs start at once)"
python3 - "$NAME" <<'PYEOF'
import json, sys
d = f"results/simulation/stage10_{sys.argv[1]}"
s10 = json.load(open(f"{d}/stage10_summary.json"))["variants"]
try:
    rel = json.load(open(f"{d}/release_retreat_summary.json")).get("variants", {})
except FileNotFoundError:
    rel = {}
ok10 = [k for k, v in sorted(s10.items()) if v.get("PASS")]
okrel = [k for k, v in sorted(rel.items()) if v.get("PASS")]
print(f"  Stage 10 PASS: {ok10}   release/retreat PASS: {okrel}")
bad = sorted(set(s10) - set(ok10)) + sorted(set(rel) - set(okrel))
if bad:
    print(f"  WARNING: not executable on the robot: {sorted(set(bad))} (the runners refuse failed plans)")
PYEOF
ln -sfn "stage10_${NAME}" results/simulation/stage10_current
echo "  results/simulation/stage10_current -> stage10_${NAME}"
python3 scripts/phase9_variants/run_variant_insert_sim.py --variant all --level L2 --plan-only 2>&1 \
  | grep -E "planned|cached|expected outcome" || true
echo
echo "Done. Restart terminals 1-2 (fake robot + MoveIt/RViz) so RViz loads the new plate positions."
echo "Next (fake hardware, ROS_DOMAIN_ID=73):"
echo "  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 1 --level L2 --full --run --speed 3"
echo "Undo: cp config/scene/scene_registration.before_${NAME}.yaml config/scene/scene_registration.yaml"
