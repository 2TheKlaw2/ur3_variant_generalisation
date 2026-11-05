#!/usr/bin/env bash
# Rebuild ONE variant's plans after a fitting touch-up (scripts/lab/insert_touchup.py --save), ~8 min, no robot motion.
#   bash scripts/lab/rebuild_variant.sh <variant> [plans_dir]      (default plans_dir results/simulation/stage10_fitfix)
# Registration (applies the session's touch-ups) -> Stage 9 L2 for that variant -> Stage 10 -> release/retreat, merged
# into plans_dir. Only that variant's fitting moved, so the other variants' plans stay valid; the merged summary takes
# the new registration fingerprint. Then: planning_scene_tools.py --reset --physical, preflight, run.
set -Eeo pipefail
V="${1:?variant 1-5}"; PLANS="${2:-results/simulation/stage10_fitfix}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$HERE"
source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"
export PYTHONUNBUFFERED=1
RG2="$(ros2 pkg prefix onrobot_description)/share/onrobot_description/meshes/rg2/collision"
LIB="${CDMP_LIBRARY:-$HOME/ros2_ws/src/rs2_dmp_demo}"
SESS="$(python3 -c "import yaml;print(yaml.safe_load(open('config/scene/scene_registration.yaml'))['poses_dir'])")"
TH="$(python3 -c "import yaml;print(yaml.safe_load(open('$SESS/notes.yaml'))['fixture_thickness_mm']/1000)")"
[[ -d "$PLANS/V$V" ]] || { echo "no $PLANS/V$V (unpack the plans first)" >&2; exit 2; }
TMP="results/simulation/rebuild_V${V}_$(date +%H%M%S)"; mkdir -p "$TMP"
echo "== registration ($SESS, touch-ups applied)"
python3 scripts/scene/compute_scene_registration.py --session "$SESS" --fixture-thickness "$TH" 2>&1 | grep -E "touchup|layout" || true
grep -A3 "touchup_offsets_mm" config/scene/scene_registration.yaml || true
echo "== Stage 9 V$V L2"
python3 scripts/phase9_variants/run_variants_offline.py --variants "$V" --levels L2 --out "$TMP/stage9" | grep -E "^V$V"
echo "== Stage 10 V$V"
python3 scripts/phase10_full_sequence/retarget_reach_transport.py --library "$LIB" --rg2-mesh-dir "$RG2" --tau 2 \
  --variants "$V" --out "$TMP/s10" | grep -E "^V$V"
echo "== release/retreat V$V"
python3 scripts/phase10_full_sequence/plan_release_retreat.py --rg2-mesh-dir "$RG2" --variants "$V" \
  --stage9 "$TMP/stage9" --out "$TMP/s10" | grep -E "^V$V"
python3 - "$V" "$PLANS" "$TMP/s10" <<'EOF'
import json, os, shutil, sys
v, plans, new = sys.argv[1], sys.argv[2], sys.argv[3]
s_new = json.load(open(f"{new}/stage10_summary.json")); r_new = json.load(open(f"{new}/release_retreat_summary.json"))
if not s_new["variants"][f"V{v}"].get("PASS") or not r_new["variants"][f"V{v}"].get("PASS"):
    sys.exit(f"V{v} did not PASS: {plans} left unchanged")
s = json.load(open(f"{plans}/stage10_summary.json")); r = json.load(open(f"{plans}/release_retreat_summary.json"))
s["variants"][f"V{v}"] = s_new["variants"][f"V{v}"]; r["variants"][f"V{v}"] = r_new["variants"][f"V{v}"]
s["registration_sha256"] = s_new["registration_sha256"]          # only V<v>'s fitting moved
s.setdefault("rebuilt_variants", []).append(f"V{v}")
shutil.rmtree(f"{plans}/V{v}"); shutil.copytree(f"{new}/V{v}", f"{plans}/V{v}")
json.dump(s, open(f"{plans}/stage10_summary.json", "w"), indent=1); json.dump(r, open(f"{plans}/release_retreat_summary.json", "w"), indent=1)
print(f"merged V{v} into {plans}")
EOF
python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset --physical
echo "Done. Preflight: python3 scripts/phase11_physical/run_full_cycle_physical.py --plans $PLANS --variant $V --level <lvl>"
