#!/usr/bin/env python3
"""Table of the physical runs from the committed per-run summaries (status.json + wrench_summary.json). No ROS.

  python3 scripts/analysis/physical_summary.py [--plans stage10_depthfix] [--csv results/analysis/physical_runs.csv]

dF = largest change of |F| from the value at the stage start (tool0 frame), i.e. contact force with the
gripper/part weight removed; T = max |torque|. Controller completion only: seated/locked come from the
operator's trial sheet.
"""
import argparse
import csv
import glob
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plans", default="stage10_depthfix", help="only runs that used this plans folder ('' = all)")
    ap.add_argument("--csv", default="results/analysis/physical_runs.csv")
    a = ap.parse_args()
    rows = []
    for d in sorted(glob.glob(str(ROOT / "results/physical/full_cycle/V*_*"))):
        try:
            s = json.load(open(f"{d}/status.json"))
        except (OSError, ValueError):
            continue
        plans = os.path.basename(str(s.get("plans", "")).rstrip("/"))
        if a.plans and plans != a.plans:
            continue
        st = {}
        if os.path.exists(f"{d}/wrench_summary.json"):
            st = {x["stage"]: x for x in json.load(open(f"{d}/wrench_summary.json")).get("stages", []) if x.get("available")}
        done = [k for k in ("INSERT", "ROTATE") if k in st]
        g = lambda k, m: round(st[k][m], 2) if k in st else ""
        rows.append({"run": os.path.basename(d), "variant": s.get("variant"), "level": s.get("level", "L2"),
                     "temporal": s.get("temporal", "fixed"), "plans": plans,
                     "insert_and_rotate_executed": len(done) == 2,
                     "insert_dF_N": g("INSERT", "max_abs_force_change_from_stage_start_N"),
                     "insert_T_Nm": g("INSERT", "max_abs_torque_Nm"),
                     "rotate_dF_N": g("ROTATE", "max_abs_force_change_from_stage_start_N"),
                     "rotate_T_Nm": g("ROTATE", "max_abs_torque_Nm")})
    out = ROOT / a.csv
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["run"]); w.writeheader(); w.writerows(rows)
    print(f"{'run':28s} {'lvl':4s} {'tmp':5s} INSERT dF/T      ROTATE dF/T")
    for r in rows:
        if r["insert_and_rotate_executed"]:
            print(f"{r['run']:28s} {r['level']:4s} {r['temporal']:5s} {r['insert_dF_N']:6} N {r['insert_T_Nm']:5} Nm"
                  f"   {r['rotate_dF_N']:6} N {r['rotate_T_Nm']:5} Nm")
    print(f"{len(rows)} runs ({sum(r['insert_and_rotate_executed'] for r in rows)} with INSERT+ROTATE) -> {a.csv}")


if __name__ == "__main__":
    main()
