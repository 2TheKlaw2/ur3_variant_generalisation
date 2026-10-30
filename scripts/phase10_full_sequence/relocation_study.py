#!/usr/bin/env python3
"""Relocation study (offline, kinematic simulation only): move the whole fixture (both blue plates and
the aluminium plate) relative to the robot, re-register, and re-run the demo-driven pipeline.

  python3 scripts/phase10_full_sequence/relocation_study.py --library <rs2_dmp_demo> --rg2-mesh-dir <...>

This tests SPATIAL RETARGETING after a correct re-registration (no new demonstration, no new teaching):
for each displacement, Stage 10 (REACH/TRANSPORT CDMP) and Stage 9 L2 (INSERT/ROTATE) are re-planned
for V1-V5 and checked exactly as in the nominal case. It does NOT test registration error; with the
rigid model, a registration error larger than the radial clearance (0.30 mm V1-V4, 0.35 mm V5)
already blocks insertion (see the doc).
Grasp offsets are unchanged (they belong to the gripper, not the plates): measured_xyz stays as taught.
"""
import argparse
import copy
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[2]
CASES = [("x+30", 30, 0, 0), ("x-30", -30, 0, 0), ("y+20 (away)", 0, 20, 0), ("y-20 (closer)", 0, -20, 0),
         ("yaw+5", 0, 0, 5), ("yaw-5", 0, 0, -5)]


def moved(reg, dx_mm, dy_mm, dyaw_deg):
    """Rigid move of the fixture about its own centre; robot and trolley stay."""
    r = copy.deepcopy(reg)
    fx = r["environment"]["fixture_plate"]["xyz_m"]
    cx, cy = fx[0], fx[1]
    a = math.radians(dyaw_deg); c, s = math.cos(a), math.sin(a)

    def tf(xyz):
        x, y = xyz[0] - cx, xyz[1] - cy
        return [round(cx + c * x - s * y + dx_mm / 1000, 6), round(cy + s * x + c * y + dy_mm / 1000, 6), xyz[2]]
    for key in ("pickup_plate", "placement_plate"):
        r[key]["xyz_m"] = tf(r[key]["xyz_m"]); r[key]["yaw_rad"] = round(r[key]["yaw_rad"] + a, 6)
        r[key]["yaw_deg"] = round(math.degrees(r[key]["yaw_rad"]), 3)
    e = r["environment"]["fixture_plate"]
    e["xyz_m"] = tf(e["xyz_m"]); e["yaw_rad"] = round(e["yaw_rad"] + a, 6)
    r["relocation"] = {"dx_mm": dx_mm, "dy_mm": dy_mm, "dyaw_deg": dyaw_deg,
                       "about": "fixture centre; dy in base y (negative = away from the robot here)"}
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", required=True)
    ap.add_argument("--rg2-mesh-dir", required=True)
    ap.add_argument("--cases", nargs="+", default=[c[0] for c in CASES])
    ap.add_argument("--out", default=str(PKG / "results/simulation/relocation"))
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    reg0 = yaml.safe_load(open(PKG / "config/scene/scene_registration.yaml"))
    # which base-y direction is "away from the robot"? (the robot base is at the origin)
    away = 1.0 if abs(reg0["pickup_plate"]["xyz_m"][1] + 0.05) > abs(reg0["pickup_plate"]["xyz_m"][1]) else -1.0
    summary = {"type": "KINEMATIC SIMULATION (offline) - not a physical result", "cases": {}}
    sp = out / "relocation_summary.json"
    if sp.exists():
        summary = json.load(open(sp))
    for name, dx, dy, dyaw in CASES:
        if name not in a.cases:
            continue
        d = out / name.split()[0]; d.mkdir(exist_ok=True)
        dy_base = dy * away                                      # "away" from the robot in base y
        reg = moved(reg0, dx, dy_base, dyaw)
        rp = d / "scene_registration.yaml"
        yaml.safe_dump(reg, open(rp, "w"), sort_keys=False)
        env = dict(os.environ, SCENE_REGISTRATION=str(rp))
        print(f"\n==== {name}: dx {dx} mm, dy {dy_base} mm (base), yaw {dyaw} deg", flush=True)
        r10 = subprocess.run([sys.executable, str(PKG / "scripts/phase10_full_sequence/retarget_reach_transport.py"),
                              "--library", a.library, "--rg2-mesh-dir", a.rg2_mesh_dir, "--tau", "2", "--stride", "10",
                              "--out", str(d / "stage10")], env=env, capture_output=True, text=True)
        r9 = subprocess.run([sys.executable, str(PKG / "scripts/phase9_variants/run_variants_offline.py"),
                             "--levels", "L2", "--out", str(d / "stage9")], env=env, capture_output=True, text=True)
        lines10 = [x for x in r10.stdout.splitlines() if x.startswith("V")]
        lines9 = [x for x in r9.stdout.splitlines() if x.startswith("V")]
        for x in lines10 + lines9:
            print("  " + x[:220], flush=True)
        if r10.returncode or r9.returncode:
            print("  ERROR:", (r10.stderr or r9.stderr)[-600:], flush=True)
        s10 = json.load(open(d / "stage10" / "stage10_summary.json"))["variants"] if (d / "stage10" / "stage10_summary.json").exists() else {}
        s9 = json.load(open(d / "stage9" / "stage9_summary.json")) if (d / "stage9" / "stage9_summary.json").exists() else {}
        summary["cases"][name] = {
            "dx_mm": dx, "dy_base_mm": dy_base, "dyaw_deg": dyaw,
            "stage10_reach_transport": {k: v["PASS"] for k, v in s10.items()},
            "stage10_failure": {k: v.get("failure") for k, v in s10.items() if not v["PASS"]},
            "stage9_L2": {k.split("_")[0]: v["functional_outcome"] for k, v in s9.items() if k.endswith("_L2")}}
        json.dump(summary, open(sp, "w"), indent=1, default=float)
    print("->", sp)


if __name__ == "__main__":
    main()
