#!/usr/bin/env python3
"""Offline (no ROS) Stage 8 preview: A, B1, B2 from the Stage 7 ALIGN_INSERT end state.

  python3 scripts/phase8_insert_sim/preview_stage8_offline.py [--out DIR]
Needs numpy, scipy, pyyaml, trimesh, rtree, python-fcl.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stage8_core as s8  # noqa: E402
import lift_profile as lp  # noqa: E402
import scene_model as sm  # noqa: E402
import transport_profile as tp  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import yaml  # noqa: E402

np = sm.np


def write_outputs(folder, res):
    os.makedirs(folder, exist_ok=True)
    rows = res["rows"]
    with open(os.path.join(folder, "trajectory.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stage", "sample", "t_s", *lp.JOINTS, "cmd_x_mm", "cmd_y_mm", "cmd_z_mm", "exec_x_mm", "exec_y_mm",
                    "exec_z_mm", "unused_descent_mm", "v1_lat_x_mm", "v1_lat_y_mm", "v1_centre_offset_mm", "v1_lowest_z_mm",
                    "v1_yaw_deg", "v1_tilt_deg", "min_dist_mm", "penetration_mm", "status", "feasible"])
        for r in rows:
            w.writerow([r["stage"], r["sample"], round(r["t_s"], 4), *r["q"], *np.round(r["cmd_xyz_mm"], 5),
                        *np.round(r["exec_xyz_mm"], 5), round(r["clamp_mm"], 5), round(r["lat_x_mm"], 5),
                        round(r["lat_y_mm"], 5), round(r["lat_mm"], 5), round(r["lowest_z_mm"], 5), round(r["yaw_deg"], 5),
                        round(r["tilt_deg"], 5), round(r["min_dist_mm"], 5), round(r["pen_mm"], 5), r["status"], r["feasible"]])
    with open(os.path.join(folder, "collision_log.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stage", "sample", "t_s", "status", "contact", "penetration_mm", "regions", "v1_centre_offset_mm",
                    "v1_lowest_z_mm", "v1_yaw_deg", "v1_tilt_deg"])
        for r in rows:
            if r["status"] != "clear":
                w.writerow([r["stage"], r["sample"], round(r["t_s"], 4), r["status"], r["contact"], round(r["pen_mm"], 5),
                            json.dumps(r["regions"]), round(r["lat_mm"], 5), round(r["lowest_z_mm"], 5),
                            round(r["yaw_deg"], 5), round(r["tilt_deg"], 5)])
    summ = {k: v for k, v in res.items() if k != "rows"}
    with open(os.path.join(folder, "summary.json"), "w") as f:
        json.dump(summ, f, indent=2, default=float)
    return summ


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/stage8_offline"))
    ap.add_argument("--only", choices=list(s8.EXPERIMENTS))
    a = ap.parse_args()
    calib = yaml.safe_load(open(os.path.join(sm.PKG, "config/ur3e_lab_calibration.yaml")))
    rows = list(csv.DictReader(open(os.path.join(sm.PKG, "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv"))))
    q_grip = [float(rows[-1][j]) for j in lp.JOINTS]
    rows = list(csv.DictReader(open(os.path.join(sm.PKG, tp.ALIGN_INSERT_CSV))))
    q_align = [float(rows[-1][j]) for j in lp.JOINTS]
    objs = sm.scene_objects()
    T_grasp = np.linalg.inv(uk.tcp(q_grip, calib)) @ objs["pickup_V1"]["T"]
    T_fit = objs["fitting_V1"]["T"]
    geom = s8.Geometry(objs, T_fit)
    for exp, name in s8.EXPERIMENTS.items():
        if a.only and exp != a.only:
            continue
        res = s8.run_experiment(exp, q_align, calib, T_grasp, T_fit, objs, sm.PKG, geom)
        s = write_outputs(os.path.join(a.out, name), res)
        print(f"\n=== {exp} ({name}) -> {s['outcome']}")
        print(json.dumps({k: s[k] for k in ("initial", "stages", "first_unacceptable", "final", "max_insertion_depth_mm",
                                            "min_clearance_mm", "max_penetration_mm", "contact_samples", "outcome_criteria")},
                         indent=1, default=lambda x: round(float(x), 4)))


if __name__ == "__main__":
    main()
