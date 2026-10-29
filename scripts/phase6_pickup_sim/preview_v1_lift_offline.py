#!/usr/bin/env python3
"""OFFLINE preview of the simulated V1 GRIP -> ATTACH -> LIFT (no ROS, no motion).

Uses the lab-calibrated UR3e FK (validated to 0.03 mm against recorded TCPs) and a
damped-least-squares IK in place of MoveIt, so the fake-hardware run can be compared
against an independent expectation.

  python3 scripts/phase6_pickup_sim/preview_v1_lift_offline.py \
      --rg2-mesh-dir <onrobot_description>/meshes/rg2/collision --out results/simulation/v1_lift_preview
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "scene"))
import lift_profile as lp  # noqa: E402
import scene_model as sm  # noqa: E402
import validate_scene as vs  # noqa: E402  (rg2_meshes, dist)
import ur_kinematics as uk  # noqa: E402
import yaml  # noqa: E402

ALIGN_CSV = "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv"


def tcp(q, calib):
    return uk.tcp(q, calib)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rg2-mesh-dir"); ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/v1_lift_preview"))
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    calib = yaml.safe_load(open(os.path.join(sm.PKG, "config/ur3e_lab_calibration.yaml")))
    rows = list(csv.DictReader(open(os.path.join(sm.PKG, ALIGN_CSV))))
    q_grip = [float(rows[-1][j]) for j in lp.JOINTS]

    objs = sm.scene_objects()
    T_tcp0 = tcp(q_grip, calib)
    T_v1 = objs["pickup_V1"]["T"]
    T_grasp = np.linalg.inv(T_tcp0) @ T_v1          # T(gripper_tcp -> V1), fixed from GRIP

    # One IK solution per 20 ms timestamp of the min-jerk profile (same as the fake-hardware run)
    tg, frac = lp.min_jerk_grid(lp.SIM_LIFT_DURATION_S, lp.SAMPLE_DT_S)
    q_path = [np.array(q_grip)]
    for k in range(1, len(tg)):
        target = T_tcp0.copy(); target[2, 3] += lp.LIFT_M * frac[k]
        q_path.append(uk.ik_local(q_path[-1], target, calib))
    q_path, _ = lp.correct_wraps([list(x) for x in q_path], q_grip)
    qg = np.array(q_path)
    step, speed = lp.verify_path("LIFT(sim 3 s)", qg.tolist(), tg.tolist())

    # Evaluate every resampled sample with FK
    res = []
    for ti, qi in zip(tg, qg):
        Tt = tcp(qi, calib); Tv = Tt @ T_grasp
        d = Tt[:3, 3] - T_tcp0[:3, 3]; dv = Tv[:3, 3] - T_v1[:3, 3]
        tilt = math.degrees(math.acos(np.clip((np.trace(Tv[:3, :3].T @ T_v1[:3, :3]) - 1) / 2, -1, 1)))
        res.append([ti, *qi, Tt[2, 3], d[2] * 1000, math.hypot(d[0], d[1]) * 1000, Tv[2, 3], dv[2] * 1000,
                    math.hypot(dv[0], dv[1]) * 1000, tilt])
    R = np.array(res)
    v_z = np.gradient(R[:, 11], R[:, 0])
    summary = {
        "timing": {"profile": "minimum-jerk on path length", "sim_duration_s": lp.SIM_LIFT_DURATION_S,
                   "physical_reference_min_duration_s": lp.PHYSICAL_LIFT_MIN_DURATION_S,
                   "note": "3 s is a NEW simulation setting; do not compare durations with the physical lift"},
        "grasp_transform_tcp_to_V1": {"xyz_mm": (T_grasp[:3, 3] * 1000).round(3).tolist(),
                                      "quat_xyzw": [round(x, 6) for x in sm.matrix_to_quat(T_grasp[:3, :3])]},
        "samples": int(len(R)), "peak_joint_step_rad": round(step, 5), "peak_joint_speed_rad_s": round(speed, 4),
        "peak_V1_vertical_speed_mm_s": round(float(np.max(np.abs(v_z))), 2),
        "V1_rise_mm": round(float(R[-1, 11]), 4), "V1_max_xy_drift_mm": round(float(R[:, 12].max()), 4),
        "V1_max_tilt_deg": round(float(R[:, 13].max()), 5),
        "TCP_rise_mm": round(float(R[-1, 8]), 4), "TCP_max_xy_drift_mm": round(float(R[:, 9].max()), 4),
        "joint_motion_rad": [round(float(x), 5) for x in (qg[-1] - qg[0])],
    }

    if a.rg2_mesh_dir:
        import trimesh
        meshes = {}
        for k, o in objs.items():
            v, f = sm.read_stl(o["mesh"]); m = trimesh.Trimesh(v, f, process=False); m.apply_transform(o["T"]); meshes[k] = m
        v1_local = meshes["pickup_V1"].copy(); v1_local.apply_transform(np.linalg.inv(T_v1))
        worst = {"fingers_to_V2": 1e9, "fingers_to_pickup_plate": 1e9, "V1_to_V2": 1e9, "V1_to_pickup_plate_after_2mm": 1e9}
        pocket_exit_t = None
        for i in range(0, len(qg), 5):
            base_, Tt = uk.ur_fk(qg[i], calib)
            g = vs.rg2_meshes(base_, sm.RG2_GRIP_M, a.rg2_mesh_dir)
            fingers = trimesh.util.concatenate([g[k] for k in g if "finger" in k or "knuckle" in k])
            v1 = v1_local.copy(); v1.apply_transform(Tt @ T_grasp)
            worst["fingers_to_V2"] = min(worst["fingers_to_V2"], vs.dist(fingers, meshes["pickup_V2"])[0])
            worst["fingers_to_pickup_plate"] = min(worst["fingers_to_pickup_plate"], vs.dist(fingers, meshes["pickup_plate"])[0])
            worst["V1_to_V2"] = min(worst["V1_to_V2"], vs.dist(v1, meshes["pickup_V2"])[0])
            if R[i, 11] > sm.POCKET_DEPTH_M * 1000:
                pocket_exit_t = pocket_exit_t or float(R[i, 0])
                worst["V1_to_pickup_plate_after_2mm"] = min(worst["V1_to_pickup_plate_after_2mm"], vs.dist(v1, meshes["pickup_plate"])[0])
        summary["min_clearance_mm"] = {k: round(v * 1000, 2) for k, v in worst.items()}
        summary["V1_clears_pocket_at_s"] = round(pocket_exit_t, 3) if pocket_exit_t else None
        others = {k: objs[k]["T"] for k in ("pickup_V2", "pickup_V3", "pickup_V4", "pickup_V5")}
        summary["V2_V5_moved_mm"] = 0.0 if all(np.allclose(others[k], sm.scene_objects()[k]["T"]) for k in others) else None

    with open(os.path.join(a.out, "lift_preview.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", *lp.JOINTS, "tcp_z_m", "tcp_rise_mm", "tcp_xy_drift_mm", "v1_z_m", "v1_rise_mm",
                    "v1_xy_drift_mm", "v1_tilt_deg"])
        w.writerows([[round(x, 9) for x in r] for r in res])
    with open(os.path.join(a.out, "lift_preview_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
