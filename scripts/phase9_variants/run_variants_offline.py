#!/usr/bin/env python3
"""Stage 9 offline (no ROS): V1 demonstration transferred to V1-V5 under L0 / L1 / L2.

  python3 scripts/phase9_variants/cad_corridor.py                 # once: CAD yaw corridors
  python3 scripts/phase9_variants/run_variants_offline.py [--variants 1 2 3 4 5] [--levels L0 L1 L2]

Kinematic simulation only; not a physical result. See variant_core.py for the conditions and
docs/STAGE9_VARIANT_TRANSFER.md for the definitions and results.
Output: results/simulation/stage9_offline/V<i>_<level>/{trajectory.csv, collision_log.csv,
summary.json} and stage9_summary.json.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for d in (".", "../phase8_insert_sim", "../scene", "../phase6_pickup_sim", "../phase7_transport_sim"):
    sys.path.insert(0, os.path.join(HERE, d))
import cad_corridor as cc  # noqa: E402
import lift_profile as lp  # noqa: E402
import preview_stage8_offline as out8  # noqa: E402
import scene_model as sm  # noqa: E402
import stage8_core as s8  # noqa: E402
import transport_profile as tp  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import variant_core as vc  # noqa: E402
import yaml  # noqa: E402

np = sm.np


def grasps(objs, calib):
    """V1: the Stage 6-8 grasp (align_to_grip end). V2-V5: the measured pickup geometry poses.
    Grasp offsets come from where the plates were when those poses were taught (measured layout).
    After a lab calibration session (scene_registration.yaml: poses_dir), all five come from that
    session's v1_pickup_V<i>_geometry poses, measured on the same plates as the registration."""
    objs = sm.scene_objects(measured=True)
    reg = yaml.safe_load(open(sm.registration_path()))
    sess = reg.get("poses_dir")
    G, src = {}, {}
    if sess:
        for i in range(1, 6):
            p = os.path.join(sess, f"v1_pickup_V{i}_geometry.yaml")
            q = yaml.safe_load(open(os.path.join(sm.PKG, p)))["arm"]["positions_rad"]
            G[i] = np.linalg.inv(uk.tcp(q, calib)) @ objs[f"pickup_V{i}"]["T"]
            src[i] = p
        return G, src
    rows = list(csv.DictReader(open(os.path.join(sm.PKG, "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv"))))
    q_grip = [float(rows[-1][j]) for j in lp.JOINTS]
    G[1] = np.linalg.inv(uk.tcp(q_grip, calib)) @ objs["pickup_V1"]["T"]
    src[1] = "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv (last row), as Stage 6-8"
    for i in range(2, 6):
        p = f"config/saved_poses/v1_pickup_V{i}_geometry.yaml"
        q = yaml.safe_load(open(os.path.join(sm.PKG, p)))["arm"]["positions_rad"]
        G[i] = np.linalg.inv(uk.tcp(q, calib)) @ objs[f"pickup_V{i}"]["T"]
        src[i] = p
    return G, src


def v1_start(objs, calib, G1):
    """Stage 8 B2 state after CORRECTION: part centred/upright at its Stage 7 arrival height."""
    rows = list(csv.DictReader(open(os.path.join(sm.PKG, tp.ALIGN_INSERT_CSV))))
    q_align = [float(rows[-1][j]) for j in lp.JOINTS]
    T_fit = objs["fitting_V1"]["T"]
    T_v0 = uk.tcp(q_align, calib) @ G1
    T_vc = s8.corrected_v1_start(T_v0, T_fit)
    T_tcp_c = T_vc @ np.linalg.inv(G1)
    q_c = vc.ik_continuation(q_align, uk.tcp(q_align, calib), T_tcp_c, calib, steps=100)
    L = np.linalg.inv(T_fit) @ T_vc
    yaw0 = float(np.degrees(np.arctan2(L[1, 0], L[0, 0])))
    return q_c, T_tcp_c, L[2, 3] * 1000, yaw0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--levels", nargs="+", default=list(vc.LEVELS))
    ap.add_argument("--features", default=os.path.join(sm.PKG, "results/cad_features"))
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/stage9_offline"))
    a = ap.parse_args()
    calib = yaml.safe_load(open(os.path.join(sm.PKG, "config/ur3e_lab_calibration.yaml")))
    objs = sm.scene_objects()
    G, gsrc = grasps(objs, calib)
    prof = vc.demo_profiles(sm.PKG)
    q1, T_tcp1, z1, yaw0 = v1_start(objs, calib, G[1])
    above_top = z1 - 7.0
    rows1 = vc.load_corridor(os.path.join(a.features, "V1_corridor.csv"))
    v1_lock = vc.seat_target(rows1, prof["Y_demo"], 0.0)["lock_end_deg"]
    os.makedirs(a.out, exist_ok=True)
    spath = os.path.join(a.out, "stage9_summary.json")
    summary = json.load(open(spath)) if os.path.exists(spath) else {}
    summary["_common"] = {"type": "KINEMATIC SIMULATION (offline) - not a physical result",
                          "start": f"part centred/upright, tabs on slot axis, bottom {above_top:.3f} mm above fitting top "
                                   "(= Stage 8 B2 after CORRECTION)", "part_yaw_in_fitting_frame_deg": yaw0,
                          "demo": {"insert_csv": s8.INSERT_CSV, "rotate_csv": s8.ROTATE_CSV, "D_demo_mm": prof["D_demo"],
                                   "Y_demo_deg": prof["Y_demo"], "insert_s": prof["t_ins"][-1], "rotate_s": prof["t_rot"][-1]},
                          "v1_lock_end_deg": v1_lock, "grasp_sources": gsrc,
                          "tolerances": {"contact_mm": s8.CONTACT_TOL_MM, "floor_mm": s8.FLOOR_TOL_MM,
                                         "yaw_target_deg": vc.YAW_TARGET_TOL_DEG, "retain_lift_mm": vc.RETAIN_LIFT_MM,
                                         "corridor_margin_deg": vc.CORRIDOR_MARGIN_DEG}}
    for i in a.variants:
        T_fit = objs[f"fitting_V{i}"]["T"]
        geom = vc.VariantGeometry(i, objs, T_fit)
        z_start = geom.top_mm + above_top
        rows = vc.load_corridor(os.path.join(a.features, f"V{i}_corridor.csv"))
        feat = vc.seat_target(rows, prof["Y_demo"], v1_lock)
        T_tcp_i = vc.part_pose(T_fit, z_start, yaw0) @ np.linalg.inv(G[i])
        q_i = q1 if i == 1 else vc.ik_continuation(q1, T_tcp1, T_tcp_i, calib, steps=200)
        ctx = {"calib": calib, "T_fit": T_fit, "G": G[i], "z_start": z_start, "q_start": q_i, "yaw0": yaw0}
        for lev in a.levels:
            res = vc.run_condition(i, lev, ctx, geom, rows, feat, prof)
            res["cad_features"] = feat
            res["start"] = {"bottom_z_mm": z_start, "q": q_i, "grasp_xyz_mm": (G[i][:3, 3] * 1000).tolist()}
            res["type"] = summary["_common"]["type"]
            s = out8.write_outputs(os.path.join(a.out, f"V{i}_{lev}"), res)
            fu = s["first_unacceptable"]
            summary[f"V{i}_{lev}"] = {k: s[k] for k in ("functional_outcome", "strict_outcome", "criteria", "final",
                                                         "max_insertion_depth_mm", "cad_seat_depth_mm", "max_penetration_mm",
                                                         "touching_samples", "first_unacceptable", "cad_seat_yaw_deg")}
            print(f"V{i} {lev}: functional {s['functional_outcome']}, strict {s['strict_outcome']}; depth "
                  f"{s['max_insertion_depth_mm']:.2f}/{s['cad_seat_depth_mm']:.2f} mm; final "
                  f"{'-' if not s['final'] else format(s['final']['yaw_deg'], '.2f')} deg (CAD seat {feat['target_deg']:.2f}), gap "
                  f"{'-' if not s['final'] else format(s['final']['seating_gap_mm'], '.3f')} mm; first bad: "
                  f"{'none' if not fu else fu['stage'] + ' t=' + format(fu['t_s'] or 0, '.2f') + ' ' + str(fu['status']) + ' ' + str(fu['contact'])}",
                  flush=True)
            json.dump(summary, open(spath, "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
