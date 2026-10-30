#!/usr/bin/env python3
"""Fix A for V4/V5: the SAME CDMP-retargeted Cartesian path, executed in a different arm posture.

  python3 scripts/phase10_full_sequence/posture_variant.py --library <rs2_dmp_demo> --rg2-mesh-dir <...> \
      --variant 4 --seed-q -1.73 -2.29 -1.56 -1.56 2.85 -2.27 --out results/simulation/stage10_posture

The pure goal-change TRANSPORT for V4/V5 reaches an elbow singularity on the HOME posture
(Stage 10). v45_fix_experiments.py found other IK branches on which the whole path is continuous.
This builds the complete sequence on such a branch (seed = joint vector at the lift end):
  RECONFIG      joint min-jerk from the taught HOME to the same TCP pose in the new posture
  REACH, ALIGN_GRIP, LIFT, TRANSPORT, ALIGN_INSERT   identical Cartesian targets to Stage 10
  INSERT, ROTATE                                     Stage 9 L2, re-solved on the new branch
  (release_retreat.csv) RELEASE, RETREAT +30 mm, HOME (joint min-jerk back to the taught HOME)
Checks: continuity, UR3e limits (elbow screened at +-160 deg, the more restrictive UR document),
RG2 + part FCL vs the scene, elbow/wrist height proxy, Jacobian smallest singular value.
Arm-link meshes: MoveIt on fake hardware. Kinematic simulation only.
"""
import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import plan_release_retreat as PR  # noqa: E402
import retarget_reach_transport as R  # noqa: E402
import v45_fix_experiments as X  # noqa: E402

np = R.np
uk, lp, sm, vc = R.uk, R.lp, R.sm, R.vc
ELBOW_LIMIT = math.radians(160.0)      # PolyScope 5.24 manual (datasheet says 360; screen with the stricter)
RECONFIG_PEAK = 0.3


def jmin(Q, calib):
    """Smallest singular value of the 6x6 geometric-ish Jacobian (numerical, translation in m, rotation in rad)."""
    out = []
    for q in Q:
        T0 = uk.tcp(q, calib); J = np.zeros((6, 6))
        for j in range(6):
            dq = np.array(q, float); dq[j] += 1e-6
            J[:, j] = uk._err(uk.tcp(dq, calib), T0) / 1e-6
        out.append(np.linalg.svd(J, compute_uv=False)[-1])
    return float(min(out))


def joint_path(qa, qb, peak=RECONFIG_PEAK):
    d = lp.maxdiff(qa, qb)
    T = max(3.0, 1.875 * d / peak)
    t, frac = lp.min_jerk_grid(T, lp.SAMPLE_DT_S)
    a_, b_ = np.asarray(qa), np.asarray(qb)
    return t, [(a_ + (b_ - a_) * s).tolist() for s in frac]


def limits_ok(Q):
    Q = np.asarray(Q)
    jl = X.joint_limits(Q)
    elbow_ok = bool(np.abs(Q[:, 2]).max() <= ELBOW_LIMIT)
    return {"ur_description_ok": jl["ok"], "elbow_within_160deg": elbow_ok,
            "max_abs_elbow_deg": round(float(np.degrees(np.abs(Q[:, 2]).max())), 1), "ok": jl["ok"] and elbow_ok}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", type=Path, required=True)
    ap.add_argument("--rg2-mesh-dir", required=True)
    ap.add_argument("--variant", type=int, required=True)
    ap.add_argument("--seed-q", type=float, nargs=6, required=True, help="joint vector near the lift end on the new branch")
    ap.add_argument("--tau", type=float, default=2.0)
    ap.add_argument("--level", default="L2")
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/stage10_posture"))
    a = ap.parse_args()
    import yaml
    root = Path(sm.PKG); i = a.variant
    calib = yaml.safe_load(open(root / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects()
    G, _ = R.rvo.grasps(objs, calib)
    q1, T_tcp1, z1, yaw0 = R.rvo.v1_start(objs, calib, G[1])
    col = R.Collider(objs, a.rg2_mesh_dir)
    reach, tr, pr, qr = R.train(a.library, root / R.REACH_CSV)
    trans, tt, pt, qt = R.train(a.library, root / R.TRANSPORT_CSV)
    q_home = list(PR.HOME); T_home = uk.tcp(q_home, calib)
    fix_top = objs["fixture_plate"]["T"][2, 3] + 0.0125 / 2
    T_fit = objs[f"fitting_V{i}"]["T"]; geom = vc.VariantGeometry(i, objs, T_fit)
    z_start = geom.top_mm + (z1 - 7.0)
    T_start = vc.part_pose(T_fit, z_start, yaw0) @ np.linalg.inv(G[i])
    T_grip = objs[f"pickup_V{i}"]["T"] @ np.linalg.inv(G[i])
    T_lift = T_grip.copy(); T_lift[2, 3] += R.LIFT_M
    part, z0 = f"pickup_V{i}", objs[f"pickup_V{i}"]["T"][2, 3]

    # Cartesian targets: identical to Stage 10
    t_r, Ts_r, info_r = R.rollout(reach, pr, qr, T_home, T_grip, a.tau); t_r, Ts_r = R.resample(t_r, Ts_r)
    t_t, Ts_t, info_t = R.rollout(trans, pt, qt, T_lift, T_start, a.tau); t_t, Ts_t = R.resample(t_t, Ts_t)
    t_l, frac = lp.min_jerk_grid(R.LIFT_S, lp.SAMPLE_DT_S)
    Ts_l = []
    for s_ in frac:
        T = T_grip.copy(); T[2, 3] += R.LIFT_M * s_; Ts_l.append(T)
    # solve on the new branch: from the lift end forwards (TRANSPORT) and backwards (LIFT, ALIGN, REACH)
    q_lift = X.unwrap_near(uk.ik_local(a.seed_q, T_lift, calib, iters=200).tolist(), q_home)
    Q_t = R.ik_path(q_lift, Ts_t, calib, "TRANSPORT")
    Q_l = R.ik_path(q_lift, Ts_l[::-1], calib, "LIFT (backwards)")[::-1]
    t_ag = lp.min_jerk_grid(R.ALIGN_S, lp.SAMPLE_DT_S)[0]
    Ts_ag_end = Ts_r[-1]
    import transport_profile as tp
    Ts_ag = [tp.interp_pose(Ts_ag_end, T_grip, s_) for s_ in lp.min_jerk_grid(R.ALIGN_S, lp.SAMPLE_DT_S)[1]]
    Q_ag = R.ik_path(Q_l[0], Ts_ag[::-1], calib, "ALIGN_GRIP (backwards)")[::-1]
    Q_r = R.ik_path(Q_ag[0], Ts_r[::-1], calib, "REACH (backwards)")[::-1]
    q_home_alt = Q_r[0]
    t_ai, Q_ai = R.align(Q_t[-1], T_start, calib, "ALIGN_INSERT")
    t_rc, Q_rc = joint_path(q_home, q_home_alt)
    # INSERT / ROTATE (Stage 9 L2) on the new branch
    prof = vc.demo_profiles(str(root))
    rows = vc.load_corridor(str(root / f"results/cad_features/V{i}_corridor.csv"))
    rows1 = vc.load_corridor(str(root / "results/cad_features/V1_corridor.csv"))
    v1_lock = vc.seat_target(rows1, prof["Y_demo"], 0.0)["lock_end_deg"]
    feat = vc.seat_target(rows, prof["Y_demo"], v1_lock)
    ctx = {"calib": calib, "T_fit": T_fit, "G": G[i], "z_start": z_start, "q_start": Q_ai[-1], "yaw0": yaw0}
    res9 = vc.run_condition(i, a.level, ctx, geom, rows, feat, prof)
    ins = [r for r in res9["rows"] if r["feasible"]]
    # RELEASE / RETREAT / HOME
    q_end = ins[-1]["q"]
    T_part = uk.tcp(q_end, calib) @ G[i]
    t_rt, frac = lp.min_jerk_grid(PR.RETREAT_S, lp.SAMPLE_DT_S)
    T0 = uk.tcp(q_end, calib); Ts_rt = []
    for s_ in frac:
        T = T0.copy(); T[2, 3] += PR.RETREAT_M * s_; Ts_rt.append(T)
    Q_rt = R.ik_path(q_end, Ts_rt, calib, "RETREAT")
    t_h, Q_h = joint_path(Q_rt[-1], q_home)

    stages = [("RECONFIG", t_rc, Q_rc), ("REACH", t_r, Q_r), ("ALIGN_GRIP", t_ag, Q_ag), ("LIFT", t_l, Q_l),
              ("TRANSPORT", t_t, Q_t), ("ALIGN_INSERT", t_ai, Q_ai)]
    res = {"type": "KINEMATIC SIMULATION (offline) - not a physical result", "variant": f"V{i}",
           "method": "fix A: same CDMP Cartesian path, different IK branch", "seed_q": a.seed_q,
           "home_alt_q": [round(x, 4) for x in q_home_alt], "reconfig_max_joint_rad": round(lp.maxdiff(q_home, q_home_alt), 3),
           "stage9_on_new_branch": {k: res9[k] for k in ("functional_outcome", "strict_outcome")},
           "durations_s": {n: float(t[-1]) for n, t, _ in stages}, "collision": {}}
    allQ = []
    for n, t, Q in stages:
        held = n in ("LIFT", "TRANSPORT", "ALIGN_INSERT")
        res["collision"][n] = R.check_stage(col, n, Q, calib, sm.RG2_GRIP_M if held else sm.RG2_OPEN_M,
                                            part if held else None, G[i], z0, stride=5 if len(Q) > 200 else 1)
        allQ += Q
    col.static[part][0].setTransform(col._tf(T_part))
    res["collision"]["RELEASE"] = PR.open_check(col, q_end, calib, list(np.linspace(sm.RG2_GRIP_M, sm.RG2_OPEN_M, 17)), part)
    res["collision"]["RETREAT"] = PR.path_check(col, Q_rt, calib, sm.RG2_OPEN_M, set())
    res["collision"]["HOME"] = PR.path_check(col, Q_h, calib, sm.RG2_OPEN_M, set(), stride=2)
    allQ += [r["q"] for r in ins] + Q_rt + Q_h
    res["limits"] = limits_ok(allQ)
    res["arm_proxy"] = X.arm_proxy(allQ, calib, fix_top)
    res["min_jacobian_singular_value"] = {"posture_A_transport": round(jmin(Q_t[::10], calib), 4)}
    res["peak_joint_speed_rad_s"] = {n: round(R.speeds(np.asarray(t), Q)[0], 3) for n, t, Q in stages}
    res["PASS"] = bool(all(c["collision_free"] for c in res["collision"].values()) and res["limits"]["ok"]
                       and res["arm_proxy"]["ok"] and res9["functional_outcome"] == "PASS")
    d = Path(a.out) / f"V{i}"; d.mkdir(parents=True, exist_ok=True)
    with open(d / "joint_trajectory.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stage", "t_s"] + list(lp.JOINTS))
        for n, t, Q in stages:
            for ti, q in zip(t, Q):
                w.writerow([n, f"{ti:.3f}"] + [f"{x:.9f}" for x in q])
        for r in ins:
            w.writerow([r["stage"], f"{r['t_s']:.3f}"] + [f"{x:.9f}" for x in r["q"]])
    with open(d / "release_retreat.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stage", "t_s"] + list(lp.JOINTS) + ["gripper_width_m"])
        w.writerow(["RELEASE", "0.000"] + [f"{x:.9f}" for x in q_end] + [f"{sm.RG2_OPEN_M:.4f}"])
        for n, t, Q in (("RETREAT", t_rt, Q_rt), ("HOME", t_h, Q_h)):
            for ti, q in zip(t, Q):
                w.writerow([n, f"{ti:.3f}"] + [f"{x:.9f}" for x in q] + [f"{sm.RG2_OPEN_M:.4f}"])
    sp = Path(a.out) / "posture_summary.json"
    allres = json.load(open(sp)) if sp.exists() else {}
    allres[f"V{i}"] = res
    json.dump(allres, open(sp, "w"), indent=1, default=float)
    c = res["collision"]
    print(f"V{i} posture A: {'PASS' if res['PASS'] else 'FAIL'}  reconfig {res['reconfig_max_joint_rad']} rad "
          f"({t_rc[-1]:.1f}s)  stage9 {res9['functional_outcome']}  limits {res['limits']}  arm {res['arm_proxy']['min_elbow_wrist_z_above_fixture_mm']} mm  "
          + "  ".join(f"{n}:{v.get('min_clearance_mm', v.get('min_clearance_other_mm'))}" for n, v in c.items())
          + "  " + "  ".join(f"{n} HIT {v['first_collision']}" for n, v in c.items() if v["first_collision"]), flush=True)


if __name__ == "__main__":
    main()
