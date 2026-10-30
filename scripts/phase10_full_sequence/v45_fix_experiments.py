#!/usr/bin/env python3
"""V4/V5 TRANSPORT fix experiments (offline, kinematic simulation only).

The pure goal-change CDMP TRANSPORT for V4/V5 drives the UR3e into an elbow singularity
(Stage 10). Two candidate fixes are tested here, unchanged REACH/LIFT otherwise:

  A  ARM POSTURE: same Cartesian CDMP path, different IK configuration (branch).
     Distinct IK solutions at the TRANSPORT start are found from many seeds; each is walked
     along the whole TRANSPORT path, and REACH is walked backwards from its grip pose to the
     HOME TCP on the same branch (so the robot would need a re-configuration at HOME).
  B  HEIGHT-LIMIT COUPLING TERM: Victor's CDMP rollout with an external force on z
     (ext_force, the library's coupling input) that acts only above a ceiling:
        f_z = -K_c (z - z_cap) - D_c v_z    for z > z_cap,   0 otherwise
     The same learned weights; only the coupling term is added. Ceilings are swept.

Every candidate: IK every 20 ms seeded from the previous sample (no branch jumps), RG2 +
held part FCL-checked against all scene objects, and an arm-height proxy (elbow and wrist
joint origins must stay >= 60 mm above the fixture top). Arm-link meshes are NOT checked
offline (MoveIt does that on fake hardware).

  python3 scripts/phase10_full_sequence/v45_fix_experiments.py --library <rs2_dmp_demo> --rg2-mesh-dir <...>
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import retarget_reach_transport as R  # noqa: E402

np = R.np
uk, lp, sm, vc = R.uk, R.lp, R.sm, R.vc
ARM_MIN_Z_MM = 60.0          # elbow / wrist joint origins above the fixture top (proxy for link clearance)
CAPS_MM = [180, 160, 140, 120, 100]


def joint_origins(q, calib):
    """Base-frame origins of the shoulder..wrist_3 joint frames (calibrated chain, as ur_fk)."""
    k = calib["kinematics"]; M = uk.T(r=(0, 0, math.pi)); out = []
    Binv = np.linalg.inv(uk.T(r=(0, 0, math.pi)))
    for name, qi in zip(["shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"], q):
        e = k[name]
        M = M @ uk.T((e["x"], e["y"], e["z"]), (e["roll"], e["pitch"], e["yaw"])) @ uk.axis_angle([0, 0, 1.0], qi)
        out.append((Binv @ M)[:3, 3])
    return np.array(out)


def arm_proxy(Q, calib, fixture_top_z):
    lo = min(joint_origins(q, calib)[2:, 2].min() for q in Q[::10] + Q[-1:])
    return {"min_elbow_wrist_z_above_fixture_mm": round((lo - fixture_top_z) * 1000, 1),
            "ok": bool((lo - fixture_top_z) * 1000 >= ARM_MIN_Z_MM)}


def walk(q0, Ts, calib):
    """IK walk; returns (Q, None) or (partial Q, failure dict)."""
    q, out = list(q0), []
    for k, T in enumerate(Ts):
        try:
            q = uk.ik_local(q, T, calib, iters=200).tolist()
        except RuntimeError:
            return out, {"sample": k, "tcp_mm": (T[:3, 3] * 1000).round(1).tolist(),
                         "q": np.round(q, 3).tolist(), "reason": "IK (singular/unreachable on this branch)"}
        if out and lp.maxdiff(out[-1], q) > 0.1:
            return out, {"sample": k, "reason": f"branch jump {lp.maxdiff(out[-1], q):.3f} rad"}
        out.append(q)
    return out, None


def capped_rollout(c, demo_p, demo_q, start_T, goal_T, tau, z_cap, Kc=None):
    """R.rollout with a ceiling coupling term on z (ext_force), same learned weights."""
    from scipy.spatial.transform import Rotation
    start_p, start_q = start_T[:3, 3], R.xyzw_to_wxyz(Rotation.from_matrix(start_T[:3, :3]).as_quat())
    if np.dot(start_q, demo_q[0]) < 0:
        start_q = -start_q
    gp, gq = goal_T[:3, 3], Rotation.from_matrix(goal_T[:3, :3])
    q_rel = (gq * Rotation.from_quat(R.wxyz_to_xyzw(demo_q[-1])).inv()).as_quat()
    with R.quiet():
        c.init_reproduction(tau_scaler=tau, rep_abspose_start=np.r_[start_p, start_q],
                            rep_relpose_goal=np.r_[gp - demo_p[-1], R.xyzw_to_wxyz(q_rel)])
    Kc = Kc if Kc is not None else 20.0 * c.K
    Dc = 2.0 * math.sqrt(Kc) * c.rep_tau                    # critically damped in the library's scaling
    active = 0

    def force():
        nonlocal active
        z, vz = c.rep_pos[-1, 2], c.rep_linVel[-1, 2]
        if z_cap is None or z <= z_cap:
            return np.zeros(3)
        active += 1
        return np.array([0.0, 0.0, -Kc * (z - z_cap) - Dc * vz])
    for s in c.rep_cs.s_track:
        c.rollout_step(curr_s_step=float(s), ext_force=force())
    phase, decay, n = float(c.rep_cs.s_track[-1]), math.exp(-c.alpha_s * c.dt / c.rep_tau), 0

    def err():
        return float(np.linalg.norm(np.asarray(c.rep_pos[-1]) - gp) * 1000)
    while n < int(math.ceil(1.5 * c.rep_tau / c.dt)) and err() > 2.0:
        phase *= decay; c.rollout_step(curr_s_step=phase, ext_force=force()); n += 1
    t = np.asarray(c.rep_time, float).reshape(-1)
    P = np.asarray(c.rep_pos, float); Q = np.asarray(c.rep_quat, float)
    Q = np.column_stack([Q[:, 1], Q[:, 2], Q[:, 3], Q[:, 0]]); Q /= np.linalg.norm(Q, axis=1)[:, None]
    Ts = []
    for pi, qi in zip(P, Q):
        T = np.eye(4); T[:3, :3] = Rotation.from_quat(qi).as_matrix(); T[:3, 3] = pi; Ts.append(T)
    return t - t[0], Ts, {"coupling_active_steps": active, "goal_err_mm": err(), "max_tcp_z_mm": float(P[:, 2].max() * 1000)}


def distinct_solutions(T, calib, seed_q, n=300, rng=None):
    rng = rng or np.random.default_rng(0)
    sols = []
    for _ in range(n):
        s = np.array(seed_q) + rng.uniform(-math.pi, math.pi, 6)
        try:
            q = uk.ik_local(s, T, calib, iters=200)
        except RuntimeError:
            continue
        q = (q + math.pi) % (2 * math.pi) - math.pi            # compare branches modulo 2pi
        if all(np.abs(((q - o + math.pi) % (2 * math.pi)) - math.pi).max() > 0.05 for o in sols):
            sols.append(q)
    return sols


def unwrap_near(q, ref):
    q = np.array(q, float)
    return (ref + ((q - ref + math.pi) % (2 * math.pi) - math.pi)).tolist()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", type=Path, required=True)
    ap.add_argument("--rg2-mesh-dir", required=True)
    ap.add_argument("--variants", nargs="+", type=int, default=[4, 5])
    ap.add_argument("--tau", type=float, default=2.0)
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/stage10_v45_fix"))
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    import csv
    import yaml
    root = Path(sm.PKG)
    calib = yaml.safe_load(open(root / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects()
    G, _ = R.rvo.grasps(objs, calib)
    q1, T_tcp1, z1, yaw0 = R.rvo.v1_start(objs, calib, G[1])
    col = R.Collider(objs, a.rg2_mesh_dir)
    reach, tr, pr, qr = R.train(a.library, root / R.REACH_CSV)
    trans, tt, pt, qt = R.train(a.library, root / R.TRANSPORT_CSV)
    r0 = next(csv.DictReader(open(root / R.REACH_CSV)))
    q_home = [float(r0[j]) for j in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
    T_home = uk.tcp(q_home, calib)
    fix_top = objs["fixture_plate"]["T"][2, 3] + 0.0125 / 2
    out = {"type": "KINEMATIC SIMULATION (offline) - not a physical result", "tau": a.tau,
           "arm_check": f"proxy only: elbow/wrist joint origins >= {ARM_MIN_Z_MM} mm above the fixture top; "
                        "arm meshes checked by MoveIt on fake hardware", "variants": {}}

    for i in a.variants:
        res = {}
        T_fit = objs[f"fitting_V{i}"]["T"]
        geom = vc.VariantGeometry(i, objs, T_fit)
        T_start = vc.part_pose(T_fit, geom.top_mm + (z1 - 7.0), yaw0) @ np.linalg.inv(G[i])
        T_grip = objs[f"pickup_V{i}"]["T"] @ np.linalg.inv(G[i])
        T_lift = T_grip.copy(); T_lift[2, 3] += R.LIFT_M
        part, z0 = f"pickup_V{i}", objs[f"pickup_V{i}"]["T"][2, 3]

        def held_check(Q, n):
            return R.check_stage(col, n, Q, calib, sm.RG2_GRIP_M, part, G[i], z0, stride=5)

        # ---------------- A: arm posture (same Cartesian path)
        t_t, Ts_t, _ = R.rollout(trans, pt, qt, T_lift, T_start, a.tau)
        t_t, Ts_t = R.resample(t_t, Ts_t)
        t_r, Ts_r, _ = R.rollout(reach, pr, qr, T_home, T_grip, a.tau)
        t_r, Ts_r = R.resample(t_r, Ts_r)
        sols = distinct_solutions(T_lift, calib, q_home)
        A = []
        for b, qb in enumerate(sols):
            qb = unwrap_near(qb, q_home)
            Qt, fail = walk(qb, Ts_t, calib)
            entry = {"branch": b, "q_at_lift_end": np.round(qb, 3).tolist(),
                     "transport": "PASS" if fail is None else fail}
            if fail is None:
                Qr_back, fr = walk(uk.ik_local(qb, T_grip, calib).tolist(), Ts_r[::-1], calib)
                entry["reach_same_branch"] = "PASS" if fr is None else fr
                entry["arm_proxy"] = arm_proxy(Qt + (Qr_back if fr is None else []), calib, fix_top)
                entry["gripper_part_collision"] = held_check(Qt, "TRANSPORT")
                if fr is None:
                    entry["home_on_this_branch"] = np.round(Qr_back[-1], 3).tolist()
                    entry["reconfiguration_at_home_rad"] = round(float(lp.maxdiff(Qr_back[-1], q_home)), 3)
            A.append(entry)
            print(f"V{i} A branch {b}: q {np.round(qb, 2).tolist()} transport "
                  f"{'PASS' if fail is None else 'FAIL @' + str(fail['sample'])}"
                  + (f", reach {'PASS' if entry.get('reach_same_branch') == 'PASS' else 'FAIL'}, "
                     f"arm proxy {entry['arm_proxy']}, collision-free {entry['gripper_part_collision']['collision_free']}, "
                     f"reconfig at HOME {entry.get('reconfiguration_at_home_rad')} rad" if fail is None else ""), flush=True)
        res["A_arm_posture"] = {"distinct_ik_solutions_at_lift_end": len(sols), "branches": A}

        # ---------------- B: height-limit coupling term (current branch)
        q_grip = R.ik_path(q_home, Ts_r, calib, "REACH")[-1]
        q_lift = uk.ik_local(q_grip, T_lift, calib).tolist()
        _, Ts_free = R.rollout(trans, pt, qt, T_lift, T_start, a.tau)[:2]
        P_free = np.array([T[:3, 3] for T in Ts_free])
        B = []
        for cap in CAPS_MM:
            t_c, Ts_c, info = capped_rollout(trans, pt, qt, T_lift, T_start, a.tau, cap / 1000)
            P_c = np.array([T[:3, 3] for T in Ts_c])
            n = min(len(P_c), len(P_free))
            dev_xy = float(np.linalg.norm(P_c[:n, :2] - P_free[:n, :2], axis=1).max() * 1000)
            t_c, Ts_c = R.resample(t_c, Ts_c)
            Qc, fail = walk(q_lift, Ts_c, calib)
            e = {"z_cap_mm": cap, **info, "max_xy_deviation_from_uncapped_mm": round(dev_xy, 1),
                 "duration_s": float(t_c[-1]), "ik": "PASS" if fail is None else fail}
            if fail is None:
                e["gripper_part_collision"] = held_check(Qc, "TRANSPORT")
                e["arm_proxy"] = arm_proxy(Qc, calib, fix_top)
                e["peak_joint_speed_rad_s"] = round(R.speeds(np.asarray(t_c), Qc)[0], 3)
                e["end_tcp_err_mm"] = float(np.linalg.norm(uk.tcp(Qc[-1], calib)[:3, 3] - T_start[:3, 3]) * 1000)
                e["PASS"] = bool(e["gripper_part_collision"]["collision_free"] and e["arm_proxy"]["ok"])
            else:
                e["PASS"] = False
            B.append(e)
            print(f"V{i} B cap {cap} mm: max z {info['max_tcp_z_mm']:.0f} mm, xy dev {dev_xy:.1f} mm, IK "
                  f"{'PASS' if fail is None else 'FAIL @' + str(fail['sample'])}"
                  + (f", collision-free {e['gripper_part_collision']['collision_free']} "
                     f"(min {e['gripper_part_collision']['min_clearance_mm']} mm {e['gripper_part_collision']['closest_pair']}), "
                     f"arm proxy {e['arm_proxy']['min_elbow_wrist_z_above_fixture_mm']} mm, peak {e['peak_joint_speed_rad_s']} rad/s"
                     if fail is None else ""), flush=True)
        res["B_height_coupling"] = B
        out["variants"][f"V{i}"] = res
        json.dump(out, open(os.path.join(a.out, "v45_fix_summary.json"), "w"), indent=1, default=float)
    print("->", a.out)


if __name__ == "__main__":
    main()
