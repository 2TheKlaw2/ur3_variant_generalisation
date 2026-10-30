#!/usr/bin/env python3
"""Retarget the single V1 REACH and TRANSPORT demonstrations to V1-V5 with Victor's CDMPs (offline).

  python3 scripts/phase10_full_sequence/retarget_reach_transport.py \
      --library ~/ros2_ws/src/rs2_dmp_demo --rg2-mesh-dir <OnRobot_ROS2_Description>/meshes/rg2/collision

Per variant i (one CDMP trained per skill, on the V1 demo only; weights reused for every variant):
  REACH      CDMP(v1_reach_001_motion) from the demo start (HOME) to the grip TCP of V<i>
             = T(pickup_V<i>) . inv(grasp_i)                              gripper open 40 mm
  GRIP       close to 8.2 mm, attach V<i> at its grasp transform
  LIFT       30 mm straight up, 3.0 s min-jerk (Stage 6, unchanged)
  TRANSPORT  CDMP(v1_transport_001_motion) from the lift end to the Stage 9 start of V<i>
             (part centred/upright, 0.70 mm above the fitting top - where the V1 demo ends)
  -> hands over to Stage 9 INSERT/ROTATE (run_variants_offline / run_variant_insert_sim).
Goals use the CDMP goal-change (spatial scaling) property; timing = demo timing x tau.
Every sample: calibrated-UR3e IK seeded from the previous sample (branch-jump check), joint
speed, and FCL collision of the RG2 (+ attached part) against every scene object.
Arm links above the RG2 are NOT checked here (MoveIt does that on fake hardware).
Kinematic simulation only; not a physical result. No ROS, no robot.
"""
import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for d in ("phase9_variants", "phase8_insert_sim", "phase7_transport_sim", "phase6_pickup_sim", "scene", "phase2_reach"):
    sys.path.insert(0, str(HERE.parent / d))
import fcl  # noqa: E402
import lift_profile as lp  # noqa: E402
import run_variants_offline as rvo  # noqa: E402
import scene_model as sm  # noqa: E402
import trimesh  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import variant_core as vc  # noqa: E402
import yaml  # noqa: E402
from generate_phase2_cdmp import load_clean_demo, quat_angle_deg, wxyz_to_xyzw, xyzw_to_wxyz  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

np = sm.np
REACH_CSV = "demonstrations/processed/v1_reach_001_motion.csv"
TRANSPORT_CSV = "demonstrations/processed/v1_transport_001_motion.csv"
CDMP_PARAMS = {"alpha_s": 7.0, "k_gain": 100.0, "rbfs_pSec": 4.0}       # as Phase 2/3
LIFT_M, LIFT_S = 0.030, 3.0
ALIGN_S = 1.0                # final straight min-jerk correction from the CDMP end to the exact pose (as V1 ALIGN)
MAX_STEP_RAD = 0.02          # IK branch-jump guard per 20 ms sample
PHYS_GUARD_RAD_S = 0.14      # physical pilot joint-speed guard (reported, Stage 6/7)
CONTACT_MM = 0.0
POCKET_RELEASE_MM = 2.5      # part<->pickup_plate ignored until the part is this far above its start (pocket 2 mm)


# ------------------------------------------------------------------ CDMP
class quiet:
    """The library prints its parameters on every call; keep the console readable."""
    def __enter__(self):
        self._o = sys.stdout; sys.stdout = open(os.devnull, "w")
    def __exit__(self, *e):
        sys.stdout.close(); sys.stdout = self._o


def train(lib, csv_path):
    sys.path.insert(0, str(lib))
    from cdmp.src.cdmps import CartesianDMPs
    t, p, q = load_clean_demo(csv_path)
    c = CartesianDMPs()
    with quiet():
        c.load_demo(filename=csv_path.name, dem_time=t, dem_pos=p, dem_quat=q)
        c.learn_cdmp(**CDMP_PARAMS)
    return c, t, p, q


def rollout(c, demo_p, demo_q, start_T, goal_T, tau):
    """Victor's CDMP: absolute start, goal = demo goal moved by a relative pose (as Phase 2).
    Settling continues the canonical phase (no forcing), bounded to 1.5 tau, as Phase 2."""
    start_p, start_q = start_T[:3, 3], xyzw_to_wxyz(Rotation.from_matrix(start_T[:3, :3]).as_quat())
    if np.dot(start_q, demo_q[0]) < 0:
        start_q = -start_q
    gp, gq = goal_T[:3, 3], Rotation.from_matrix(goal_T[:3, :3])
    src = Rotation.from_quat(wxyz_to_xyzw(demo_q[-1]))
    # library: rep goal = demo goal + rel position, rel quaternion * demo goal quaternion
    q_rel = (gq * src.inv()).as_quat()
    with quiet():
        c.init_reproduction(tau_scaler=tau, rep_abspose_start=np.r_[start_p, start_q],
                            rep_relpose_goal=np.r_[gp - demo_p[-1], xyzw_to_wxyz(q_rel)])
    for s in c.rep_cs.s_track:
        c.rollout_step(curr_s_step=float(s))

    def err():
        pe = np.asarray(c.rep_pos[-1], float); qe = np.asarray(c.rep_quat[-1], float)
        return (float(np.linalg.norm(pe - gp) * 1000),
                quat_angle_deg(wxyz_to_xyzw(qe / np.linalg.norm(qe)), gq.as_quat()))
    nominal = err()
    phase, decay = float(c.rep_cs.s_track[-1]), math.exp(-c.alpha_s * c.dt / c.rep_tau)
    n = 0
    while n < int(math.ceil(1.5 * c.rep_tau / c.dt)) and (err()[0] > 2.0 or err()[1] > 0.5):   # Phase 2 bounds
        phase *= decay; c.rollout_step(curr_s_step=phase); n += 1
    t = np.asarray(c.rep_time, float).reshape(-1)
    P = np.asarray(c.rep_pos, float); Q = np.asarray(c.rep_quat, float)
    Q = np.column_stack([Q[:, 1], Q[:, 2], Q[:, 3], Q[:, 0]]); Q /= np.linalg.norm(Q, axis=1)[:, None]
    Ts = []
    for pi, qi in zip(P, Q):
        T = np.eye(4); T[:3, :3] = Rotation.from_quat(qi).as_matrix(); T[:3, 3] = pi; Ts.append(T)
    return t - t[0], Ts, {"nominal_goal_err_mm_deg": nominal, "final_goal_err_mm_deg": err(), "settle_steps": n,
                          "start_err_mm": float(np.linalg.norm(P[0] - start_p) * 1000)}


# ------------------------------------------------------------------ resample to 20 ms + IK
def resample(t, Ts, dt=lp.SAMPLE_DT_S):
    tt = np.arange(0.0, t[-1] + 1e-9, dt)
    if tt[-1] < t[-1] - 1e-9:
        tt = np.r_[tt, t[-1]]
    P = np.array([T[:3, 3] for T in Ts])
    from scipy.spatial.transform import Slerp
    S = Slerp(t, Rotation.from_matrix([T[:3, :3] for T in Ts]))
    out = []
    for k, ti in enumerate(tt):
        T = np.eye(4); T[:3, :3] = S([ti]).as_matrix()[0]
        T[:3, 3] = [np.interp(ti, t, P[:, j]) for j in range(3)]; out.append(T)
    return tt, out


def ik_path(q0, Ts, calib, name):
    import transport_profile as tp
    q, out, T_prev = list(q0), [], uk.tcp(q0, calib)
    for k, T in enumerate(Ts):
        try:
            q = uk.ik_local(q, T, calib).tolist()
        except RuntimeError:
            # DLS did not converge in one step: walk to the same target in 10 sub-steps (same branch)
            for j in range(1, 11):
                q = uk.ik_local(q, tp.interp_pose(T_prev, T, j / 10), calib, iters=200).tolist()
        T_prev = T
        if out and lp.maxdiff(out[-1], q) > MAX_STEP_RAD * 5:
            raise RuntimeError(f"{name}: IK branch jump at sample {k}")
        out.append(q)
    return out


def align(q0, T_goal, calib, name):
    import transport_profile as tp
    t, frac = lp.min_jerk_grid(ALIGN_S, lp.SAMPLE_DT_S)
    T0 = uk.tcp(q0, calib)
    return t, ik_path(q0, [tp.interp_pose(T0, T_goal, s_) for s_ in frac], calib, name)


def speeds(t, Q):
    Q = np.asarray(Q); dt = np.diff(t)[:, None]
    v = np.abs(np.diff(Q, axis=0)) / dt
    return float(v.max()), float(np.abs(np.diff(Q, axis=0)).max())


# ------------------------------------------------------------------ collision
class Collider:
    def __init__(self, objs, mesh_dir):
        self.static = {}
        for k, o in objs.items():
            m = trimesh.load(o["mesh"], force="mesh"); m.apply_scale(0.001)
            self.static[k] = (self._obj(m), o["T"])
            self.static[k][0].setTransform(self._tf(o["T"]))
        self.mesh_dir = mesh_dir
        self.parts = {k: self._obj(trimesh.load(objs[k]["mesh"], force="mesh").apply_scale(0.001))
                      for k in objs if k.startswith("pickup_V")}
        self.rg2 = {}

    @staticmethod
    def _obj(m):
        g = fcl.BVHModel(); g.beginModel(len(m.vertices), len(m.faces)); g.addSubModel(m.vertices, m.faces); g.endModel()
        return fcl.CollisionObject(g, fcl.Transform())

    @staticmethod
    def _tf(T):
        return fcl.Transform(T[:3, :3], T[:3, 3])

    def gripper(self, rg2_base, width):
        import validate_scene as vs
        out = {}
        for name, M in vs.rg2_links(rg2_base, width).items():
            f = name.split("_", 1)[1] if name != "base_link" else "base_link"
            if f not in self.rg2:
                self.rg2[f] = trimesh.load(os.path.join(self.mesh_dir, f + ".stl"), force="mesh")
            o = self._obj(self.rg2[f]); o.setTransform(self._tf(M)); out[name] = o
        return out

    def check(self, moving, ignore):
        """moving: {name: fcl obj}; ignore: set of (moving, static) pairs. -> (min dist mm, worst pair, contacts)."""
        best, pair, hits = 1e9, None, []
        for mn, mo in moving.items():
            for sn, (so, _) in self.static.items():
                if (mn, sn) in ignore or (mn, "*") in ignore:
                    continue
                c = fcl.CollisionResult(); fcl.collide(mo, so, fcl.CollisionRequest(), c)
                if c.is_collision:
                    hits.append(f"{mn}<->{sn}"); d = 0.0
                else:
                    r = fcl.DistanceResult(); fcl.distance(mo, so, fcl.DistanceRequest(), r); d = r.min_distance
                if d * 1000 < best:
                    best, pair = d * 1000, f"{mn}<->{sn}"
        return best, pair, hits


def check_stage(col, name, Q, calib, width, part=None, G=None, z0=None, stride=1):
    """Collision-check every `stride`-th sample (and the last). Returns summary dict."""
    worst, wpair, first_hit = 1e9, None, None
    idx = list(range(0, len(Q), stride)) + ([len(Q) - 1] if (len(Q) - 1) % stride else [])
    for k in idx:
        rg2_base, T_tcp = uk.ur_fk(Q[k], calib)
        mov = col.gripper(rg2_base, width)
        ign = set()
        if part:
            # the grasped part moves with the gripper: drop its static copy, add the moving one
            ign |= {(m, part) for m in list(mov) + [part]}
            po = col.parts[part]; T_part = T_tcp @ G; po.setTransform(col._tf(T_part)); mov[part] = po
            if T_part[2, 3] - z0 < POCKET_RELEASE_MM / 1000:
                ign.add((part, "pickup_plate"))                       # resting in its 2 mm pocket at pickup
        d, pair, hits = col.check(mov, ign)
        if d < worst:
            worst, wpair = d, pair
        if hits and first_hit is None:
            first_hit = {"sample": k, "contacts": hits}
    return {"samples_checked": len(idx), "min_clearance_mm": round(worst, 3), "closest_pair": wpair,
            "first_collision": first_hit, "collision_free": first_hit is None}


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", type=Path, default=Path.home() / "ros2_ws/src/rs2_dmp_demo")
    ap.add_argument("--rg2-mesh-dir", required=True)
    ap.add_argument("--variants", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--tau", type=float, default=None,
                    help="CDMP time scale for REACH and TRANSPORT (default: smallest, in 0.5 steps, within the 0.14 rad/s guard)")
    ap.add_argument("--stride", type=int, default=5, help="collision-check every Nth 20 ms sample (+ the last)")
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/simulation/stage10_retarget"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    root = Path(sm.PKG)
    calib = yaml.safe_load(open(root / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects()
    G, gsrc = rvo.grasps(objs, calib)
    q1, T_tcp1, z1, yaw0 = rvo.v1_start(objs, calib, G[1])
    col = Collider(objs, a.rg2_mesh_dir)

    reach, tr, pr, qr = train(a.library, root / REACH_CSV)
    trans, tt, pt, qt = train(a.library, root / TRANSPORT_CSV)
    _r0 = next(csv.DictReader(open(root / REACH_CSV)))
    q_home = [float(_r0[j]) for j in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
    summary = {"type": "KINEMATIC SIMULATION (offline) - not a physical result",
               "method": "Victor Hernandez-Moreno CartesianDMPs, trained once per skill on the V1 demo; goal change per variant",
               "cdmp_params": CDMP_PARAMS, "reach_demo": REACH_CSV, "transport_demo": TRANSPORT_CSV,
               "demo_durations_s": {"reach": float(tr[-1]), "transport": float(tt[-1])},
               "grasp_sources": gsrc, "layout": yaml.safe_load(open(root / "config/scene/scene_registration.yaml")).get("layout"),
               "collision_scope": "RG2 links (+ attached part) vs every scene object; arm links not checked offline",
               "variants": {}}
    spath = os.path.join(a.out, "stage10_summary.json")
    if os.path.exists(spath):                       # keep variants from earlier runs into the same folder
        summary["variants"] = json.load(open(spath)).get("variants", {})
    for i in a.variants:
        T_fit = objs[f"fitting_V{i}"]["T"]
        geom = vc.VariantGeometry(i, objs, T_fit)
        z_start = geom.top_mm + (z1 - 7.0)
        T_start_i = vc.part_pose(T_fit, z_start, yaw0) @ np.linalg.inv(G[i])      # Stage 9 start
        q_start_i = q1 if i == 1 else vc.ik_continuation(q1, T_tcp1, T_start_i, calib, steps=200)
        T_grip_i = objs[f"pickup_V{i}"]["T"] @ np.linalg.inv(G[i])
        T_home = uk.tcp(q_home, calib)
        def plan(tau_r, tau_t):
            t_r, Ts_r, info_r = rollout(reach, pr, qr, T_home, T_grip_i, tau_r)
            t_r, Ts_r = resample(t_r, Ts_r)
            Q_r = ik_path(q_home, Ts_r, calib, "REACH")
            t_ag, Q_ag = align(Q_r[-1], T_grip_i, calib, "ALIGN_GRIP")
            t_l, frac = lp.min_jerk_grid(LIFT_S, lp.SAMPLE_DT_S)
            Ts_l = []
            for s_ in frac:
                T = T_grip_i.copy(); T[2, 3] += LIFT_M * s_; Ts_l.append(T)
            Q_l = ik_path(Q_ag[-1], Ts_l, calib, "LIFT")
            t_t, Ts_t, info_t = rollout(trans, pt, qt, Ts_l[-1], T_start_i, tau_t)
            t_t, Ts_t = resample(t_t, Ts_t)
            Q_t = ik_path(Q_l[-1], Ts_t, calib, "TRANSPORT")
            t_ai, Q_ai = align(Q_t[-1], T_start_i, calib, "ALIGN_INSERT")
            st = [("REACH", t_r, Q_r), ("ALIGN_GRIP", t_ag, Q_ag), ("LIFT", t_l, Q_l),
                  ("TRANSPORT", t_t, Q_t), ("ALIGN_INSERT", t_ai, Q_ai)]
            v = {n: speeds(np.asarray(t), Q) for n, t, Q in st}
            return st, v, info_r, info_t, Ts_r, Ts_t

        if a.tau:
            tau_r = tau_t = a.tau
        else:   # temporal scaling: the smallest tau (0.5 steps) that keeps the CDMP stages within the guard
            _, v1_, *_ = plan(1.0, 1.0)
            tau_r = max(1.0, math.ceil(2 * v1_["REACH"][0] / PHYS_GUARD_RAD_S) / 2)
            tau_t = max(1.0, math.ceil(2 * v1_["TRANSPORT"][0] / PHYS_GUARD_RAD_S) / 2)
        while True:
            st, v, info_r, info_t, Ts_r, Ts_t = plan(tau_r, tau_t)
            if a.tau or (v["REACH"][0] <= PHYS_GUARD_RAD_S and v["TRANSPORT"][0] <= PHYS_GUARD_RAD_S) or max(tau_r, tau_t) >= 10:
                break
            tau_r += 0.5 * (v["REACH"][0] > PHYS_GUARD_RAD_S); tau_t += 0.5 * (v["TRANSPORT"][0] > PHYS_GUARD_RAD_S)
        S = {n: (t, Q) for n, t, Q in st}
        peak = max(x[0] for x in v.values())
        res = {"tau_reach": tau_r, "tau_transport": tau_t, "reach_cdmp": info_r, "transport_cdmp": info_t,
               "durations_s": {n: float(t[-1]) for n, t, _ in st},
               "peak_joint_speed_rad_s": {k: round(x[0], 4) for k, x in v.items()},
               "max_joint_step_rad": {k: round(x[1], 5) for k, x in v.items()},
               "within_physical_speed_guard": bool(peak <= PHYS_GUARD_RAD_S),
               "align_grip_correction_mm": float(np.linalg.norm(Ts_r[-1][:3, 3] - T_grip_i[:3, 3]) * 1000),
               "align_insert_correction_mm": float(np.linalg.norm(Ts_t[-1][:3, 3] - T_start_i[:3, 3]) * 1000),
               "handover_to_stage9_joint_gap_rad": float(lp.maxdiff(S["ALIGN_INSERT"][1][-1], q_start_i)),
               "transport_min_tcp_z_mm": float(min(T[2, 3] for T in Ts_t) * 1000)}
        part = f"pickup_V{i}"
        z0 = (objs[part]["T"])[2, 3]
        res["collision"] = {}
        for n, t, Q in st:
            held = n not in ("REACH", "ALIGN_GRIP")
            res["collision"][n] = check_stage(col, n, Q, calib, sm.RG2_GRIP_M if held else sm.RG2_OPEN_M,
                                              part if held else None, G[i], z0,
                                              stride=a.stride if n in ("REACH", "TRANSPORT") else 1)
        ok = all(c_["collision_free"] for c_ in res["collision"].values())
        res["PASS"] = bool(ok and res["handover_to_stage9_joint_gap_rad"] < 1e-3)
        summary["variants"][f"V{i}"] = res
        d = os.path.join(a.out, f"V{i}"); os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "joint_trajectory.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stage", "t_s"] + list(lp.JOINTS))
            for n, t, Q in st:
                for ti, q in zip(t, Q):
                    w.writerow([n, f"{ti:.3f}"] + [f"{x:.9f}" for x in q])
        c = res["collision"]
        D = res["durations_s"]
        print(f"V{i}: {'PASS' if res['PASS'] else 'FAIL'}  tau R/T {tau_r}/{tau_t}  REACH {D['REACH']:.1f}s  TRANSPORT {D['TRANSPORT']:.1f}s  "
              f"peak {peak:.3f} rad/s  ALIGN {res['align_grip_correction_mm']:.2f}/{res['align_insert_correction_mm']:.2f} mm  "
              f"hand-over gap {res['handover_to_stage9_joint_gap_rad']:.1e} rad  min clearance "
              + "/".join(f"{c[n]['min_clearance_mm']:.1f}" for n in c) + " mm  "
              + "  ".join(f"{k}: {v_['first_collision']}" for k, v_ in c.items() if v_["first_collision"]), flush=True)
        json.dump(summary, open(os.path.join(a.out, "stage10_summary.json"), "w"), indent=1, default=float)
    print("->", a.out)


if __name__ == "__main__":
    main()
