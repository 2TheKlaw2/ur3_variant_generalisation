#!/usr/bin/env python3
"""Plan RELEASE -> RETREAT -> HOME after the Stage 9 L2 assembly (offline, kinematic simulation only).

  python3 scripts/phase10_full_sequence/plan_release_retreat.py --rg2-mesh-dir <...>

Same steps as the physical V1 sequence (scripts/phase5_integration/run_v1_physical_auto.py):
  RELEASE  RG2 opens 8.2 -> 40 mm (the part stays seated in its fitting); checked at 17 widths:
           the fingers must clear the part by 40 mm and touch nothing else while opening
  RETREAT  +30 mm straight up in base Z, 3.0 s min-jerk, gripper open
  HOME     joint-space min-jerk to the taught HOME (peak <= 0.3 rad/s, as the Stage 9 APPROACH)
The assembled part is placed at its final Stage 9 pose and treated as a scene object.
Checked: RG2 links vs every scene object (incl. the assembled part) at every sample; for RELEASE
at 12 finger widths. Arm links: MoveIt on fake hardware (offline: elbow/wrist height proxy).
Writes <stage10 dir>/V<i>/release_retreat.csv (stage, t_s, joints, gripper_width_m).
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import retarget_reach_transport as R  # noqa: E402
import v45_fix_experiments as X  # noqa: E402

np = R.np
uk, lp, sm = R.uk, R.lp, R.sm
HOME = [-0.675926510488, -1.34777768076, -0.527696251869, -2.8273965321, 1.55948805809, -3.87329799334]
RETREAT_M, RETREAT_S = 0.030, 3.0
HOME_PEAK_RAD_S = 0.3


def home_path(q_from):
    d = lp.maxdiff(q_from, HOME)
    T = max(3.0, 1.875 * d / HOME_PEAK_RAD_S)
    t, frac = lp.min_jerk_grid(T, lp.SAMPLE_DT_S)
    a, b = np.asarray(q_from), np.asarray(HOME)
    return t, [(a + (b - a) * s).tolist() for s in frac]


def open_check(col, q, calib, widths, part):
    """Fingers opening around the seated part: contact with the part itself is the grasp until the
    fingers clear it (reported as the release width); any other contact (fitting, plate, ...) fails."""
    best, pair, hit, clear_w = 1e9, None, None, None
    rg2_base, _ = uk.ur_fk(q, calib)
    others = {(f, part) for f in ("base_link", "l_outer_knuckle", "l_inner_finger", "l_inner_knuckle",
                                  "r_outer_knuckle", "r_inner_finger", "r_inner_knuckle")}
    for w in widths:
        g = col.gripper(rg2_base, w)
        d, p, hits = col.check(g, others)                    # everything except the held part
        if d < best:
            best, pair = d, p
        if hits and hit is None:
            hit = {"width_mm": round(w * 1000, 1), "contacts": hits}
        if clear_w is None:
            only_part = {k: v for k, v in g.items()}
            dp, _, hp = col.check(only_part, {(f, s_) for f in g for s_ in col.static if s_ != part})
            if not hp and dp > 0:
                clear_w = w
    return {"min_clearance_other_mm": round(best, 3), "closest_pair": pair, "first_collision": hit,
            "fingers_clear_part_at_width_mm": None if clear_w is None else round(clear_w * 1000, 1),
            "collision_free": hit is None and clear_w is not None}


def path_check(col, Q, calib, width, skip, stride=1):
    best, pair, hit = 1e9, None, None
    idx = list(range(0, len(Q), stride)) + [len(Q) - 1]
    for k in idx:
        rg2_base, _ = uk.ur_fk(Q[k], calib)
        d, p, hits = col.check(col.gripper(rg2_base, width), skip)
        if d < best:
            best, pair = d, p
        if hits and hit is None:
            hit = {"sample": k, "contacts": hits}
    return {"samples_checked": len(idx), "min_clearance_mm": round(best, 3), "closest_pair": pair,
            "first_collision": hit, "collision_free": hit is None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rg2-mesh-dir", required=True)
    ap.add_argument("--variants", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--stage9", default=os.path.join(sm.PKG, "results/sim_reference/stage9_offline_20261030_prongs"))
    ap.add_argument("--level", default="L2")
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/sim_reference/stage10_retarget_20261030_tau2"))
    a = ap.parse_args()
    import yaml
    calib = yaml.safe_load(open(Path(sm.PKG) / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects()
    G, _ = R.rvo.grasps(objs, calib)
    col = R.Collider(objs, a.rg2_mesh_dir)
    fix_top = objs["fixture_plate"]["T"][2, 3] + 0.0125 / 2
    widths = list(np.linspace(sm.RG2_GRIP_M, sm.RG2_OPEN_M, 17))     # 2 mm steps
    summary = {"type": "KINEMATIC SIMULATION (offline) - not a physical result",
               "source_stage9": a.stage9, "level": a.level, "variants": {}}
    for i in a.variants:
        rows = list(csv.DictReader(open(Path(a.stage9) / f"V{i}_{a.level}" / "trajectory.csv")))
        rows = [r for r in rows if r.get("feasible", "True") != "False"]
        q_end = [float(rows[-1][j]) for j in lp.JOINTS]
        part = f"pickup_V{i}"
        T_part = uk.tcp(q_end, calib) @ G[i]                        # assembled pose
        col.static[part][0].setTransform(col._tf(T_part))           # the part is now in its fitting
        skip = set()                                                  # check everything, incl. the assembled part
        res = {"final_q": q_end}
        # RELEASE (fingers open around the seated part)
        res["RELEASE"] = open_check(col, q_end, calib, widths, part)
        # RETREAT
        t_r, frac = lp.min_jerk_grid(RETREAT_S, lp.SAMPLE_DT_S)
        T0 = uk.tcp(q_end, calib); Ts = []
        for s in frac:
            T = T0.copy(); T[2, 3] += RETREAT_M * s; Ts.append(T)
        Q_r = R.ik_path(q_end, Ts, calib, "RETREAT")
        res["RETREAT"] = path_check(col, Q_r, calib, sm.RG2_OPEN_M, skip)
        # HOME
        t_h, Q_h = home_path(Q_r[-1])
        res["HOME"] = path_check(col, Q_h, calib, sm.RG2_OPEN_M, skip, stride=2)
        res["HOME"]["duration_s"] = float(t_h[-1])
        res["arm_proxy"] = X.arm_proxy(Q_r + Q_h, calib, fix_top)
        res["joint_limits"] = X.joint_limits(Q_r + Q_h)
        res["peak_joint_speed_rad_s"] = {"RETREAT": round(R.speeds(np.asarray(t_r), Q_r)[0], 3),
                                         "HOME": round(R.speeds(np.asarray(t_h), Q_h)[0], 3)}
        res["PASS"] = bool(res["RELEASE"]["collision_free"] and res["RETREAT"]["collision_free"]
                           and res["HOME"]["collision_free"] and res["arm_proxy"]["ok"] and res["joint_limits"]["ok"])
        col.static[part][0].setTransform(col._tf(objs[part]["T"]))   # back to its pocket for the next variant
        d = Path(a.out) / f"V{i}"; d.mkdir(parents=True, exist_ok=True)
        with open(d / "release_retreat.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["stage", "t_s"] + list(lp.JOINTS) + ["gripper_width_m"])
            w.writerow(["RELEASE", "0.000"] + [f"{x:.9f}" for x in q_end] + [f"{sm.RG2_OPEN_M:.4f}"])
            for st, t, Q in (("RETREAT", t_r, Q_r), ("HOME", t_h, Q_h)):
                for ti, q in zip(t, Q):
                    w.writerow([st, f"{ti:.3f}"] + [f"{x:.9f}" for x in q] + [f"{sm.RG2_OPEN_M:.4f}"])
        summary["variants"][f"V{i}"] = res
        print(f"V{i}: {'PASS' if res['PASS'] else 'FAIL'}  RELEASE fingers clear the part at "
              f"{res['RELEASE']['fingers_clear_part_at_width_mm']} mm, min to anything else {res['RELEASE']['min_clearance_other_mm']} mm "
              f"({res['RELEASE']['closest_pair']})  RETREAT min {res['RETREAT']['min_clearance_mm']} mm  "
              f"HOME {t_h[-1]:.1f}s min {res['HOME']['min_clearance_mm']} mm  arm {res['arm_proxy']['min_elbow_wrist_z_above_fixture_mm']} mm  "
              + "  ".join(f"{k}: {res[k]['first_collision']}" for k in ("RELEASE", "RETREAT", "HOME") if res[k]["first_collision"]),
              flush=True)
    json.dump(summary, open(Path(a.out) / "release_retreat_summary.json", "w"), indent=1, default=float)
    print("->", a.out)


if __name__ == "__main__":
    main()
