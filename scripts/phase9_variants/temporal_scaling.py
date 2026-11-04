#!/usr/bin/env python3
"""CAD-derived TEMPORAL scaling of the assembly (INSERT, ROTATE): offline study + the rule the runners use.

Rule (per stage, per variant i):
    s_i  = length of the path a tab tip travels through the fitting corridor (CAD-conditioned L2 path):
           sum over samples of sqrt(dz^2 + (r_tab * dyaw)^2), r_tab = 16.03 mm (tab reach, CAD_CHECKS)
    tau_i = max(1, s_i / s_V1)
The stage's demonstrated timing is stretched uniformly by tau_i (a DMP time-scaling: the path is unchanged,
only time stamps are multiplied). The tab then moves through every fitting at (at most) the speed it moved
through the V1 fitting in the demonstration; a stage is never made shorter than demonstrated.
'fixed' = the demonstrated duration for every variant (Stage 9 as planned until 4 Nov 2026).

  python3 scripts/phase9_variants/temporal_scaling.py --stage9 results/simulation/stage9_<session> [--out DIR]

Kinematic simulation only: timing cannot change collision/seating outcomes there (time is not in the
geometric check); what changes is the speed through the fitting, the joint speeds and how much the
physical 0.14 rad/s guard must intervene.
"""
import argparse
import csv
import json
import math
import os

TAB_R_MM = 16.03
STAGES = ("INSERT", "ROTATE")
PHYS_GUARD_RAD_S = 0.14
PHYS_MIN_SLOWDOWN = {"INSERT": 1.5}      # run_full_cycle_physical.MIN_SLOWDOWN for these stages
JOINTS = [f"{x}_joint" for x in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]


def tab_path_mm(z_mm, yaw_deg):
    """Cumulative tab-tip path length [mm] along a stage (z: part bottom/lowest z, yaw: part yaw in the fitting)."""
    s = [0.0]
    for k in range(1, len(z_mm)):
        dz = z_mm[k] - z_mm[k - 1]; da = math.radians(yaw_deg[k] - yaw_deg[k - 1]) * TAB_R_MM
        s.append(s[-1] + math.hypot(dz, da))
    return s


def tau_for(s_variant_mm, s_v1_mm):
    return max(1.0, s_variant_mm / s_v1_mm) if s_v1_mm > 1e-9 else 1.0


def taus_from_rows(rows_i, rows_v1):
    """{stage: tau} from Stage 9 result rows (dicts with stage, lowest_z_mm or bottom_z_mm, yaw_deg, feasible)."""
    def length(rows, st):
        r = [x for x in rows if x["stage"] == st and x.get("feasible", True)]
        z = [x.get("lowest_z_mm", x.get("bottom_z_mm")) for x in r]
        return tab_path_mm(z, [x["yaw_deg"] for x in r])[-1] if len(r) > 1 else 0.0
    return {st: tau_for(length(rows_i, st), length(rows_v1, st)) for st in STAGES}


def read_traj(path):
    out = {st: [] for st in STAGES}
    for r in csv.DictReader(open(path)):
        if r["stage"] in out and r["feasible"] == "True":
            out[r["stage"]].append({"t": float(r["t_s"]), "q": [float(r[j]) for j in JOINTS],
                                    "z": float(r["v1_lowest_z_mm"]), "yaw": float(r["v1_yaw_deg"])})
    return out


def stage_metrics(seg, tau):
    t = [x["t"] * tau for x in seg]; s = tab_path_mm([x["z"] for x in seg], [x["yaw"] for x in seg])
    v_tab, v_q = [], []
    for k in range(1, len(seg)):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            continue
        v_tab.append((s[k] - s[k - 1]) / dt)
        v_q.append(max(abs(a - b) for a, b in zip(seg[k]["q"], seg[k - 1]["q"])) / dt)
    dur = t[-1] - t[0]
    return {"duration_s": round(dur, 2), "tab_path_mm": round(s[-1], 2), "peak_tab_speed_mm_s": round(max(v_tab), 3),
            "mean_tab_speed_mm_s": round(s[-1] / dur, 3), "peak_joint_speed_rad_s": round(max(v_q), 4)}


def physical_duration(m, stage):
    """Duration on the robot: the guard's uniform slowdown, at least MIN_SLOWDOWN (as the physical runner)."""
    f = max(1.0, m["peak_joint_speed_rad_s"] / PHYS_GUARD_RAD_S)
    return f, m["duration_s"] * max(f, PHYS_MIN_SLOWDOWN.get(stage, 1.0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage9", required=True, help="Stage 9 offline results folder (with V<i>_L2/trajectory.csv)")
    ap.add_argument("--level", default="L2")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    data = {i: read_traj(os.path.join(a.stage9, f"V{i}_{a.level}", "trajectory.csv")) for i in range(1, 6)}
    ref = {st: tab_path_mm([x["z"] for x in data[1][st]], [x["yaw"] for x in data[1][st]])[-1] for st in STAGES}
    res = {"type": "KINEMATIC SIMULATION (offline) - temporal scaling study, not a physical result",
           "rule": "tau = max(1, s_i / s_V1) per stage; s = tab-tip path length through the fitting (r_tab 16.03 mm)",
           "stage9": a.stage9, "level": a.level, "v1_tab_path_mm": {k: round(v, 2) for k, v in ref.items()}, "variants": {}}
    print(f"{'':4s} {'stage':7s} {'path mm':>8s} {'tau':>5s} | {'fixed: dur s':>12s} {'peak tab':>9s} {'peak q':>7s} {'robot s':>8s}"
          f" | {'scaled: dur s':>13s} {'peak tab':>9s} {'peak q':>7s} {'robot s':>8s}")
    for i in range(1, 6):
        v = {}
        for st in STAGES:
            seg = data[i][st]
            s_i = tab_path_mm([x["z"] for x in seg], [x["yaw"] for x in seg])[-1]
            tau = tau_for(s_i, ref[st])
            fx, sc = stage_metrics(seg, 1.0), stage_metrics(seg, tau)
            gfx, pfx = physical_duration(fx, st); gsc, psc = physical_duration(sc, st)
            v[st] = {"tau": round(tau, 3), "fixed": {**fx, "guard_slowdown": round(gfx, 2), "robot_duration_s": round(pfx, 1)},
                     "scaled": {**sc, "guard_slowdown": round(gsc, 2), "robot_duration_s": round(psc, 1)}}
            print(f"V{i}   {st:7s} {s_i:8.2f} {tau:5.2f} | {fx['duration_s']:12.1f} {fx['peak_tab_speed_mm_s']:9.3f} "
                  f"{fx['peak_joint_speed_rad_s']:7.3f} {pfx:8.1f} | {sc['duration_s']:13.1f} {sc['peak_tab_speed_mm_s']:9.3f} "
                  f"{sc['peak_joint_speed_rad_s']:7.3f} {psc:8.1f}")
        res["variants"][f"V{i}"] = v
    print("\npeak tab = peak tab-tip speed through the fitting [mm/s]; peak q = peak joint speed [rad/s]; robot s = duration"
          " on the robot after the 0.14 rad/s guard and MIN_SLOWDOWN")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        json.dump(res, open(os.path.join(a.out, "temporal_scaling_summary.json"), "w"), indent=1)
        print("->", os.path.join(a.out, "temporal_scaling_summary.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
