#!/usr/bin/env python3
"""Stage 9 on FAKE hardware (ROS_DOMAIN_ID=73): INSERT/ROTATE of V<i> under L0/L1/L2 in RViz.

  ./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh    # terminal 1
  ./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh   # terminal 2 (MoveIt + RViz)
  export ROS_DOMAIN_ID=73
  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 4 --level L2          # plan + preflight, no motion
  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 4 --level L2 --run    # type START
  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant all --level L1 --run  # V1..V5 in turn
  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 4 --level L2 --plan-only   # no ROS at all
  python3 scripts/phase9_variants/run_variant_insert_sim.py --variant 2 --level L2 --full --run   # whole sequence
  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset                         # afterwards

--full: replaces APPROACH/ATTACH/DESCEND with the Stage 10 CDMP-retargeted sequence
  REACH -> ALIGN_GRIP -> GRIP + ATTACH -> LIFT 30 mm -> TRANSPORT -> ALIGN_INSERT
  (joint trajectories from scripts/phase10_full_sequence/retarget_reach_transport.py; it ends exactly
  on the INSERT/ROTATE start state). V1-V5 (31 Oct reference). A plan that stops early (IK failure) is played up to that point.

Sequence per variant:
  HOME -> OPEN -> APPROACH (joint-space, min-jerk) to 40 mm above the Stage 9 start
  -> ATTACH V<i> at its grasp transform  (PICKUP AND TRANSPORT ARE NOT SIMULATED FOR V2-V5:
     the part is attached directly in the gripper above its fitting; for V1 use Stage 8)
  -> DESCEND 40 mm straight down to the Stage 9 start (part centred/upright, 0.70 mm above
     the fitting top = Stage 8 B2 after CORRECTION) -> INSERT -> ROTATE (Stage 9 plan).
The INSERT/ROTATE plan is computed by variant_core.run_condition, identical to the offline
results; only its feasible part is executed (stop at the first unacceptable contact).
APPROACH and DESCEND: every sample and joint-linear in-between state is collision-checked by
MoveIt. INSERT/ROTATE: V<i><->fitting_V<i> is allowed in MoveIt (judged by the CAD classifier);
all other collisions are checked by MoveIt on every executed sample.
Kinematic simulation only; not a physical result.
"""
import argparse
import math
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
for d in (HERE, HERE.parent / "phase8_insert_sim", HERE.parent / "phase7_transport_sim", HERE.parent / "phase6_pickup_sim",
          HERE.parent / "scene"):
    sys.path.insert(0, str(d))
import lift_profile as lp  # noqa: E402
import plan_cache  # noqa: E402
import planning_scene_tools as pst  # noqa: E402
import preview_stage8_offline as out8  # noqa: E402
import run_variants_offline as rvo  # noqa: E402
import scene_model as sm  # noqa: E402
import sim_node  # noqa: E402
import stage8_core as s8  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import variant_core as vc  # noqa: E402
import yaml  # noqa: E402

np = sm.np
ROOT = Path(sm.PKG)
JOINTS = lp.JOINTS
HOME = [-0.675926510488, -1.34777768076, -0.527696251869, -2.8273965321, 1.55948805809, -3.87329799334]  # sim_reach_action.py
ABOVE_M = 0.040                 # approach height above the Stage 9 start
DESCEND_S = 3.0                 # NEW sim timing, min-jerk
APPROACH_PEAK_RAD_S = 0.3       # min-jerk duration chosen so the peak joint speed is <= this


# ------------------------------------------------------------------ planning (no ROS)
def plan(i, level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, features_dir):
    T_fit = objs[f"fitting_V{i}"]["T"]
    geom = vc.VariantGeometry(i, objs, T_fit)
    z_start = geom.top_mm + (z1 - 7.0)
    rows = vc.load_corridor(os.path.join(features_dir, f"V{i}_corridor.csv"))
    rows1 = vc.load_corridor(os.path.join(features_dir, "V1_corridor.csv"))
    v1_lock = vc.seat_target(rows1, prof["Y_demo"], 0.0)["lock_end_deg"]
    feat = vc.seat_target(rows, prof["Y_demo"], v1_lock)
    T_tcp_start = vc.part_pose(T_fit, z_start, yaw0) @ np.linalg.inv(G[i])
    q_start = q1 if i == 1 else vc.ik_continuation(q1, T_tcp1, T_tcp_start, calib, steps=200)
    ctx = {"calib": calib, "T_fit": T_fit, "G": G[i], "z_start": z_start, "q_start": q_start, "yaw0": yaw0}
    res = vc.run_condition(i, level, ctx, geom, rows, feat, prof)
    res["cad_features"] = feat
    # DESCEND (planned upward from q_start so it ends exactly on the plan's start state)
    t_d, frac = lp.min_jerk_grid(DESCEND_S, lp.SAMPLE_DT_S)
    # Solve upward from the start (heights ABOVE*frac[k], frac 0 -> 1) so every sample stays on
    # the start's IK branch. Min-jerk is symmetric (frac(T - t) = 1 - frac(t)), so the reversed
    # list is the straight descent sampled at the same 20 ms timestamps.
    seed, up = list(q_start), []
    for s in frac:
        T = T_tcp_start.copy(); T[2, 3] = T_tcp_start[2, 3] + ABOVE_M * s
        seed = uk.ik_local(seed, T, calib).tolist(); up.append(seed)
    up[0] = list(q_start)
    q_desc = up[::-1]
    if max(lp.maxdiff(a, b) for a, b in zip(q_desc, q_desc[1:])) > 0.02:
        raise RuntimeError("DESCEND: IK branch jump")
    return {"res": res, "geom": geom, "ctx": ctx, "feat": feat, "t_desc": t_d.tolist(), "q_desc": q_desc,
            "T_tcp_start": T_tcp_start}


PLAN_FILES = ["scripts/phase9_variants/variant_core.py",
              "scripts/phase8_insert_sim/stage8_core.py", "scripts/scene/ur_kinematics.py",
              "config/scene/scene_registration.yaml", "config/ur3e_lab_calibration.yaml",
              "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv", "config/trajectories/v1/v1_align_to_insertion_joint_plan_001.csv",
              s8.INSERT_CSV, s8.ROTATE_CSV]


def plan_cached(i, level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, features_dir):
    files = PLAN_FILES + [objs[f"pickup_V{i}"]["mesh"], objs[f"fitting_V{i}"]["mesh"],
                          os.path.join(features_dir, f"V{i}_corridor.csv"), os.path.join(features_dir, "V1_corridor.csv")]
    files += [f"config/saved_poses/v1_pickup_V{k}_geometry.yaml" for k in range(2, 6)]
    # key on the planning code itself (not the whole runner), so UI/playback edits keep the cache
    import inspect
    k = plan_cache.key(f"stage9_V{i}_{level}", {"plan_src": inspect.getsource(plan), "ver": 2}, files)
    p = plan_cache.get(k)
    if p is not None:
        print(f"  V{i} {level}: cached plan", flush=True)
        p["geom"] = vc.VariantGeometry(i, objs, p["ctx"]["T_fit"])
        return p
    p = plan(i, level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, features_dir)
    plan_cache.put(k, {kk: v for kk, v in p.items() if kk != "geom"})
    return p


def _plan_worker(a):
    plan_cached(*a)          # result goes to the cache; the geometry object is not picklable
    return a[0]


def prewarm(variants, level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, features_dir):
    """Plan every uncached variant in parallel (one process each), with progress, before the run."""
    import time
    from concurrent.futures import ProcessPoolExecutor, as_completed
    todo = []
    for i in variants:
        files = PLAN_FILES + [objs[f"pickup_V{i}"]["mesh"], objs[f"fitting_V{i}"]["mesh"],
                              os.path.join(features_dir, f"V{i}_corridor.csv"), os.path.join(features_dir, "V1_corridor.csv")]
        files += [f"config/saved_poses/v1_pickup_V{k}_geometry.yaml" for k in range(2, 6)]
        import inspect
        k = plan_cache.key(f"stage9_V{i}_{level}", {"plan_src": inspect.getsource(plan), "ver": 2}, files)
        if plan_cache.get(k) is None:
            todo.append(i)
    if not todo:
        return
    n = min(len(todo), max(1, (os.cpu_count() or 2) - 1))
    print(f"  planning V{', V'.join(map(str, todo))} in parallel ({n} processes, ~1 min; once per geometry change)...",
          flush=True)
    t0 = time.monotonic()
    args_ = [(i, level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, features_dir) for i in todo]
    with ProcessPoolExecutor(n) as ex:
        futs = [ex.submit(_plan_worker, a) for a in args_]
        for f in as_completed(futs):
            print(f"  V{f.result()} planned ({time.monotonic() - t0:.0f} s)", flush=True)


def unwrap_to(q_ref, paths):
    """Shift wrist_3 of every planned joint vector by the same 2*pi multiple nearest q_ref."""
    k = round((q_ref[5] - paths[0][0][5]) / (2 * math.pi))
    if k:
        for p in paths:
            for q in p:
                q[5] += 2 * math.pi * k
    return k


def approach_path(q_from, q_to):
    d = lp.maxdiff(q_from, q_to)
    T = max(3.0, 1.875 * d / APPROACH_PEAK_RAD_S)
    t, frac = lp.min_jerk_grid(T, lp.SAMPLE_DT_S)
    a, b = np.asarray(q_from), np.asarray(q_to)
    return t.tolist(), [(a + (b - a) * s).tolist() for s in frac]


def print_plan(i, level, p):
    r = p["res"]; fu = r["first_unacceptable"]; f = r["final"]
    print(f"\nPLAN V{i} {level} (offline classifier, identical to Stage 9 results):")
    print(f"  CAD seat depth {r['cad_seat_depth_mm']:.2f} mm, CAD seat yaw {r['cad_seat_yaw_deg']:.2f} deg "
          f"({'CW' if r['cad_seat_yaw_deg'] < 0 else 'CCW'})")
    print(f"  INSERT {r['stages']['INSERT']['samples_feasible']}/{r['stages']['INSERT']['samples_commanded']} samples, "
          f"ROTATE {r['stages']['ROTATE'].get('samples_feasible', 0)}/{r['stages']['ROTATE'].get('samples_commanded', 851)}")
    print("  first unacceptable contact: " + ("none" if not fu else
          f"{fu['stage']} t={fu['t_s'] or 0:.2f} s {fu['status']} ({fu['contact']}, {fu.get('pen_mm') or 0:.3f} mm)"))
    if f:
        print(f"  final: {abs(f['yaw_deg']):.2f} deg {'CW' if f['yaw_deg'] < 0 else 'CCW'}, gap {f['seating_gap_mm']:.3f} mm, "
              f"offset {f['centre_offset_mm']:.3f} mm")
    print(f"  expected outcome: functional {r['functional_outcome']}, strict {r['strict_outcome']}", flush=True)


# ------------------------------------------------------------------ fake hardware
def preflight(node, title, lines=()):
    """Verify fake hardware, (re)load the planning scene (detaches any held part), verify poses."""
    print(title, flush=True)
    for line in lines:
        print("  " + line)
    node.guard(); node.fresh()
    print("Fake hardware verified. Loading planning scene...", flush=True)
    node.scene.load_all()
    for i in range(1, 6):
        node.scene.set_allowed(f"pickup_V{i}", f"fitting_V{i}", False)
    poses0, frame = node.scene.world_poses()
    objs = sm.scene_objects()
    bad = [k for k in pst.OBJECT_IDS if k not in poses0 or sim_node.T_err(poses0[k], objs[k]["T"])[0] > 1e-3]
    if bad:
        raise RuntimeError(f"Planning-scene readback mismatch: {bad}")
    print(f"Planning scene: {len(pst.OBJECT_IDS)} objects verified (MoveIt frame '{frame}')", flush=True)


def execute(node, i, level, p, calib, objs, run_dir, args):
    obj_id = f"pickup_V{i}"
    folder = run_dir / f"V{i}_{level}"; folder.mkdir(parents=True, exist_ok=False)
    res, geom, G = p["res"], p["geom"], p["ctx"]["G"]
    summary = {"type": "KINEMATIC SIMULATION (fake hardware) - not a physical result", "variant": f"V{i}", "level": level,
               "note": "Pickup/transport NOT simulated: the part is attached in the gripper 40 mm above its start pose",
               "playback_speed": getattr(node, "speed", 1.0),
               "stages": {}}
    print(f"\n================ V{i} {level} ================", flush=True)
    if not args.skip_home:
        # joint min-jerk to the taught HOME, collision-checked; scales with --speed (was a fixed 10 s move)
        q_now = node.fresh()
        if lp.maxdiff(q_now, HOME) > 1e-3:
            t_h, q_h = approach_path(q_now, HOME)
            bad, _ = node.check_path("HOME", t_h, q_h, calib)
            if bad:
                raise RuntimeError(f"HOME collisions: {bad[:3]}")
            node.run_arm("HOME", t_h, q_h)
    node.run_gripper("OPEN", sm.RG2_OPEN_M)
    q_home = node.fresh()
    feas = [r for r in res["rows"] if r["feasible"]]
    if args.full:
        full = load_full(Path(args.full_dir), i)
        k = unwrap_to(q_home, [q for _, q in full.values()] + [[r["q"] for r in feas]])
        if k:
            print(f"  wrist_3 shifted by {k} x 2pi to stay near HOME (same poses)", flush=True)
        summary["note"] = "Full sequence: Stage 10 CDMP-retargeted REACH/TRANSPORT (offline plan), executed and MoveIt-checked here"
        run_full(node, i, full, G, objs, calib, summary)
        if "ALIGN_INSERT" not in full:      # Stage 10 plan stops where its IK failed (V4/V5): show it and stop
            import json as _json
            sf = Path(args.full_dir) / "stage10_summary.json"
            fail = _json.load(open(sf))["variants"].get(f"V{i}", {}).get("failure") if sf.exists() else None
            print(f"\nSTOPPED: V{i} TRANSPORT ends here - the planned CDMP path could not be continued.\n  {fail}",
                  flush=True)
            res["functional_outcome"] = res["strict_outcome"] = "FAIL"
            res["first_unacceptable"] = {"stage": "TRANSPORT", "sample": (fail or {}).get("sample"),
                                         "t_s": (fail or {}).get("t_s"), "status": "IK_FAIL (elbow singularity)",
                                         "contact": str((fail or {}).get("target_tcp_mm"))}
            res["pre_stages"] = summary
            out8.write_outputs(str(folder), res)
            return res
        if "INSERT" in full:      # posture-A plans carry INSERT/ROTATE re-solved on their own IK branch
            feas = [{"q": q, "stage": st, "sample": n_, "t_s": t_}
                    for st in ("INSERT", "ROTATE") if st in full for n_, (t_, q) in enumerate(zip(*full[st]))]
            print(f"INSERT/ROTATE: same Cartesian plan, joint solution from {args.full_dir} (posture A)", flush=True)
        node.scene.set_allowed(obj_id, f"fitting_V{i}", True)
        res = finish_insert(node, i, level, p, calib, objs, folder, res, geom, G, feas, summary, obj_id)
        rr = Path(args.full_dir) / f"V{i}" / "release_retreat.csv"
        if res["functional_outcome"] != "PASS":
            print(f"V{i}: not assembled -> no RELEASE; the part stays in the gripper.", flush=True)
        elif not rr.exists():
            print(f"V{i}: no RELEASE/RETREAT plan in {args.full_dir} (run plan_release_retreat.py)", flush=True)
        else:
            run_release(node, i, rr, k, calib, summary)
            res["pre_stages"] = summary
            out8.write_outputs(str(folder), res)
        return res
    k = unwrap_to(q_home, [p["q_desc"], [r["q"] for r in feas]])
    if k:
        print(f"  wrist_3 shifted by {k} x 2pi to stay near HOME (same poses)", flush=True)
    # APPROACH (no part attached yet)
    t_a, q_a = approach_path(q_home, p["q_desc"][0])
    bad, n = node.check_path("APPROACH", t_a, q_a, calib)
    if bad:
        raise RuntimeError(f"APPROACH collisions: {bad[:3]}")
    node.run_arm("APPROACH", t_a, q_a)
    summary["stages"]["APPROACH"] = {"samples": len(q_a), "duration_s": t_a[-1], "states_checked": n,
                                     "timing": f"SIM min-jerk, peak <= {APPROACH_PEAK_RAD_S} rad/s"}
    # ATTACH at the planned grasp
    node.run_gripper("GRIP", sm.RG2_GRIP_M)
    node.scene.attach_part(obj_id, G, objs[obj_id]["mesh"])
    print(f"ATTACH: {obj_id} grasp xyz {np.round(G[:3, 3]*1000, 3).tolist()} mm (planned grasp; pickup not simulated)",
          flush=True)
    # DESCEND
    bad, n = node.check_path("DESCEND", p["t_desc"], p["q_desc"], calib)
    if bad:
        raise RuntimeError(f"DESCEND collisions: {bad[:3]}")
    node.run_arm("DESCEND", p["t_desc"], p["q_desc"])
    summary["stages"]["DESCEND"] = {"samples": len(p["q_desc"]), "duration_s": p["t_desc"][-1], "states_checked": n,
                                    "height_mm": ABOVE_M * 1000}
    # INSERT / ROTATE: MoveIt checks all but the part<->own-fitting pair (CAD classifier)
    node.scene.set_allowed(obj_id, f"fitting_V{i}", True)
    return finish_insert(node, i, level, p, calib, objs, folder, res, geom, G, feas, summary, obj_id)


def run_release(node, i, path, k, calib, summary):
    """RELEASE (RG2 to 40 mm, part detached and left seated in the world), RETREAT +30 mm, HOME."""
    import csv
    stages = {}
    for r in csv.DictReader(open(path)):
        t, q = stages.setdefault(r["stage"], ([], []))
        t.append(float(r["t_s"])); q.append([float(r[j]) + (2 * math.pi * k if j == JOINTS[5] else 0.0) for j in JOINTS])
    node.run_gripper("RELEASE", sm.RG2_OPEN_M)
    node.scene.detach_v1_if_attached(remove=False)          # MoveIt keeps it in the world, seated in its fitting
    print(f"DETACH: pickup_V{i} left in fitting_V{i}", flush=True)
    for stage in ("RETREAT", "HOME"):
        t, q = stages[stage]
        bad, n = node.check_path(stage, t, q, calib)
        if bad:
            raise RuntimeError(f"{stage} collisions (MoveIt): {bad[:3]}")
        node.run_arm(f"V{i} {stage}", t, q)
        summary["stages"][stage] = {"samples": len(q), "duration_s": t[-1], "states_checked": n}


def load_full(d, i):
    import csv
    f = d / f"V{i}" / "joint_trajectory.csv"
    if not f.exists():
        raise RuntimeError(f"No Stage 10 trajectory for V{i} ({f}); run retarget_reach_transport.py first")
    out = {}
    for r in csv.DictReader(open(f)):
        t, q = out.setdefault(r["stage"], ([], []))
        t.append(float(r["t_s"])); q.append([float(r[j]) for j in JOINTS])
    return out


def run_full(node, i, full, G, objs, calib, summary):
    obj_id = f"pickup_V{i}"
    for stage in ("RECONFIG", "REACH", "ALIGN_GRIP", "LIFT", "TRANSPORT", "ALIGN_INSERT"):
        if stage not in full:
            continue
        t, q = full[stage]
        if stage == "LIFT":
            node.run_gripper("GRIP", sm.RG2_GRIP_M)
            node.scene.attach_part(obj_id, G, objs[obj_id]["mesh"])
            node.scene.set_allowed(obj_id, "pickup_plate", True)          # resting in its pocket (Stage 6)
            print(f"ATTACH: {obj_id} at grasp xyz {np.round(G[:3, 3]*1000, 3).tolist()} mm", flush=True)
        bad, n = node.check_path(stage, t, q, calib)
        if bad:
            raise RuntimeError(f"{stage} collisions (MoveIt): {bad[:3]}")
        node.run_arm(f"V{i} {stage}", t, q)
        if stage == "LIFT":
            node.scene.set_allowed(obj_id, "pickup_plate", False)
        summary["stages"][stage] = {"samples": len(q), "duration_s": t[-1], "states_checked": n,
                                    "timing": "CDMP tau 2" if stage in ("REACH", "TRANSPORT") else "min-jerk"}


def finish_insert(node, i, level, p, calib, objs, folder, res, geom, G, feas, summary, obj_id):
    other = []
    for r in feas:
        ok, contacts = node.valid(r["q"])
        if not ok:
            other.append({"stage": r["stage"], "sample": r["sample"], "t_s": r["t_s"], "contacts": contacts}); break
    fk_dev = max(sim_node.T_err(node.fk(r["q"]), uk.tcp(r["q"], calib))[0] for r in feas[::25] + feas[-1:])
    print(f"Planned INSERT/ROTATE: MoveIt FK agreement {fk_dev:.6f} mm; other collisions: {other or 'none'}", flush=True)
    if other:
        cut = feas.index(next(r for r in feas if r["stage"] == other[0]["stage"] and r["sample"] == other[0]["sample"]))
        feas = feas[:cut]
        res["functional_outcome"] = res["strict_outcome"] = "FAIL"
        res["first_unacceptable"] = {"stage": other[0]["stage"], "sample": other[0]["sample"], "t_s": other[0]["t_s"],
                                     "status": "OTHER_COLLISION", "contact": ";".join(other[0]["contacts"])}
    executed = {}
    for stage in ("INSERT", "ROTATE"):
        seg = [r for r in feas if r["stage"] == stage]
        if len(seg) < 2:
            continue
        t = [r["t_s"] - seg[0]["t_s"] for r in seg]
        _, err = node.run_arm(f"V{i} {level} {stage}", t, [r["q"] for r in seg])
        executed[stage] = {"samples": len(seg), "duration_s": t[-1], "endpoint_err_rad": err}
    node.scene.set_allowed(obj_id, f"fitting_V{i}", False)
    # independent end-state check (TF + attached-object pose)
    T_tcp = pst.lookup(node, "base", "gripper_tcp")
    att = node.scene.attached()[obj_id].object.pose
    T_g = sm.pose_to_matrix((att.position.x, att.position.y, att.position.z),
                            (att.orientation.x, att.orientation.y, att.orientation.z, att.orientation.w))
    T_v = T_tcp @ T_g
    lat, zb, yaw, tilt = s8.v1_in_fitting(T_v, p["ctx"]["T_fit"])
    ev = geom.evaluate(T_v, "ROTATE" if "ROTATE" in executed else "INSERT", yaw)
    res["executed"] = {"stages": executed, "other_collisions": other, "moveit_fk_agreement_mm": fk_dev,
                       "end_state_from_tf": {"centre_offset_mm": float(np.hypot(*lat)), "tilt_deg": tilt, "yaw_deg": yaw,
                                             "seating_gap_mm": geom.lowest_z_mm(T_v) - s8.FLOOR_Z_MM,
                                             "retained": geom.retained(T_v), "contact_status": ev["status"]}}
    res["pre_stages"] = summary
    out8.write_outputs(str(folder), res)
    e = res["executed"]["end_state_from_tf"]
    print(f"\nV{i} {level}: functional {res['functional_outcome']}, strict {res['strict_outcome']}")
    print(f"  end state (TF): {abs(e['yaw_deg']):.2f} deg {'CW' if e['yaw_deg'] < 0 else 'CCW'}, gap {e['seating_gap_mm']:.3f} mm, "
          f"offset {e['centre_offset_mm']:.3f} mm, retained {e['retained']}, contact {e['contact_status']}", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", default="4", help="1..5 or all")
    ap.add_argument("--level", default="L2", choices=list(vc.LEVELS))
    ap.add_argument("--run", action="store_true", help="Run with one START confirmation")
    ap.add_argument("--auto", action="store_true", help="Run without prompts (fake hardware only)")
    ap.add_argument("--skip-home", action="store_true")
    ap.add_argument("--plan-only", action="store_true", help="Plan and print; no ROS")
    ap.add_argument("--features", default=str(ROOT / "results/cad_features"))
    ap.add_argument("--full", action="store_true", help="whole sequence: Stage 10 CDMP REACH/TRANSPORT before INSERT/ROTATE")
    ap.add_argument("--full-dir", default=str(ROOT / "results/simulation/stage10_current")
                    if (ROOT / "results/simulation/stage10_current").exists()
                    else str(ROOT / "results/sim_reference/stage10_retarget_20261031_zscale"),
                    help="Stage 10 plans (default: results/simulation/stage10_current from UPDATE_SIM.sh, "
                         "else the 31 Oct V1-V5 reference)")
    sim_node.add_speed_arg(ap)
    args = ap.parse_args()
    variants = [1, 2, 3, 4, 5] if args.variant == "all" else [int(args.variant)]
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects()
    G, _ = rvo.grasps(objs, calib)
    prof = vc.demo_profiles(str(ROOT))
    q1, T_tcp1, z1, yaw0 = rvo.v1_start(objs, calib, G[1])
    print("Planning (CAD classifier; ~1 min the first time after a geometry change, then cached: a few seconds)...",
          flush=True)
    prewarm(variants, args.level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, args.features)
    plans = {i: plan_cached(i, args.level, calib, objs, G, prof, q1, T_tcp1, z1, yaw0, args.features) for i in variants}
    for i in variants:
        print_plan(i, args.level, plans[i])
    if args.plan_only:
        return 0
    pst.require_sim_domain()
    rclpy, node = sim_node.make_node("variant_insert_SIM_ONLY")
    sim_node.set_speed(node, args.speed, args.max_joint_speed)
    try:
        preflight(node, f"STAGE 9 V{'/V'.join(map(str, variants))} {args.level} INSERT/ROTATE — FAKE HARDWARE ONLY "
                  "(ROS_DOMAIN_ID=73)",
                  (["REACH         CDMP (V1 demo, goal = this variant's pocket), tau 2",
                    "ALIGN_GRIP    1 s straight to the exact grip pose; GRIP + ATTACH",
                    "LIFT          30 mm, 3 s min-jerk",
                    "TRANSPORT     CDMP (V1 demo, goal = this variant's fitting), tau 2",
                    "ALIGN_INSERT  1 s straight to the INSERT start"] if args.full else
                   ["APPROACH      HOME -> 40 mm above the start (joint min-jerk)",
                    "ATTACH        part attached in the gripper (pickup/transport NOT simulated)",
                    f"DESCEND       40 mm straight down, {DESCEND_S:.1f} s min-jerk"])
                  + [f"INSERT/ROTATE {args.level} plan, original CDMP timestamps"]
                  + (["RELEASE       RG2 to 40 mm, part left seated", "RETREAT       +30 mm straight up",
                      "HOME          joint min-jerk"] if args.full else []))
        if not (args.run or args.auto):
            print("PREFLIGHT PASS — NO ROBOT MOVEMENT. Re-run with --run or --auto."); return 0
        if args.run and input(f"Type START to run {args.level} on V{', V'.join(map(str, variants))}: ").strip() != "START":
            raise RuntimeError("Cancelled before motion")
        run_dir = ROOT / "results/simulation/stage9" / datetime.now().strftime("%Y%m%d_%H%M%S")
        out = {}
        for n, i in enumerate(variants):
            if n:
                print("\nResetting scene (parts back to their pockets)...", flush=True)
                preflight(node, f"Preparing V{i}")
            out[i] = execute(node, i, args.level, plans[i], calib, objs, run_dir, args)
        print("\n================ STAGE 9 SUMMARY (fake hardware) ================")
        for i, r in out.items():
            fu = r["first_unacceptable"]
            print(f"  V{i} {args.level}: functional {r['functional_outcome']:4s} strict {r['strict_outcome']:4s}  first bad: "
                  f"{'none' if not fu else fu['stage'] + ' ' + str(fu['status'])}")
        print(f"Results -> {run_dir}")
        print("Reset afterwards: python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset")
        return 0
    except KeyboardInterrupt:
        print("Interrupted.")
        return 2
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
