#!/usr/bin/env python3
"""Stage 8: V1 INSERT/ROTATE experiments A, B1, B2 on FAKE hardware (ROS_DOMAIN_ID=73).

  ./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh    # terminal 1
  ./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh   # terminal 2
  export ROS_DOMAIN_ID=73
  pip install trimesh rtree python-fcl scipy                     # once (contact classification)
  python3 scripts/phase8_insert_sim/run_v1_insert_sim.py --experiment all --run

Each experiment starts from a fresh Stage 7 run (pickup -> lift -> bridge -> transport ->
ALIGN_INSERT), then:
  A   original CDMP INSERT/ROTATE poses (as learned)
  B1  CORRECTION, then CDMP replayed rigidly relative to the corrected start
  B2  CORRECTION, then geometry-conditioned CDMP (depth + yaw along/about the fitting axis)
  B2R B2, then ROTATE even though INSERT stopped short of the seat (--experiment B2R)
Per-sample IK, original CDMP timestamps. V1<->fitting_V1 contact is classified against CAD
mating regions (stage8_core.py); all other collisions are checked by MoveIt. Only the feasible
part is executed; the trial stops at the first unacceptable contact. ROTATE only if seated.
Results: results/simulation/stage8/<run timestamp>/<experiment>/ (trajectory.csv,
collision_log.csv, summary.json). Kinematic simulation only; not a physical result.
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
for d in (HERE, HERE.parent / "phase7_transport_sim", HERE.parent / "phase6_pickup_sim", HERE.parent / "scene"):
    sys.path.insert(0, str(d))
import plan_cache  # noqa: E402
import planning_scene_tools as pst  # noqa: E402
import preview_stage8_offline as out8  # noqa: E402
import run_v1_pickup_lift_sim as s6  # noqa: E402
import run_v1_transport_sim as s7  # noqa: E402
import scene_model as sm  # noqa: E402
import sim_node  # noqa: E402
import stage8_core as s8  # noqa: E402
import ur_kinematics as uk  # noqa: E402

np = sm.np
ROOT = Path(sm.PKG)


def run_one(node, exp, inputs, args, run_dir, poses0):
    name = s8.EXPERIMENTS[exp]
    folder = run_dir / name; folder.mkdir(parents=True, exist_ok=False)
    print(f"\n================ EXPERIMENT {exp} ({name}) ================", flush=True)
    traj, transport, align = inputs
    stage7 = {"type": "KINEMATIC SIMULATION (fake hardware) - not a physical result", "stages": {}}
    ctx = s7.run_stage7(node, traj, transport, align, args, folder, stage7, poses0)
    if stage7["result"] != "PASS":
        raise RuntimeError("Stage 7 motion checks failed; experiment not run")
    with (folder / "stage7_summary.json").open("w") as f:
        json.dump(stage7, f, indent=2, default=float)

    calib, objs, T_grasp = ctx["calib"], ctx["objs"], ctx["T_grasp"]
    T_fit = node.scene.world_poses()[0]["fitting_V1"]
    q_align = node.fresh()
    geom = s8.Geometry(objs, T_fit)
    files = ["scripts/phase8_insert_sim/stage8_core.py", "scripts/scene/ur_kinematics.py", s8.INSERT_CSV, s8.ROTATE_CSV,
             objs["pickup_V1"]["mesh"], objs["fitting_V1"]["mesh"]]
    k = plan_cache.key(f"stage8_{exp}", {"q": [round(float(v), 6) for v in q_align],
                                         "T_fit": np.round(T_fit, 9).tolist(), "T_grasp": np.round(T_grasp, 9).tolist(),
                                         "calib": calib}, files)
    res = plan_cache.get(k)
    if res is None:
        res = s8.run_experiment(exp, q_align, calib, T_grasp, T_fit, objs, str(ROOT), geom)
        plan_cache.put(k, res)
    else:
        print(f"{exp}: cached plan (same start state, CAD and code)", flush=True)

    # ---- MoveIt cross-checks on the planned samples
    rows = res["rows"]
    fk_dev = max(sim_node.T_err(node.fk(r["q"]), uk.tcp(r["q"], calib))[0] for r in rows[::10] + rows[-1:])
    node.scene.set_allowed("pickup_V1", "fitting_V1", True)      # this pair is judged by the CAD classifier
    other = []
    for r in rows:
        if not r["feasible"]:
            break
        ok, contacts = node.valid(r["q"])
        if not ok:
            other.append({"stage": r["stage"], "sample": r["sample"], "t_s": r["t_s"], "contacts": contacts})
            r["feasible"] = False; r["status"] = "OTHER_COLLISION"
            break
    if other:
        cut = rows.index(next(r for r in rows if r["status"] == "OTHER_COLLISION"))
        res["rows"] = rows = rows[:cut + 1]
        res["first_unacceptable"] = {"stage": other[0]["stage"], "sample": other[0]["sample"], "t_s": other[0]["t_s"],
                                     "status": "OTHER_COLLISION", "contact": ";".join(other[0]["contacts"])}
        res["outcome"] = "FAIL"
    print(f"Planned: FK agreement {fk_dev:.6f} mm; MoveIt (non-fitting) collisions: {other or 'none'}", flush=True)

    # ---- execute the feasible part, stage by stage (original CDMP timestamps)
    executed = {}
    for stage in ("CORRECTION", "INSERT", "ROTATE"):
        seg = [r for r in rows if r["stage"] == stage and r["feasible"]]
        if len(seg) < 2:
            continue
        t = [r["t_s"] - seg[0]["t_s"] for r in seg]
        _, err = node.run_arm(f"{exp} {stage}", t, [r["q"] for r in seg])
        executed[stage] = {"samples": len(seg), "duration_s": t[-1], "elapsed_s": round(node.last_elapsed, 3),
                           "endpoint_err_rad": err}
    node.scene.set_allowed("pickup_V1", "fitting_V1", False)

    # ---- verify the executed end state independently (TF + attached-object pose)
    T_tcp = pst.lookup(node, "base", "gripper_tcp")
    att = node.scene.attached()["pickup_V1"].object.pose
    T_g = sm.pose_to_matrix((att.position.x, att.position.y, att.position.z),
                            (att.orientation.x, att.orientation.y, att.orientation.z, att.orientation.w))
    T_v1 = T_tcp @ T_g
    lat, zb, yaw, tilt = s8.v1_in_fitting(T_v1, T_fit)
    ev = geom.evaluate(T_v1, "ROTATE" if "ROTATE" in executed else "INSERT", yaw)
    res["executed"] = {"stages": executed,
                       "end_state_from_tf": {"centre_offset_mm": float(np.hypot(*lat)), "tilt_deg": tilt, "yaw_deg": yaw,
                                             "lowest_z_mm": geom.lowest_z_mm(T_v1),
                                             "seating_error_mm": geom.lowest_z_mm(T_v1) - s8.FLOOR_Z_MM,
                                             "contact_status": ev["status"], "penetration_mm": ev["pen_mm"]},
                       "planned_vs_executed_end_mm": (abs(geom.lowest_z_mm(T_v1) - res["final"]["lowest_z_mm"])
                                                      if res["final"] else None),
                       "moveit_fk_agreement_mm": fk_dev, "other_collisions": other}
    res["type"] = "KINEMATIC SIMULATION (fake hardware) - not a physical result"
    res["timing"] = {"cdmp_insert_duration_s": res["stages"]["INSERT"].get("cdmp_duration_s"),
                     "cdmp_rotate_duration_s": res["stages"]["ROTATE"].get("cdmp_duration_s"),
                     "correction_time_s": res["stages"].get("CORRECTION", {}).get("duration_s", 0.0),
                     "note": "CDMP timestamps preserved; correction uses NEW 2.0 s min-jerk sim timing",
                     "playback_speed": node.speed}
    res["tolerances"] = {"contact_tol_mm": s8.CONTACT_TOL_MM, "floor_tol_mm": s8.FLOOR_TOL_MM,
                         "rotation_stop_from_deg": s8.STOP_YAW_DEG, "radial_clearance_mm": s8.RADIAL_CLEARANCE_MM,
                         "pass_cw_rotation_deg": s8.PASS_YAW_RANGE_DEG}
    res["fitting_registration_note"] = "fitting_V1 registered from v1_insertion_end TCP (assumes TCP on V1 axis); PROVISIONAL"
    out8.write_outputs(str(folder), res)
    print_result(exp, res)
    return res


def print_result(exp, r):
    fu = r["first_unacceptable"]
    print(f"\n{exp}: {r['outcome']}")
    print(f"  initial offset {r['initial']['centre_offset_mm']:.3f} mm, tilt {r['initial']['tilt_deg']:.3f} deg")
    if "CORRECTION" in r["stages"]:
        c = r["stages"]["CORRECTION"]["after"]
        print(f"  after correction offset {c['centre_offset_mm']:.4f} mm, tilt {c['tilt_deg']:.4f} deg")
    print(f"  max insertion depth {r['max_insertion_depth_mm']:.3f} mm; min clearance {r['min_clearance_mm']:.3f} mm; "
          f"max penetration {r['max_penetration_mm']:.4f} mm")
    print("  first unacceptable contact: " + ("none" if not fu else
          f"{fu['stage']} sample {fu['sample']} t={fu['t_s']:.2f} s {fu['status']} ({fu['contact']})"))
    if r["final"]:
        print(f"  final: seating error {r['final']['seating_error_mm']:.3f} mm, rotation {-r['final']['yaw_deg']:.2f} deg CW, "
              f"offset {r['final']['centre_offset_mm']:.3f} mm")
    print(f"  INSERT seated: {r['stages']['INSERT'].get('seated')}; ROTATE: {r['stages']['ROTATE'].get('status', 'executed')}")
    if "functional_outcome" in r:
        fc = r["functional_criteria"]
        print(f"  FUNCTIONAL (B2R): {r['functional_outcome']}  strict: {r['outcome']}  seating gap start/final "
              f"{fc['seating_error_at_rotate_start_mm']:.3f}/{fc['seating_error_final_mm']:.3f} mm, max CW {fc['max_cw_rotation_deg']:.2f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--experiment", choices=["A", "B1", "B2", "B2R", "all"], default="all",
                    help="all = A, B1, B2. B2R = B2 then ROTATE from the unseated INSERT end state (follow-up)")
    ap.add_argument("--run", action="store_true", help="Run with one START confirmation")
    ap.add_argument("--auto", action="store_true", help="Run without prompts (fake hardware only)")
    ap.add_argument("--skip-home", action="store_true")
    sim_node.add_speed_arg(ap)
    args = ap.parse_args()
    pst.require_sim_domain()
    import importlib.util
    missing = [m for m in ("fcl", "rtree", "trimesh", "scipy") if importlib.util.find_spec(m) is None]
    if missing:
        raise SystemExit(f"Stage 8 needs {missing}. Install: pip install trimesh rtree python-fcl scipy")
    inputs = s7.load_stage7_inputs()
    exps = ["A", "B1", "B2"] if args.experiment == "all" else [args.experiment]
    rclpy, node = sim_node.make_node("v1_insert_SIM_ONLY")
    sim_node.set_speed(node, args.speed, args.max_joint_speed)
    try:
        poses0, frame = s6.preflight(node, "V1 STAGE 8 INSERT/ROTATE EXPERIMENTS — FAKE HARDWARE ONLY (ROS_DOMAIN_ID=73)",
                                     inputs[0], s7.STAGE7_LINES + [f"EXPERIMENTS   {', '.join(exps)}"])
        if not (args.run or args.auto):
            print("PREFLIGHT PASS — NO ROBOT MOVEMENT. Re-run with --run or --auto."); return 0
        if args.run and input(f"Type START to run experiments {', '.join(exps)}: ").strip() != "START":
            raise RuntimeError("Cancelled before motion")
        run_dir = ROOT / "results/simulation/stage8" / datetime.now().strftime("%Y%m%d_%H%M%S")
        results = {}
        for i, exp in enumerate(exps):
            if i:
                print("\nResetting scene (V1 back to its pocket) for the next experiment...", flush=True)
                poses0, _ = s6.preflight(node, f"Preparing experiment {exp}", inputs[0])
            results[exp] = run_one(node, exp, inputs, args, run_dir, poses0)
        print("\n================ STAGE 8 SUMMARY ================")
        for exp, r in results.items():
            fu = r["first_unacceptable"]
            print(f"  {exp:3s} {s8.EXPERIMENTS[exp]:26s} {r['outcome']:4s}  depth {r['max_insertion_depth_mm']:.2f} mm  "
                  f"first bad: {'none' if not fu else fu['stage'] + ' t=' + format(fu['t_s'], '.2f') + ' s ' + fu['status']}")
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
