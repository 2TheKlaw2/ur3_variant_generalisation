#!/usr/bin/env python3
"""Stage 7: V1 pickup -> LIFT -> BRIDGE -> TRANSPORT -> ALIGN_INSERT -> STOP (FAKE hardware only).

  ./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh    # terminal 1
  ./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh   # terminal 2 (MoveIt + RViz)
  export ROS_DOMAIN_ID=73
  python3 scripts/phase7_transport_sim/run_v1_transport_sim.py          # preflight only, no motion
  python3 scripts/phase7_transport_sim/run_v1_transport_sim.py --run    # one START confirmation
  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset     # put V1 back afterwards

Stops after ALIGN_INSERT (no insertion): the physical pilot needed an E-stop at insertion,
so this stage measures how V1 arrives at fitting_V1 before Stage 8 inserts anything.

- Pickup + LIFT: Stage 6 (unchanged).
- BRIDGE: re-planned in simulation with V1 attached and all CAD objects loaded, using
  the physically tested raised-bridge geometry (run_v1_lift_bridge_transport_staged.py):
  join row = first learned transport row >= 31 mm above the grip TCP (expected 24).
  Per-sample IK every 20 ms; NEW sim timing 6.0 s (min-jerk per segment).
- TRANSPORT: learned joint trajectory from the join row, original timing x1.5 (as tested).
- ALIGN_INSERT: existing alignment trajectory x2.0 (as run_v1_integrated_fake.py).
- Every stage: every sample + joint-linear in-between states (TCP step <= 1 mm)
  collision-checked with V1 attached (V1<->fitting_V1 is checked, not allowed).
- At STOP: V1 pose relative to fitting_V1 and a no-motion straight-down insertion probe.
Kinematic simulation only; not a physical result.
"""
import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / "phase6_pickup_sim")); sys.path.insert(0, str(HERE.parent / "scene"))
import lift_profile as lp  # noqa: E402
import planning_scene_tools as pst  # noqa: E402
import run_v1_pickup_lift_sim as s6  # noqa: E402
import scene_model as sm  # noqa: E402
import sim_node  # noqa: E402
import transport_profile as tp  # noqa: E402
import ur_kinematics as uk  # noqa: E402

np = sm.np
ROOT = Path(sm.PKG)
JOINTS = lp.JOINTS
CLEARANCE = {"radial_mm": 0.25, "tab_slot_half_angle_deg": 1.0}  # from cad/CAD_CHECKS.md


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)


def insertion_probe(node, q_end, T_grasp, T_fit, calib, max_depth_m=0.006, step_m=0.00025):
    """No motion: lower the TCP straight down in base z and report the first colliding depth."""
    T0 = uk.tcp(q_end, calib); seed = list(q_end); rows = []
    for k in range(1, int(round(max_depth_m / step_m)) + 1):
        Tk = T0.copy(); Tk[2, 3] -= k * step_m
        seed = uk.ik_local(seed, Tk, calib).tolist()
        ok, contacts = node.valid(seed)
        a = tp.alignment_in_fitting(Tk @ T_grasp, T_fit)
        rows.append({"descent_mm": round(k * step_m * 1000, 3), "valid": ok, "contacts": contacts,
                     "v1_bottom_above_fitting_base_mm": a["v1_bottom_above_fitting_base_mm"]})
        if not ok:
            break
    first = next((r for r in rows if not r["valid"]), None)
    return {"first_contact_descent_mm": first["descent_mm"] if first else None,
            "first_contact_pairs": first["contacts"] if first else [],
            "first_contact_v1_bottom_above_fitting_base_mm": first["v1_bottom_above_fitting_base_mm"] if first else None,
            "probe_max_descent_mm": round(max_depth_m * 1000, 2), "steps": rows}


def optional_clearance(T_v1, objs):
    """Exact min distance V1 <-> fitting_V1 if python-fcl is installed (else None)."""
    try:
        import fcl
    except ImportError:
        return None

    def obj(path, T):
        v, f = sm.read_stl(path)
        g = fcl.BVHModel(); g.beginModel(len(v), len(f)); g.addSubModel(v, f); g.endModel()
        return fcl.CollisionObject(g, fcl.Transform(T[:3, :3], T[:3, 3]))
    a = obj(objs["pickup_V1"]["mesh"], T_v1); b = obj(objs["fitting_V1"]["mesh"], objs["fitting_V1"]["T"])
    r = fcl.DistanceResult(); fcl.distance(a, b, fcl.DistanceRequest(), r)
    c = fcl.CollisionResult(); fcl.collide(a, b, fcl.CollisionRequest(), c)
    return 0.0 if c.is_collision else round(r.min_distance * 1000, 4)


def load_stage7_inputs():
    traj = s6.load_pickup_trajectories()
    sha, t_tr, q_tr = tp.load_transport(str(ROOT))
    align = sim_node.load_arm(tp.ALIGN_INSERT_CSV, tp.ALIGN_INSERT_MULTIPLIER, 0.05)
    if lp.maxdiff(q_tr[-1], align["q"][0]) > 0.010:
        raise RuntimeError("TRANSPORT end -> ALIGN_INSERT start handoff mismatch")
    return traj, (sha, t_tr, q_tr), align


STAGE7_LINES = [f"BRIDGE        raised bridge to learned row {tp.EXPECTED_JOIN_ROW}, {tp.BRIDGE_SIM_DURATION_S:.1f} s min-jerk (SIM timing)",
                f"TRANSPORT     learned rows {tp.EXPECTED_JOIN_ROW}..{tp.TRANSPORT_ROWS-1}, original timing x{tp.TRANSPORT_TIME_SCALE}",
                "ALIGN_INSERT  existing alignment trajectory x2.0"]


def run_stage7(node, traj, transport, align, args, log, summary, poses0):
    """Pickup + LIFT (Stage 6) -> BRIDGE -> TRANSPORT -> ALIGN_INSERT; fills summary, returns context."""
    sha, t_tr, q_tr = transport
    ctx = s6.run_pickup_and_lift(node, traj, args, log, summary, poses0)
    if summary["result"] != "PASS":
        raise RuntimeError("Stage 6 pickup/lift checks failed; not continuing")
    summary["pickup_lift_checks"] = summary.pop("checks"); summary.pop("result")
    calib, objs, T_grasp = ctx["calib"], ctx["objs"], ctx["T_grasp"]
    z_grip = ctx["T_tcp_grip"][2, 3]
    checks = {}

    # ---------------- BRIDGE (re-planned with V1 attached and the CAD scene)
    q_now = node.fresh()
    if lp.maxdiff(q_now, ctx["q_lift_end"]) > 0.002:
        raise RuntimeError("Robot is not at the simulated lift end")
    row, rise, high = tp.find_join_row(q_tr, z_grip, calib)
    if row != tp.EXPECTED_JOIN_ROW:
        raise RuntimeError(f"Unexpected join row {row} (physical run used {tp.EXPECTED_JOIN_ROW}); NO MOTION")
    T_lift_end = node.fk(q_now); T_row = node.fk(q_tr[row])
    W = tp.bridge_targets(T_lift_end, T_row)
    frac, n_mv = node.cartesian(W[1:], "Cartesian bridge")
    t_b, q_b, T_b, durs, wraps = tp.plan_bridge(q_now, T_lift_end, T_row, calib)
    gap = lp.maxdiff(q_b[-1], q_tr[row])
    dev = max(sim_node.T_err(node.fk(q), T)[0] for q, T in zip(q_b[::5], T_b[::5]))
    low = min(node.fk(q)[2, 3] - z_grip for q in q_b[::5])
    step, speed = lp.verify_path("BRIDGE", q_b.tolist(), t_b.tolist())
    print(f"BRIDGE planned: join row {row} (+{rise*1000:.1f} mm), MoveIt Cartesian fraction {frac:.4f} "
          f"({n_mv} pts); {len(t_b)} IK samples, segments {np.round(durs, 2).tolist()} s, join gap {gap:.2e} rad, "
          f"min height {low*1000:.2f} mm, max FK dev {dev:.5f} mm, peak speed {speed:.4f} rad/s", flush=True)
    bad_b, n_b = node.check_path("BRIDGE", t_b, q_b.tolist(), calib)
    checks["BRIDGE joins learned transport (<= 0.0055 rad)"] = gap <= tp.JOIN_TOL_RAD
    checks["BRIDGE stays >= 28 mm above pickup"] = low >= tp.BRIDGE_MIN_HEIGHT_M
    checks["BRIDGE collision-free (samples + interpolated)"] = not bad_b
    if bad_b:
        raise RuntimeError(f"BRIDGE collisions: {bad_b[:3]}")
    s_b, e_b = node.run_arm("BRIDGE", t_b.tolist(), q_b.tolist())
    summary["stages"]["BRIDGE"] = {"join_row": row, "join_rise_mm": rise * 1000, "max_learned_rise_mm": high * 1000,
                                   "moveit_cartesian_fraction": frac, "ik_samples": len(t_b),
                                   "segment_durations_s": durs, "join_gap_rad": gap, "min_height_mm": low * 1000,
                                   "max_fk_dev_mm": dev, "peak_joint_speed_rad_s": speed, "wraps": wraps,
                                   "states_checked": n_b, "endpoint_err_rad": e_b,
                                   "timing": f"SIM {tp.BRIDGE_SIM_DURATION_S} s min-jerk per segment (physical: MoveIt, >= 6 s)"}
    write_csv(log / "bridge_planned.csv", ["t_s"] + JOINTS, [[float(a), *b] for a, b in zip(t_b, q_b.tolist())])

    # ---------------- TRANSPORT tail (learned, x1.5)
    t_t, q_t = tp.transport_tail(t_tr, q_tr, row)
    step, speed = lp.verify_path("TRANSPORT", q_t.tolist(), t_t.tolist(), speed_limit=tp.TAIL_SPEED_LIMIT)
    bad_t, n_t = node.check_path("TRANSPORT", t_t, q_t.tolist(), calib)
    checks["TRANSPORT collision-free (samples + interpolated)"] = not bad_t
    if bad_t:
        raise RuntimeError(f"TRANSPORT collisions: {bad_t[:3]}")
    s_t, e_t = node.run_arm("TRANSPORT", t_t.tolist(), q_t.tolist())
    summary["stages"]["TRANSPORT"] = {"rows": f"{row}..{len(q_tr)-1}", "samples": len(t_t), "duration_s": float(t_t[-1]),
                                      "time_scale": tp.TRANSPORT_TIME_SCALE, "transport_sha256": sha,
                                      "peak_joint_speed_rad_s": speed, "states_checked": n_t, "endpoint_err_rad": e_t}
    write_csv(log / "transport_executed_joints.csv", ["t_s"] + JOINTS, s_t)

    # ---------------- ALIGN_INSERT (existing alignment trajectory)
    bad_a, n_a = node.check_path("ALIGN_INSERT", align["t"], align["q"], calib)
    T_v1_plan = uk.tcp(align["q"][-1], calib) @ T_grasp
    a_plan = tp.alignment_in_fitting(T_v1_plan, objs["fitting_V1"]["T"])
    checks["ALIGN_INSERT collision-free (samples + interpolated)"] = not bad_a
    executed = False
    if not bad_a:
        s_a, e_a = node.run_arm("ALIGN_INSERT", align["t"], align["q"])
        executed = True
        write_csv(log / "align_insert_executed_joints.csv", ["t_s"] + JOINTS, s_a)
    else:
        print(f"ALIGN_INSERT NOT EXECUTED: planned collision(s) {bad_a[:2]}", flush=True)
    summary["stages"]["ALIGN_INSERT"] = {"file": align["file"], "multiplier": align["multiplier"],
                                         "samples": len(align["q"]), "duration_s": align["t"][-1],
                                         "states_checked": n_a, "collisions": bad_a, "executed": executed}

    # ---------------- STOP: how does V1 arrive at fitting_V1?
    q_stop = node.fresh() if executed else align["q"][-1]
    T_tcp = node.fk(q_stop)
    att = node.scene.attached()["pickup_V1"].object.pose
    T_g = sm.pose_to_matrix((att.position.x, att.position.y, att.position.z),
                            (att.orientation.x, att.orientation.y, att.orientation.z, att.orientation.w))
    T_v1 = T_tcp @ T_g
    T_fit = node.scene.world_poses()[0]["fitting_V1"]
    a = tp.alignment_in_fitting(T_v1, T_fit)
    a_tcp = tp.alignment_in_fitting(T_tcp, T_fit)
    probe = insertion_probe(node, q_stop, T_g, T_fit, calib)
    clearance = optional_clearance(T_v1, objs)
    within = (a["lateral_offset_abs_mm"] <= CLEARANCE["radial_mm"] and
              abs(a["tab_to_slot_yaw_error_deg"]) <= CLEARANCE["tab_slot_half_angle_deg"])
    summary["alignment_at_stop"] = {
        "source": "executed joints (TF/FK)" if executed else "planned ALIGN_INSERT end (not executed)",
        "V1_relative_to_fitting_V1": a,
        "TCP_relative_to_fitting_V1": {"lateral_offset_mm": a_tcp["lateral_offset_mm"],
                                       "lateral_offset_abs_mm": a_tcp["lateral_offset_abs_mm"]},
        "planned_vs_executed_lateral_mm": round(abs(a_plan["lateral_offset_abs_mm"] - a["lateral_offset_abs_mm"]), 5),
        "reference_clearances": CLEARANCE,
        "within_lateral_and_yaw_clearance": within,
        "min_distance_V1_to_fitting_V1_mm": clearance,
        "straight_down_insertion_probe": {k: v for k, v in probe.items() if k != "steps"},
        "fitting_registration_note": "fitting_V1 xy = v1_insertion_end TCP (assumes TCP on V1 axis); PROVISIONAL",
    }
    write_csv(log / "insertion_probe.csv", ["descent_mm", "valid", "v1_bottom_above_fitting_base_mm", "contacts"],
              [[r["descent_mm"], r["valid"], r["v1_bottom_above_fitting_base_mm"], ";".join(r["contacts"])] for r in probe["steps"]])
    poses1, _ = node.scene.world_poses()
    moved = max(sim_node.T_err(poses1[k], poses0[k])[0] for k in pst.OBJECT_IDS if k != "pickup_V1")
    checks["V2-V5, plates, fittings unchanged"] = moved < 1e-6
    checks["V1 still attached at STOP"] = "pickup_V1" in node.scene.attached()
    summary["checks"] = {k: bool(v) for k, v in checks.items()}
    summary["result"] = "PASS" if all(summary["checks"].values()) else "FAIL"

    ctx.update({"q_stop": list(q_stop), "align_executed": executed, "alignment": a,
                "clearance": clearance, "within": within})
    return ctx


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="Run with one START confirmation")
    ap.add_argument("--auto", action="store_true", help="Run without prompts (fake hardware only)")
    ap.add_argument("--skip-home", action="store_true", help="Robot already at taught home")
    sim_node.add_speed_arg(ap)
    args = ap.parse_args()
    pst.require_sim_domain()

    traj, transport, align = load_stage7_inputs()

    rclpy, node = sim_node.make_node("v1_transport_SIM_ONLY")
    sim_node.set_speed(node, args.speed, args.max_joint_speed)
    try:
        poses0, frame = s6.preflight(
            node, "V1 PICKUP -> TRANSPORT -> ALIGN_INSERT — FAKE HARDWARE ONLY (ROS_DOMAIN_ID=73)", traj,
            [f"BRIDGE        raised bridge to learned row {tp.EXPECTED_JOIN_ROW}, {tp.BRIDGE_SIM_DURATION_S:.1f} s min-jerk (SIM timing)",
             f"TRANSPORT     learned rows {tp.EXPECTED_JOIN_ROW}..{tp.TRANSPORT_ROWS-1}, original timing x{tp.TRANSPORT_TIME_SCALE}",
             f"ALIGN_INSERT  {len(align['q'])} pts {align['t'][-1]:.2f} s (x{align['multiplier']} of {align['file']})",
             "STOP          before insertion"])
        if not (args.run or args.auto):
            print("PREFLIGHT PASS — NO ROBOT MOVEMENT. Re-run with --run or --auto."); return 0
        if args.run and input("Type START to run the fake pickup + transport: ").strip() != "START":
            raise RuntimeError("Cancelled before motion")

        log = ROOT / "results/simulation/v1_transport" / datetime.now().strftime("%Y%m%d_%H%M%S")
        log.mkdir(parents=True, exist_ok=False)
        summary = {"type": "KINEMATIC SIMULATION (fake hardware) - not a physical result",
                   "planning_frame": frame, "stages": {}}
        ctx = run_stage7(node, traj, transport, align, args, log, summary, poses0)
        a, clearance, within = ctx["alignment"], ctx["clearance"], ctx["within"]
        with (log / "summary.json").open("w") as f:
            json.dump(summary, f, indent=2, default=float)
        print("\nSTOP — V1 held at the insertion-ready pose above fitting_V1 (no insertion).")
        for k, v in summary["checks"].items():
            print(f"  [{'PASS' if v else 'FAIL'}] {k}")
        print("\nALIGNMENT OF V1 AT fitting_V1 (measurement, kinematic simulation):")
        print(f"  lateral offset        {a['lateral_offset_abs_mm']:.3f} mm  {a['lateral_offset_mm']}  (radial clearance 0.25 mm)")
        print(f"  tilt                  {a['tilt_deg']:.3f} deg")
        print(f"  tab/slot yaw error    {a['tab_to_slot_yaw_error_deg']:.3f} deg  (slot allows about +/-1 deg)")
        print(f"  V1 bottom             {a['v1_bottom_above_fitting_base_mm']:.3f} mm above fitting base (top at 7.0 mm)")
        if clearance is not None:
            print(f"  min distance to fitting {clearance:.3f} mm")
        p = summary["alignment_at_stop"]["straight_down_insertion_probe"]
        if p["first_contact_descent_mm"] is None:
            print(f"  straight-down probe   no contact within {p['probe_max_descent_mm']} mm")
        else:
            print(f"  straight-down probe   first contact after {p['first_contact_descent_mm']} mm: {p['first_contact_pairs']}")
        print(f"  within clearance      {'YES' if within else 'NO'}")
        print(f"\nRESULT: {summary['result']} (motion stages)  ->  {log}")
        print("Reset afterwards: python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset")
        return 0 if summary["result"] == "PASS" else 1
    except KeyboardInterrupt:
        print("Interrupted.")
        return 2
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
