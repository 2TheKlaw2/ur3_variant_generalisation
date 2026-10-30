#!/usr/bin/env python3
"""V1 pickup-and-lift on FAKE hardware only (ROS_DOMAIN_ID=73). No physical robot commands.

HOME -> OPEN 40 mm -> REACH -> ALIGN -> GRIP 8.2 mm -> ATTACH V1 -> LIFT 30 mm -> STOP

  ./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh    # terminal 1
  ./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh   # terminal 2 (MoveIt + RViz)
  export ROS_DOMAIN_ID=73
  python3 scripts/phase6_pickup_sim/run_v1_pickup_lift_sim.py          # preflight only, no motion
  python3 scripts/phase6_pickup_sim/run_v1_pickup_lift_sim.py --run    # one START confirmation
  python3 scripts/phase6_pickup_sim/run_v1_pickup_lift_sim.py --auto   # no prompts
  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset    # put V1 back afterwards

LIFT geometry reuses run_v1_lift_bridge_transport_staged.py (physically tested):
+30 mm base Z, constant orientation, MoveIt Cartesian feasibility check through the same
15 waypoints, wrist_3 2*pi correction. Samples: one IK solution per 20 ms timestamp of the min-jerk profile, on the
lab-calibrated model, each verified by MoveIt FK and MoveIt collision checking (V1 attached).
LIFT timing is a NEW 3.0 s minimum-jerk simulation profile (physical: >= 5 s).
Needs the CALIBRATED fake robot + MoveIt launchers above (the upstream launch files ignore
the calibration, so the older start_fake_*.sh scripts run the default UR3e kinematics).
Grasp transform: T(gripper_tcp->V1) = inv(T(base->gripper_tcp)) @ T(base->V1), taken at GRIP
and held fixed. Kinematic simulation only; not a physical result.
"""
import argparse
import csv
import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / "scene"))
import lift_profile as lp  # noqa: E402
import planning_scene_tools as pst  # noqa: E402
import scene_model as sm  # noqa: E402
import sim_node  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import yaml  # noqa: E402

np = sm.np
ROOT = Path(sm.PKG)
JOINTS = lp.JOINTS
T_err = sim_node.T_err
# Same trajectories and multipliers as run_v1_integrated_fake.py
STAGES = {
    "REACH": ("demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv", 2.0, 0.10),
    "ALIGN_PICKUP": ("config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv", 2.0, 0.05),
}
PASS = {  # criteria agreed before the run
    "attach_jump_mm": 0.01, "follow_error_mm": 0.01, "rise_tol_mm": 0.5,
    "xy_drift_mm": 0.5, "tilt_deg": 0.5, "endpoint_joint_rad": 0.002, "others_moved_mm": 1e-6,
}


def load_pickup_trajectories():
    traj = {k: sim_node.load_arm(*v) for k, v in STAGES.items()}
    hand = lp.maxdiff(traj["REACH"]["q"][-1], traj["ALIGN_PICKUP"]["q"][0])
    if hand > 0.010:
        raise RuntimeError(f"REACH->ALIGN handoff {hand:.4f} rad")
    return traj


def preflight(node, title, traj, extra_lines=()):
    """Verify fake hardware, load + verify the planning scene. Returns (world poses, frame)."""
    print(title, flush=True)
    for k, d in traj.items():
        print(f"  {k:13s} {len(d['q']):4d} pts {d['t'][-1]:6.2f} s  (x{d['multiplier']} of {d['file']})")
    print(f"  LIFT          +{lp.LIFT_M*1000:.0f} mm, {lp.N_WAYPOINTS} waypoints, {lp.SIM_LIFT_DURATION_S:.1f} s min-jerk (SIM timing)")
    for line in extra_lines:
        print("  " + line)
    node.guard(); node.fresh()
    print("Fake hardware verified. Loading planning scene...", flush=True)
    node.scene.load_all()
    node.scene.set_allowed("pickup_V1", "pickup_plate", False)
    poses0, frame = node.scene.world_poses()
    objs = sm.scene_objects()
    bad = [k for k in pst.OBJECT_IDS if k not in poses0 or T_err(poses0[k], objs[k]["T"])[0] > 1e-3]
    if bad:
        raise RuntimeError(f"Planning-scene readback mismatch: {bad}")
    print(f"Planning scene: {len(pst.OBJECT_IDS)} objects verified (MoveIt frame '{frame}')", flush=True)
    return poses0, frame


def run_pickup_and_lift(node, traj, args, log, summary, poses0):
    """HOME -> OPEN -> REACH -> ALIGN -> GRIP -> ATTACH -> LIFT. Fills summary; returns context."""
    objs = sm.scene_objects()
    if not args.skip_home:
        print("\nHOME", flush=True)
        subprocess.run([sys.executable, str(ROOT / "scripts/phase1_reach/sim_reach_action.py"), "--home"],
                       cwd=str(ROOT), check=True, input="HOME\n", text=True)
    node.run_gripper("OPEN", sm.RG2_OPEN_M)
    for name in ("REACH", "ALIGN_PICKUP"):
        s, e = node.run_arm(name, traj[name]["t"], traj[name]["q"])
        summary["stages"][name] = {"samples": len(s), "endpoint_err_rad": e,
                                   "timing": f"x{traj[name]['multiplier']} of recorded file (evenly retimed; not CDMP timing)"}
        with (log / f"{name.lower()}_joints.csv").open("w", newline="") as f:
            csv.writer(f).writerows([["t_s"] + JOINTS] + s)
    node.run_gripper("GRIP", sm.RG2_GRIP_M)

    # ---------------- ATTACH: grasp transform fixed at GRIP
    q_grip = node.fresh()
    T_tcp_tf = pst.lookup(node, "base", "gripper_tcp")
    T_tcp_fk = node.fk(q_grip)
    tf_fk = T_err(T_tcp_tf, T_tcp_fk)
    if tf_fk[0] > 0.1:
        raise RuntimeError(f"TF and MoveIt FK disagree by {tf_fk[0]:.3f} mm")
    T_v1 = node.scene.world_poses()[0]["pickup_V1"]
    T_grasp = np.linalg.inv(T_tcp_fk) @ T_v1
    node.scene.attach_v1(T_grasp, objs["pickup_V1"]["mesh"])
    node.scene.set_allowed("pickup_V1", "pickup_plate", True)   # resting contact in the 2 mm pocket
    att = node.scene.attached().get("pickup_V1")
    if att is None or att.link_name != "gripper_tcp":
        raise RuntimeError("V1 not attached to gripper_tcp")
    p = att.object.pose
    T_grasp_rb = sm.pose_to_matrix((p.position.x, p.position.y, p.position.z),
                                   (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
    jump = T_err(T_tcp_fk @ T_grasp_rb, T_v1)
    print(f"ATTACH: grasp xyz {np.round(T_grasp[:3, 3]*1000, 3).tolist()} mm; attach jump {jump[0]:.5f} mm / {jump[1]:.5f} deg", flush=True)
    summary["attach"] = {"T_tcp_to_V1_xyz_mm": (T_grasp[:3, 3] * 1000).round(4).tolist(),
                         "T_tcp_to_V1_quat_xyzw": [round(v, 7) for v in sm.matrix_to_quat(T_grasp[:3, :3])],
                         "tf_vs_fk_mm": tf_fk[0], "jump_mm": jump[0], "jump_deg": jump[1],
                         "touch_links": sm.RG2_TOUCH_LINKS,
                         "acm": "pickup_V1<->pickup_plate allowed during LIFT (resting contact in 2 mm pocket)"}

    # ---------------- LIFT: physical geometry, new 3 s sim timing
    o = T_tcp_fk[:3, 3]; quat = sm.matrix_to_quat(T_tcp_fk[:3, :3])
    poses = [sm.pose_to_matrix(xyz, qq) for xyz, qq in lp.lift_waypoints(o, quat)]
    # (1) physical-script feasibility check: MoveIt Cartesian path through the 15 waypoints
    fraction, n_moveit = node.cartesian(poses, "Cartesian lift")
    # (2) per-sample IK every 0.5 mm, seeded from the previous solution (joint continuity).
    #     Local damped-least-squares IK on the lab-calibrated UR3e model (stays on the seed's
    #     branch); every sample is then checked with MoveIt FK (< 0.01 mm) and MoveIt
    #     collision checking below. (KDL via /compute_ik proved unreliable: 5 ms solver
    #     timeout, branch jumps from random restarts.)
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    # One IK solution per 20 ms timestamp of the min-jerk profile, so the executed
    # trajectory carries the profile itself (no joint-space resampling between samples).
    t_l, frac = lp.min_jerk_grid(lp.SIM_LIFT_DURATION_S, lp.SAMPLE_DT_S)
    raw, seed = [list(q_grip)], list(q_grip)
    for k in range(1, len(t_l)):
        Tk = T_tcp_fk.copy(); Tk[2, 3] += lp.LIFT_M * frac[k]
        q_new = uk.ik_local(seed, Tk, calib).tolist()
        if lp.maxdiff(q_new, seed) > 0.02:
            raise RuntimeError(f"IK branch jump {lp.maxdiff(q_new, seed):.4f} rad at sample {k}")
        seed = q_new; raw.append(seed)
    path, wraps = lp.correct_wraps(raw, q_grip)
    fk_dev = []
    for k, qi in enumerate(path):
        Tk = node.fk(qi); target = o[2] + lp.LIFT_M * frac[k]
        fk_dev.append(max(abs(Tk[2, 3] - target), math.hypot(*(Tk[:2, 3] - o[:2]))) * 1000)
    if max(fk_dev) > 0.01:
        raise RuntimeError(f"IK samples deviate from the min-jerk straight lift by {max(fk_dev):.4f} mm")
    q_l = np.array(path)
    step, speed = lp.verify_path("LIFT", q_l.tolist(), t_l.tolist())
    print(f"LIFT planned: MoveIt Cartesian fraction {fraction:.4f} ({n_moveit} pts); {len(path)} timed IK samples "
          f"every {lp.SAMPLE_DT_S*1000:.0f} ms (max FK deviation {max(fk_dev):.5f} mm), {t_l[-1]:.2f} s, "
          f"peak step {step:.4f} rad, peak speed {speed:.4f} rad/s, 2pi turns {wraps}", flush=True)
    invalid = []
    for i in range(0, len(q_l)):
        ok, contacts = node.valid(q_l[i])
        if not ok:
            invalid.append((round(float(t_l[i]), 3), contacts))
    if invalid:
        raise RuntimeError(f"LIFT collision/invalid states (attached V1 included): {invalid[:3]}")
    lift_samples, lift_err = node.run_arm("LIFT", t_l.tolist(), q_l.tolist())
    node.scene.set_allowed("pickup_V1", "pickup_plate", False)   # restore normal checking
    ok_end, contacts_end = node.valid(node.fresh())

    # ---------------- STOP: evaluate executed lift from recorded joint states
    rows, rises, drifts, tilts = [], [], [], []
    att2 = node.scene.attached()["pickup_V1"].object.pose
    T_grasp_end = sm.pose_to_matrix((att2.position.x, att2.position.y, att2.position.z),
                                    (att2.orientation.x, att2.orientation.y, att2.orientation.z, att2.orientation.w))
    for s in lift_samples[::2] + [lift_samples[-1]]:
        Tt = node.fk(s[1:]); Tv = Tt @ T_grasp_rb
        dv = Tv[:3, 3] - T_v1[:3, 3]; tilt = T_err(Tv, T_v1)[1]
        rises.append(dv[2] * 1000); drifts.append(math.hypot(dv[0], dv[1]) * 1000); tilts.append(tilt)
        rows.append([round(s[0], 4), *s[1:], *(Tt[:3, 3]), *(Tv[:3, 3]), dv[2] * 1000, drifts[-1], tilt])
    follow_err = T_err(T_grasp_end, T_grasp_rb)[0]
    tcp_end = pst.lookup(node, "base", "gripper_tcp") @ T_grasp_end
    tf_rise = (tcp_end[2, 3] - T_v1[2, 3]) * 1000
    poses1, _ = node.scene.world_poses()
    moved = max(T_err(poses1[k], poses0[k])[0] for k in pst.OBJECT_IDS if k != "pickup_V1")
    checks = {
        "attach jump < 0.01 mm": jump[0] < PASS["attach_jump_mm"],
        "grasp transform unchanged through LIFT (< 0.01 mm)": follow_err < PASS["follow_error_mm"],
        "V1 rise 30 +/- 0.5 mm (FK of executed joints)": abs(rises[-1] - 30) < PASS["rise_tol_mm"],
        "V1 rise 30 +/- 0.5 mm (TF at STOP)": abs(tf_rise - 30) < PASS["rise_tol_mm"],
        "V1 xy drift < 0.5 mm": max(drifts) < PASS["xy_drift_mm"],
        "V1 tilt < 0.5 deg": max(tilts) < PASS["tilt_deg"],
        "LIFT endpoint joint error < 0.002 rad": lift_err < PASS["endpoint_joint_rad"],
        "V2-V5, plates, fittings unchanged": moved < PASS["others_moved_mm"],
        "no planned-state collisions (every timed sample)": not invalid,
        "final state valid with V1<->plate checked again": ok_end,
    }
    summary["lift"] = {
        "geometry_source": "run_v1_lift_bridge_transport_staged.py (+30 mm base Z, constant orientation, 15 waypoints)",
        "timing": {"profile": "minimum-jerk on path length", "sim_duration_s": lp.SIM_LIFT_DURATION_S,
                   "physical_reference": "MoveIt-timed, >= 5 s (not comparable)"},
        "moveit_cartesian_fraction": fraction, "moveit_cartesian_points": n_moveit, "ik_samples": len(path), "ik_sample_dt_s": lp.SAMPLE_DT_S, "ik_max_fk_deviation_mm": max(fk_dev), "samples": int(len(t_l)), "wrist_wraps": wraps,
        "peak_joint_step_rad": step, "peak_joint_speed_rad_s": speed,
        "V1_rise_mm_fk": rises[-1], "V1_rise_mm_tf": tf_rise, "V1_max_xy_drift_mm": max(drifts),
        "V1_max_tilt_deg": max(tilts), "endpoint_joint_err_rad": lift_err,
        "others_max_move_mm": moved, "final_state_contacts": contacts_end,
    }
    checks = {k: bool(v) for k, v in checks.items()}
    summary["checks"] = checks
    summary["result"] = "PASS" if all(checks.values()) else "FAIL"
    with (log / "lift_executed.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", *JOINTS, "tcp_x", "tcp_y", "tcp_z", "v1_x", "v1_y", "v1_z", "v1_rise_mm", "v1_xy_drift_mm", "v1_tilt_deg"])
        w.writerows(rows)
    with (log / "lift_planned.csv").open("w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", *JOINTS]); w.writerows([[float(a), *map(float, b)] for a, b in zip(t_l, q_l)])
    return {"T_grasp": T_grasp_rb, "T_v1_pick": T_v1, "T_tcp_grip": T_tcp_fk, "q_grip": q_grip,
            "q_lift_end": q_l[-1].tolist(), "calib": calib, "objs": objs}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="Run with one START confirmation")
    ap.add_argument("--auto", action="store_true", help="Run without prompts (fake hardware only)")
    ap.add_argument("--skip-home", action="store_true", help="Robot already at taught home")
    sim_node.add_speed_arg(ap)
    args = ap.parse_args()
    pst.require_sim_domain()
    traj = load_pickup_trajectories()
    rclpy, node = sim_node.make_node("v1_pickup_lift_SIM_ONLY")
    sim_node.set_speed(node, args.speed, args.max_joint_speed)
    try:
        poses0, frame = preflight(node, "V1 PICKUP + LIFT — FAKE HARDWARE ONLY (ROS_DOMAIN_ID=73)", traj)
        if not (args.run or args.auto):
            print("PREFLIGHT PASS — NO ROBOT MOVEMENT. Re-run with --run or --auto."); return 0
        if args.run and input("Type START to run the fake pickup + lift: ").strip() != "START":
            raise RuntimeError("Cancelled before motion")
        log = ROOT / "results/simulation/v1_pickup_lift" / datetime.now().strftime("%Y%m%d_%H%M%S")
        log.mkdir(parents=True, exist_ok=False)
        summary = {"type": "KINEMATIC SIMULATION (fake hardware) - not a physical result",
                   "planning_frame": frame, "stages": {}}
        run_pickup_and_lift(node, traj, args, log, summary, poses0)
        with (log / "summary.json").open("w") as f:
            json.dump(summary, f, indent=2, default=float)
        print("\nSTOP — V1 held 30 mm above the pickup plate.")
        for k, v in summary["checks"].items():
            print(f"  [{'PASS' if v else 'FAIL'}] {k}")
        print(f"\nRESULT: {summary['result']}  ->  {log}")
        print("Reset afterwards: python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset")
        return 0 if summary["result"] == "PASS" else 1
    except KeyboardInterrupt:
        print("Interrupted.")
        return 2
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
