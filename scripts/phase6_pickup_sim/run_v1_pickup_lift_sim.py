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
15 waypoints, wrist_3 2*pi correction. Samples: per-sample IK every 0.5 mm on the
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
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / "scene"))
import lift_profile as lp  # noqa: E402
import planning_scene_tools as pst  # noqa: E402
import scene_model as sm  # noqa: E402
import ur_kinematics as uk  # noqa: E402
import yaml  # noqa: E402

np = sm.np
ROOT = Path(sm.PKG)
JOINTS = lp.JOINTS
# Same trajectories and multipliers as run_v1_integrated_fake.py
STAGES = {
    "REACH": ("demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv", 2.0, 0.10),
    "ALIGN_PICKUP": ("config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv", 2.0, 0.05),
}
PASS = {  # criteria agreed before the run
    "attach_jump_mm": 0.01, "follow_error_mm": 0.01, "rise_tol_mm": 0.5,
    "xy_drift_mm": 0.5, "tilt_deg": 0.5, "endpoint_joint_rad": 0.002, "others_moved_mm": 1e-6,
}


def load_arm(rel, multiplier, max_step):
    p = ROOT / rel
    with p.open(newline="") as f:
        r = csv.DictReader(f)
        if not set(["time"] + JOINTS).issubset(r.fieldnames or []):
            raise RuntimeError(f"Unexpected CSV columns: {p}")
        rows = list(r)
    t0 = float(rows[0]["time"])
    t = [(float(x["time"]) - t0) * multiplier for x in rows]
    q = [[float(x[j]) for j in JOINTS] for x in rows]
    step = max(lp.maxdiff(a, b) for a, b in zip(q, q[1:]))
    speed = max(lp.maxdiff(a, b) / (tb - ta) for a, b, ta, tb in zip(q, q[1:], t, t[1:]))
    if any(b <= a for a, b in zip(t, t[1:])) or step > max_step or speed > 0.5:
        raise RuntimeError(f"Trajectory guard failed: {p}")
    return {"t": t, "q": q, "file": rel, "multiplier": multiplier}


def T_err(A, B):
    dt = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
    ang = math.degrees(math.acos(np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1)))
    return dt, ang


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true", help="Run with one START confirmation")
    ap.add_argument("--auto", action="store_true", help="Run without prompts (fake hardware only)")
    ap.add_argument("--skip-home", action="store_true", help="Robot already at taught home")
    args = ap.parse_args()
    pst.require_sim_domain()

    traj = {k: load_arm(*v) for k, v in STAGES.items()}
    hand = lp.maxdiff(traj["REACH"]["q"][-1], traj["ALIGN_PICKUP"]["q"][0])
    if hand > 0.010:
        raise RuntimeError(f"REACH->ALIGN handoff {hand:.4f} rad")

    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from rclpy.node import Node

    class PickupSim(Node):
        def __init__(self):
            super().__init__("v1_pickup_lift_SIM_ONLY")
            self.q = None; self.msg = None; self.width = None; self.q_at = 0.0
            self.rec = False; self.samples = []; self.t0 = 0.0
            self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)
            self.hw = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
            self.ctl = self.create_client(ListControllers, "/controller_manager/list_controllers")
            self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
            self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
            self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
            self.arm = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
            self.grip = ActionClient(self, FollowJointTrajectory, "/finger_width_trajectory_controller/follow_joint_trajectory")
            self.scene = pst.SceneClient(self)

        def on_js(self, msg):
            if "finger_width" in msg.name:
                self.width = float(msg.position[msg.name.index("finger_width")])
            if all(j in msg.name for j in JOINTS):
                self.q = [float(msg.position[msg.name.index(j)]) for j in JOINTS]
                self.msg = msg; self.q_at = time.monotonic()
                if self.rec:
                    self.samples.append([time.monotonic() - self.t0] + self.q)

        def spin_until(self, fn, timeout, label):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if fn():
                    return
            raise RuntimeError("Timeout: " + label)

        def rpc(self, cli, req, label, timeout=15):
            if not cli.wait_for_service(timeout_sec=6):
                raise RuntimeError("Missing service: " + label)
            fut = cli.call_async(req)
            self.spin_until(fut.done, timeout, label)
            if fut.result() is None:
                raise RuntimeError(label + " failed")
            return fut.result()

        def fresh(self):
            t = time.monotonic()
            self.spin_until(lambda: self.q_at > t and self.width is not None, 5, "joint states")
            return self.q[:]

        def guard(self):
            comps = self.rpc(self.hw, ListHardwareComponents.Request(), "hardware list").component
            actual = {(h.name, h.class_type) for h in comps}
            expected = {("ur3e", "mock_components/GenericSystem"),
                        ("OnRobotGripper", "test_hardware_components/TestSingleJointActuator")}
            if actual != expected:
                raise RuntimeError(f"BLOCKED: not the fake UR3e/RG2 hardware: {actual}")
            st = {c.name: c.state.lower() for c in self.rpc(self.ctl, ListControllers.Request(), "controllers").controller}
            if st.get("joint_trajectory_controller") != "active" or st.get("finger_width_trajectory_controller") != "active":
                raise RuntimeError("Fake arm/gripper controller inactive")
            if st.get("scaled_joint_trajectory_controller") == "active":
                raise RuntimeError("BLOCKED: scaled (physical) controller active")

        def width_target(self, m):
            # RG2 description: finger_width is prismatic in metres, limits 0..0.110.
            # The fake actuator starts at 0.0, so 0 is accepted as metres.
            if 0.0 <= self.width <= 0.120: return m, 0.0015
            if 5 <= self.width <= 120: return m * 1000, 1.5
            raise RuntimeError(f"Unknown gripper units: {self.width}")

        def send(self, client, jt, timeout):
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError("Trajectory action unavailable")
            goal = FollowJointTrajectory.Goal(); goal.trajectory = jt
            goal.goal_time_tolerance = Duration(seconds=2).to_msg()
            sent = client.send_goal_async(goal); self.spin_until(sent.done, 5, "goal response")
            if sent.result() is None or not sent.result().accepted:
                raise RuntimeError("Trajectory goal rejected")
            res = sent.result().get_result_async(); self.spin_until(res.done, timeout, "execution")
            r = res.result()
            if r is None or r.status != GoalStatus.STATUS_SUCCEEDED or r.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError("Controller reported execution failure")

        def run_arm(self, name, t, q, start_tol=0.005, end_tol=0.002):
            self.guard(); live = self.fresh()
            if lp.maxdiff(live, q[0]) > start_tol:
                raise RuntimeError(f"{name}: start mismatch {lp.maxdiff(live, q[0]):.5f} rad")
            jt = JointTrajectory(); jt.joint_names = list(JOINTS)
            for ti, qi in zip(t, q):
                p = JointTrajectoryPoint(); p.positions = [float(v) for v in qi]
                p.time_from_start = Duration(seconds=float(ti)).to_msg(); jt.points.append(p)
            self.samples = []; self.t0 = time.monotonic(); self.rec = True
            try:
                self.send(self.arm, jt, t[-1] + 10)
            finally:
                self.rec = False
            err = lp.maxdiff(self.fresh(), q[-1])
            if err > end_tol:
                raise RuntimeError(f"{name}: endpoint error {err:.5f} rad")
            print(f"{name}: PASS ({len(q)} pts, {t[-1]:.2f} s, endpoint err {err:.6f} rad)", flush=True)
            return list(self.samples), err

        def run_gripper(self, name, width_m):
            self.guard(); self.fresh()
            target, tol = self.width_target(width_m)
            jt = JointTrajectory(); jt.joint_names = ["finger_width"]
            p = JointTrajectoryPoint(); p.positions = [target]; p.time_from_start = Duration(seconds=3).to_msg()
            jt.points = [p]; self.send(self.grip, jt, 8); self.fresh()
            if abs(self.width - target) > tol:
                raise RuntimeError(f"{name}: gripper did not reach {width_m*1000:.1f} mm")
            print(f"{name}: PASS (RG2 {width_m*1000:.1f} mm)", flush=True)

        def fk(self, q):
            req = GetPositionFK.Request(); req.header.frame_id = "base"; req.fk_link_names = ["gripper_tcp"]
            req.robot_state = RobotState(); req.robot_state.joint_state.name = list(JOINTS)
            req.robot_state.joint_state.position = [float(v) for v in q]
            r = self.rpc(self.fk_cli, req, "MoveIt FK", 6)
            if r.error_code.val != 1 or len(r.pose_stamped) != 1:
                raise RuntimeError("MoveIt FK failed")
            p = r.pose_stamped[0].pose
            return sm.pose_to_matrix((p.position.x, p.position.y, p.position.z),
                                     (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

        def cartesian(self, poses):
            req = GetCartesianPath.Request(); req.header.frame_id = "base"
            req.group_name = "ur_onrobot_manipulator"; req.link_name = "gripper_tcp"
            req.start_state = RobotState(); req.start_state.joint_state = self.msg; req.start_state.is_diff = True
            req.waypoints = poses; req.max_step = 0.002; req.jump_threshold = 0.0   # as physical script
            req.avoid_collisions = True
            for f, v in (("max_velocity_scaling_factor", 0.03), ("max_acceleration_scaling_factor", 0.03)):
                if hasattr(req, f):  # newer moveit_msgs only (physical script used 0.03)
                    setattr(req, f, v)
            r = self.rpc(self.cart_cli, req, "Cartesian lift", 30)
            jt = r.solution.joint_trajectory
            if r.error_code.val != 1 or r.fraction < 0.999 or len(jt.points) < 2:
                raise RuntimeError(f"LIFT plan incomplete: fraction={r.fraction:.4f}, error={r.error_code.val}")
            return r.fraction, len(jt.points)

        def valid(self, q):
            req = GetStateValidity.Request(); req.group_name = "ur_onrobot_manipulator"
            req.robot_state = RobotState(); req.robot_state.is_diff = True  # keep attached V1
            req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = [float(v) for v in q]
            r = self.rpc(self.valid_cli, req, "state validity", 6)
            return r.valid, [f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts]

    # ------------------------------------------------------------------ run
    rclpy.init()
    node = PickupSim()
    log = None
    try:
        print("V1 PICKUP + LIFT — FAKE HARDWARE ONLY (ROS_DOMAIN_ID=73)", flush=True)
        for k, d in traj.items():
            print(f"  {k:13s} {len(d['q']):4d} pts {d['t'][-1]:6.2f} s  (x{d['multiplier']} of {d['file']})")
        print(f"  LIFT          +{lp.LIFT_M*1000:.0f} mm, {lp.N_WAYPOINTS} waypoints, {lp.SIM_LIFT_DURATION_S:.1f} s min-jerk (SIM timing)")
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
        if not (args.run or args.auto):
            print("PREFLIGHT PASS — NO ROBOT MOVEMENT. Re-run with --run or --auto."); return 0
        if args.run and input("Type START to run the fake pickup + lift: ").strip() != "START":
            raise RuntimeError("Cancelled before motion")

        log = ROOT / "results/simulation/v1_pickup_lift" / datetime.now().strftime("%Y%m%d_%H%M%S")
        log.mkdir(parents=True, exist_ok=False)
        summary = {"type": "KINEMATIC SIMULATION (fake hardware) - not a physical result",
                   "planning_frame": frame, "stages": {}}

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
        o = T_tcp_fk[:3, 3]; quat = sm.matrix_to_quat(T_tcp_fk[:3, :3]); poses = []
        for xyz, qq in lp.lift_waypoints(o, quat):
            ps = Pose(); ps.position.x, ps.position.y, ps.position.z = (float(v) for v in xyz)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = (float(v) for v in qq); poses.append(ps)
        # (1) physical-script feasibility check: MoveIt Cartesian path through the 15 waypoints
        fraction, n_moveit = node.cartesian(poses)
        # (2) per-sample IK every 0.5 mm, seeded from the previous solution (joint continuity).
        #     Local damped-least-squares IK on the lab-calibrated UR3e model (stays on the seed's
        #     branch); every sample is then checked with MoveIt FK (< 0.01 mm) and MoveIt
        #     collision checking below. (KDL via /compute_ik proved unreliable: 5 ms solver
        #     timeout, branch jumps from random restarts.)
        calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
        n_s = int(round(lp.LIFT_M / lp.CARTESIAN_MAX_STEP_M))
        raw, seed = [list(q_grip)], list(q_grip)
        for k in range(1, n_s + 1):
            Tk = T_tcp_fk.copy(); Tk[2, 3] += lp.LIFT_M * k / n_s
            q_new = uk.ik_local(seed, Tk, calib).tolist()
            if lp.maxdiff(q_new, seed) > 0.02:
                raise RuntimeError(f"IK branch jump {lp.maxdiff(q_new, seed):.4f} rad at sample {k}")
            seed = q_new; raw.append(seed)
        path, wraps = lp.correct_wraps(raw, q_grip)
        fk_dev = []
        for k, qi in enumerate(path):
            Tk = node.fk(qi); target = o[2] + lp.LIFT_M * k / n_s
            fk_dev.append(max(abs(Tk[2, 3] - target), math.hypot(*(Tk[:2, 3] - o[:2]))) * 1000)
        if max(fk_dev) > 0.01:
            raise RuntimeError(f"IK samples deviate from the straight lift by {max(fk_dev):.4f} mm")
        z = np.arange(n_s + 1) * lp.LIFT_M / n_s
        t_raw = lp.min_jerk_times(z / lp.LIFT_M)
        t_l, q_l = lp.resample_uniform(t_raw, path, 0.02)
        step, speed = lp.verify_path("LIFT", q_l.tolist(), t_l.tolist())
        print(f"LIFT planned: MoveIt Cartesian fraction {fraction:.4f} ({n_moveit} pts); {len(path)} IK samples "
              f"(max FK deviation {max(fk_dev):.5f} mm) -> {len(t_l)} timed samples, {t_l[-1]:.2f} s, "
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
            "moveit_cartesian_fraction": fraction, "moveit_cartesian_points": n_moveit, "ik_samples": len(path), "ik_max_fk_deviation_mm": max(fk_dev), "samples": int(len(t_l)), "wrist_wraps": wraps,
            "peak_joint_step_rad": step, "peak_joint_speed_rad_s": speed,
            "V1_rise_mm_fk": rises[-1], "V1_rise_mm_tf": tf_rise, "V1_max_xy_drift_mm": max(drifts),
            "V1_max_tilt_deg": max(tilts), "endpoint_joint_err_rad": lift_err,
            "others_max_move_mm": moved, "final_state_contacts": contacts_end,
        }
        summary["checks"] = checks
        summary["result"] = "PASS" if all(checks.values()) else "FAIL"
        with (log / "lift_executed.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", *JOINTS, "tcp_x", "tcp_y", "tcp_z", "v1_x", "v1_y", "v1_z", "v1_rise_mm", "v1_xy_drift_mm", "v1_tilt_deg"])
            w.writerows(rows)
        with (log / "lift_planned.csv").open("w", newline="") as f:
            w = csv.writer(f); w.writerow(["t_s", *JOINTS]); w.writerows([[float(a), *map(float, b)] for a, b in zip(t_l, q_l)])
        with (log / "summary.json").open("w") as f:
            json.dump(summary, f, indent=2, default=float)
        print("\nSTOP — V1 held 30 mm above the pickup plate.")
        for k, v in checks.items():
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
