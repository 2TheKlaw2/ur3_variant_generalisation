#!/usr/bin/env python3
"""STAGED physical run of one variant's full demo-driven cycle on the REAL UR3e + RG2.

  python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1                   # preflight only (no ROS)
  python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1 --until ALIGN_INSERT --execute
  python3 scripts/phase11_physical/run_full_cycle_physical.py --variant 1 --from INSERT --execute

Default is an OFFLINE PREFLIGHT: no ROS, no motion. With --execute (ROS_DOMAIN_ID=0, real hardware):
  * one variant per run; every stage needs the operator to type RUN_<STAGE> (no auto mode);
  * the real UR driver + real RG2 are verified, mock hardware is rejected (as Phase 5);
  * before each stage the LIVE joints must match the planned start (<= 0.006 rad), the first
    point is replaced by the fresh joints, and the endpoint is checked (<= 0.012 rad);
  * every stage is retimed uniformly so no joint exceeds --max-joint-speed (default 0.14 rad/s,
    the physical-pilot guard; hard cap 0.35); the path itself is never changed;
  * MoveIt /check_state_validity is called along every stage (the variant scene must be loaded in
    MoveIt, with the part attached while it is held), and the run stops on any invalid state;
  * Ctrl+C cancels the active controller goal and waits for the controller's acknowledgement
    (NOT an E-stop: keep the pendant stop in reach);
  * joint states, gripper width and stage results are logged to results/physical/full_cycle/<time>/.
Plans: Stage 10 REACH/TRANSPORT + RELEASE/RETREAT/HOME from --plans (default stage10_current or the
31 Oct reference) and Stage 9 INSERT/ROTATE (L2) from the same cache the fake-hardware runner uses.
Run the same variant on fake hardware first (run_variant_insert_sim.py --full) after any calibration.
The ROS/controller handling is adapted from scripts/phase5_integration/run_v1_physical_sequence.py.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
JOINTS = [f"{x}_joint" for x in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
ARM_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/finger_width_trajectory_controller/follow_joint_trajectory"
RG2_OPEN_M, RG2_GRIP_M = 0.040, 0.0082
RG2_OPEN_S, RG2_GRIP_S = 3.0, 2.0
START_TOL, END_TOL = 0.006, 0.012
HARD_MAX_SPEED = 0.35            # rad/s, never exceeded whatever --max-joint-speed says
MAX_POINT_STEP = 0.09            # rad between consecutive points (as Phase 5)
ORDER = ["HOME_START", "OPEN", "REACH", "ALIGN_GRIP", "GRIP", "LIFT", "TRANSPORT", "ALIGN_INSERT",
         "INSERT", "ROTATE", "RELEASE", "RETREAT", "HOME"]
HELD = {"LIFT", "TRANSPORT", "ALIGN_INSERT", "INSERT", "ROTATE"}
HOME_Q = [-0.675926510488, -1.34777768076, -0.527696251869, -2.8273965321, 1.55948805809, -3.87329799334]


def err(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


# ------------------------------------------------------------------ plans (no ROS)
def default_plans():
    cur = ROOT / "results/simulation/stage10_current"
    return cur if cur.exists() else ROOT / "results/sim_reference/stage10_retarget_20261031_zscale"


def read_stages(path, extra_cols=0):
    out = {}
    for r in csv.DictReader(open(path)):
        t, q = out.setdefault(r["stage"], ([], []))
        t.append(float(r["t_s"])); q.append([float(r[j]) for j in JOINTS])
    return out


def load_insert_rotate(i):
    """Stage 9 L2 INSERT/ROTATE for variant i, from the fake-hardware runner's plan cache (identical plan)."""
    sys.path[:0] = [str(ROOT / "scripts" / d) for d in ("phase9_variants", "phase8_insert_sim", "phase7_transport_sim",
                                                        "phase6_pickup_sim", "scene")]
    import run_variant_insert_sim as rvi
    import run_variants_offline as rvo
    import scene_model as sm
    import variant_core as vc
    import yaml
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    objs = sm.scene_objects(); G, _ = rvo.grasps(objs, calib)
    prof = vc.demo_profiles(str(ROOT)); q1, T1, z1, yaw0 = rvo.v1_start(objs, calib, G[1])
    p = rvi.plan_cached(i, "L2", calib, objs, G, prof, q1, T1, z1, yaw0, str(ROOT / "results/cad_features"))
    res = p["res"]
    if res["functional_outcome"] != "PASS":
        raise RuntimeError(f"V{i} L2 plan does not assemble in simulation ({res['functional_outcome']}); not executable")
    feas = [r for r in res["rows"] if r["feasible"]]
    st = {}
    for name in ("INSERT", "ROTATE"):
        seg = [r for r in feas if r["stage"] == name]
        st[name] = ([r["t_s"] - seg[0]["t_s"] for r in seg], [list(r["q"]) for r in seg])
    return st, G[i], objs


def retime(t, q, vmax):
    """Uniform time scaling so the peak joint speed is <= vmax. The joint path is unchanged."""
    peak = max((max(abs(b[j] - a[j]) for j in range(6)) / (t1 - t0))
               for (t0, a), (t1, b) in zip(zip(t, q), zip(t[1:], q[1:])) if t1 > t0) if len(t) > 1 else 0.0
    f = max(1.0, peak / vmax) if peak > 0 else 1.0
    return [x * f for x in t], f, peak


def build(i, plans_dir, vmax):
    full = read_stages(plans_dir / f"V{i}" / "joint_trajectory.csv")
    rel = read_stages(plans_dir / f"V{i}" / "release_retreat.csv")
    s10 = json.load(open(plans_dir / "stage10_summary.json"))["variants"].get(f"V{i}", {})
    if not s10.get("PASS"):
        raise RuntimeError(f"V{i}: Stage 10 plan in {plans_dir} did not pass; not executable")
    if "ALIGN_INSERT" not in full:
        raise RuntimeError(f"V{i}: Stage 10 plan stops early; not executable")
    ir, G, objs = load_insert_rotate(i)
    seq = {}
    for name in ("REACH", "ALIGN_GRIP", "LIFT", "TRANSPORT", "ALIGN_INSERT"):
        seq[name] = full[name]
    seq.update(ir)
    seq["RETREAT"] = rel["RETREAT"]; seq["HOME"] = rel["HOME"]
    # wrist_3 2pi bookkeeping: keep every stage on the same unwrapped branch as HOME
    k = round((HOME_Q[5] - seq["REACH"][1][0][5]) / (2 * math.pi))
    for name, (t, q) in seq.items():
        for row in q:
            row[5] += 2 * math.pi * k
    out, prev = {}, ("HOME", HOME_Q)
    for name in ["REACH", "ALIGN_GRIP", "LIFT", "TRANSPORT", "ALIGN_INSERT", "INSERT", "ROTATE", "RETREAT", "HOME"]:
        t, q = seq[name]
        gap = err(prev[1] if name != "RETREAT" else seq["ROTATE"][1][-1], q[0])
        if gap > START_TOL:
            raise RuntimeError(f"V{i}: hand-over {prev[0]} -> {name} is {gap:.5f} rad (> {START_TOL})")
        step = max(err(a, b) for a, b in zip(q, q[1:]))
        if step > MAX_POINT_STEP:
            raise RuntimeError(f"V{i} {name}: point step {step:.4f} rad > {MAX_POINT_STEP}")
        if any(abs(v) > 2 * math.pi + 1e-3 for row in q for v in row[:5]) or any(abs(row[2]) > math.radians(160) for row in q):
            raise RuntimeError(f"V{i} {name}: joint outside the software limits (+-2pi; elbow +-160 deg)")
        t2, f, peak = retime(t, q, vmax)
        out[name] = {"t": t2, "q": q, "slowdown": f, "plan_peak_rad_s": peak, "duration_s": t2[-1]}
        prev = (name, q[-1])
    return out, G, objs, s10


def preflight(i, plans_dir, vmax):
    plans, G, objs, s10 = build(i, plans_dir, vmax)
    print(f"\n=========== V{i} FULL-CYCLE PHYSICAL PREFLIGHT (no ROS, no motion) ===========")
    print(f"plans: {plans_dir}  (Stage 10 z-scale {s10.get('transport_z_scale')}, sigma_min {s10.get('transport_sigma_min')})")
    print(f"speed guard: every stage retimed to <= {vmax} rad/s per joint (path unchanged)\n")
    print(f"{'stage':13s} {'points':>6s} {'planned peak':>12s} {'slowdown':>8s} {'duration':>9s}")
    total = 0.0
    for name, p in plans.items():
        print(f"{name:13s} {len(p['q']):6d} {p['plan_peak_rad_s']:10.3f}/s {p['slowdown']:7.2f}x {p['duration_s']:8.1f}s")
        total += p["duration_s"]
    print(f"{'':13s} {'':6s} {'':12s} {'':8s} {total:8.1f}s  arm motion (+ gripper, + operator reviews)")
    print("\nAll hand-overs <= 0.006 rad, point steps <= 0.09 rad, joints within +-2pi and elbow +-160 deg.")
    print("PREFLIGHT PASS. Simulation only proves the CAD model; the real fixture must be inspected at every stage.")
    return plans, G, objs


def home_move(q0, vmax):
    """Joint-space minimum-jerk move to HOME with peak joint speed vmax (1.875 = min-jerk peak factor)."""
    d = err(q0, HOME_Q); T = max(5.0, 1.875 * d / vmax)
    n = int(math.ceil(T / 0.04)); t, q = [], []
    for k in range(n + 1):
        s = k / n; f = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        t.append(T * s); q.append([a + (b - a) * f for a, b in zip(q0, HOME_Q)])
    return {"t": t, "q": q, "slowdown": 1.0, "plan_peak_rad_s": vmax, "duration_s": T}


# ------------------------------------------------------------------ physical execution
def execute(i, plans, G, objs, args):
    import yaml
    reg = yaml.safe_load(open(ROOT / "config/scene/scene_registration.yaml"))
    s10 = json.load(open(args.plans / "stage10_summary.json"))
    if reg.get("layout") != "session" or s10.get("layout") != "session":
        raise SystemExit("BLOCKED: the plate registration / plans are not from a lab calibration session "
                         f"(registration layout '{reg.get('layout')}', plans layout '{s10.get('layout')}'). "
                         "Run CALIBRATE_PLATES.sh and UPDATE_SIM.sh first.")
    if os.environ.get("ROS_DOMAIN_ID", "0") != "0":
        raise SystemExit("BLOCKED: physical execution requires ROS_DOMAIN_ID=0 (the real robot), "
                         f"not {os.environ.get('ROS_DOMAIN_ID')}")
    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    sys.path.insert(0, str(ROOT / "scripts/phase6_pickup_sim"))
    import planning_scene_tools as pst

    class Robot(Node):
        def __init__(self):
            super().__init__("full_cycle_PHYSICAL")
            self.q = None; self.width = None; self.last_at = 0.0; self.width_at = 0.0
            self.monitor = False; self.records = []; self.stage = ""
            self.stop_requested = False; self.goal_inflight = False
            self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)
            self.hw = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
            self.ctrl = self.create_client(ListControllers, "/controller_manager/list_controllers")
            self.valid_srv = self.create_client(GetStateValidity, "/check_state_validity")
            self.arm = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
            self.grip = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION)

        def on_js(self, msg):
            if len(msg.name) != len(msg.position):
                return
            now = time.monotonic()
            if "finger_width" in msg.name:
                w = float(msg.position[msg.name.index("finger_width")])
                if math.isfinite(w):
                    self.width, self.width_at = w, now
            if not all(j in msg.name for j in JOINTS):
                return
            q = [float(msg.position[msg.name.index(j)]) for j in JOINTS]
            if all(math.isfinite(x) for x in q):
                self.q, self.last_at = q, now
                if self.monitor:
                    self.records.append([datetime.now().isoformat(timespec="milliseconds"), self.stage] + q + [self.width])

        def fresh(self, sec=5):
            after = time.monotonic(); deadline = after + sec
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.12)
                if self.q is not None and self.last_at >= after:
                    return list(self.q)
            raise RuntimeError("No NEW live joint state; no command sent")

        def request(self, client, req, timeout=8):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("Required ROS service unavailable")
            fut = client.call_async(req); deadline = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not fut.done() or fut.result() is None:
                raise RuntimeError("ROS service request timed out")
            return fut.result()

        def hardware(self):
            hw = self.request(self.hw, ListHardwareComponents.Request()).component
            types = [h.class_type for h in hw]
            if not any(t == "ur_robot_driver/URPositionHardwareInterface" for t in types):
                raise RuntimeError("Real URPositionHardwareInterface not verified")
            if not any(h.name == "OnRobotGripper" and h.class_type == "onrobot_driver/RGHardwareInterface" for h in hw):
                raise RuntimeError("Real OnRobot RG2 not verified")
            if any("mock_components" in t or "test_hardware_components" in t for t in types):
                raise RuntimeError("Mock hardware present: REJECTED")
            states = {c.name: c.state.lower() for c in self.request(self.ctrl, ListControllers.Request()).controller}
            if states.get("scaled_joint_trajectory_controller") != "active":
                raise RuntimeError("Scaled arm controller is not active")
            if states.get("joint_trajectory_controller") == "active":
                raise RuntimeError("Another arm position controller is active")
            if states.get("finger_width_trajectory_controller") != "active":
                raise RuntimeError("RG2 controller not active")
            if not self.arm.wait_for_server(timeout_sec=5) or not self.grip.wait_for_server(timeout_sec=5):
                raise RuntimeError("Arm or gripper action unavailable")

        def check_states(self, name, q_list, every=10):
            if not self.valid_srv.wait_for_service(timeout_sec=3):
                raise RuntimeError("MoveIt /check_state_validity unavailable: start MoveIt for the real robot")
            idx = sorted(set(list(range(0, len(q_list), every)) + [len(q_list) - 1]))
            for k in idx:
                req = GetStateValidity.Request(); req.group_name = "ur_onrobot_manipulator"
                req.robot_state = RobotState(); req.robot_state.joint_state.name = JOINTS
                req.robot_state.joint_state.position = list(q_list[k])
                r = self.request(self.valid_srv, req, timeout=5)
                if not r.valid:
                    c = [f"{x.contact_body_1}<->{x.contact_body_2}" for x in r.contacts][:3]
                    raise RuntimeError(f"{name}: MoveIt state INVALID at point {k}/{len(q_list) - 1} {c}")
            print(f"{name}: {len(idx)} states valid in MoveIt", flush=True)

        def cancel(self, handle, label, result_future):
            print(f"\n{label}: REQUESTING CONTROLLER CANCELLATION (not an E-stop).", file=sys.stderr, flush=True)
            try:
                cf = handle.cancel_goal_async(); until = time.monotonic() + 6
                while not cf.done() and time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.1)
                until = time.monotonic() + 8
                while not result_future.done() and time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.1)
                ok = result_future.done() and result_future.result() is not None and \
                    result_future.result().status == GoalStatus.STATUS_CANCELED
                print(f"{label}: {'CONTROLLER CANCEL CONFIRMED' if ok else 'CANCEL NOT CONFIRMED: use the pendant stop'}",
                      file=sys.stderr, flush=True)
            except BaseException as e:
                print(f"{label}: CANCEL FAILED ({e}): use the pendant stop", file=sys.stderr, flush=True)

        def send(self, client, traj, label, timeout):
            if self.stop_requested:
                raise RuntimeError(f"{label}: stop requested; nothing sent")
            goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
            goal.goal_time_tolerance = Duration(seconds=5).to_msg()
            self.stage = label; self.monitor = True; self.goal_inflight = True
            handle = done = None
            try:
                fut = client.send_goal_async(goal); until = time.monotonic() + 8
                while not fut.done() and time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.1)
                if not fut.done() or fut.result() is None or not fut.result().accepted:
                    raise RuntimeError(f"{label}: goal not accepted")
                handle = fut.result(); done = handle.get_result_async(); until = time.monotonic() + timeout
                while not done.done() and time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.stop_requested:
                        raise RuntimeError(f"{label}: Ctrl+C")
                    if time.monotonic() - self.last_at > 1.0:
                        raise RuntimeError(f"{label}: live joint state lost")
                if not done.done() or done.result() is None:
                    raise RuntimeError(f"{label}: controller outcome unknown (timeout)")
                r = done.result()
                if r.status != GoalStatus.STATUS_SUCCEEDED or r.result.error_code != 0:
                    raise RuntimeError(f"{label}: controller failure {r.result.error_code} {r.result.error_string}")
            except BaseException:
                self.stop_requested = True
                if handle is not None and done is not None and not done.done():
                    self.cancel(handle, label, done)
                raise
            finally:
                self.monitor = False; self.goal_inflight = False

        def arm_stage(self, name, p):
            live = self.fresh()
            e0 = err(live, p["q"][0])
            if e0 > START_TOL:
                raise RuntimeError(f"{name}: live joints {e0:.5f} rad from the planned start (> {START_TOL}); nothing sent")
            traj = JointTrajectory(); traj.joint_names = JOINTS
            for k, (t, q) in enumerate(zip(p["t"], p["q"])):
                pt = JointTrajectoryPoint(); pt.positions = list(live if k == 0 else q)
                pt.time_from_start = Duration(seconds=float(t)).to_msg(); traj.points.append(pt)
            self.hardware()
            self.send(self.arm, traj, name, p["duration_s"] * 3 + 30)
            end = self.fresh()
            if err(end, p["q"][-1]) > END_TOL:
                raise RuntimeError(f"{name}: endpoint error {err(end, p['q'][-1]):.5f} rad")
            print(f"{name}: PASS ({p['duration_s']:.1f} s, endpoint err {err(end, p['q'][-1]):.5f} rad)", flush=True)

        def rg2(self, name, width_m, dur):
            self.fresh()
            if self.width is None or time.monotonic() - self.width_at > 1.0:
                raise RuntimeError("No fresh RG2 finger_width state")
            traj = JointTrajectory(); traj.joint_names = ["finger_width"]
            pt = JointTrajectoryPoint(); pt.positions = [width_m]; pt.time_from_start = Duration(seconds=dur).to_msg()
            traj.points = [pt]
            self.hardware(); self.send(self.grip, traj, name, 30)
            after = time.monotonic()
            while time.monotonic() < after + 3 and self.width_at < after:
                rclpy.spin_once(self, timeout_sec=0.1)
            tol = 0.005 if name == "GRIP" else 0.003
            if self.width is None or abs(self.width - width_m) > tol:
                raise RuntimeError(f"{name}: RG2 width {self.width} m vs {width_m} m; inspect the gripper")
            print(f"{name}: PASS (measured {self.width * 1000:.2f} mm; grasp security NOT proven by width)", flush=True)

    def approve(label, detail):
        print(f"\n--- PHYSICAL STAGE REVIEW: {label} ---\n{detail}")
        print("Check: fixture and part as planned, path clear, nobody in the workspace, pendant stop in reach.")
        if node.stop_requested or input(f"Type RUN_{label} to continue (anything else stops): ").strip() != f"RUN_{label}":
            raise RuntimeError(f"{label}: operator stopped; no further stages sent")

    logdir = ROOT / "results/physical/full_cycle" / f"V{i}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    logdir.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Robot(); scene = pst.SceneClient(node)

    def on_sigint(*_):
        if node.goal_inflight:
            node.stop_requested = True
            print("\nCtrl+C: cancelling the active goal; use the pendant stop if motion continues.", file=sys.stderr, flush=True)
        else:
            raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, on_sigint)
    status = {"type": "PHYSICAL RUN (staged)", "variant": f"V{i}", "started": datetime.now().isoformat(),
              "max_joint_speed_rad_s": args.max_joint_speed, "plans": str(args.plans), "stages": []}
    part = f"pickup_V{i}"
    try:
        node.hardware()
        poses, _ = scene.world_poses()
        need = ["pickup_plate", "placement_plate", f"fitting_V{i}"]
        if any(k not in poses for k in need) and part not in scene.attached():
            raise RuntimeError("Variant scene not loaded in MoveIt: run "
                               "planning_scene_tools.py --load --physical first")
        stages = ORDER[ORDER.index(args.from_stage): ORDER.index(args.until) + 1]
        print(f"\nV{i}: stages {stages[0]} .. {stages[-1]}")
        approve("V%d_START" % i, f"Start the staged V{i} cycle ({stages[0]} .. {stages[-1]}).")
        for name in stages:
            t0 = time.monotonic()
            if name == "HOME_START":
                live = node.fresh()
                if err(live, HOME_Q) <= START_TOL:
                    print("HOME_START: already at HOME", flush=True); continue
                p = home_move(live, args.max_joint_speed)
                node.check_states(name, p["q"], every=5)
                approve(name, f"Joint-space move from the current pose to HOME ({err(live, HOME_Q):.3f} rad max joint "
                              f"change, {p['duration_s']:.1f} s). The whole route must be clear.")
                node.arm_stage(name, p)
                status["stages"].append({"name": name, "status": "PASS", "wall_s": round(time.monotonic() - t0, 2)})
                continue
            if name in ("OPEN", "RELEASE"):
                approve(name, f"RG2 opens to {RG2_OPEN_M * 1000:.0f} mm"
                        + (" around the seated part: it must stay seated." if name == "RELEASE" else "."))
                node.rg2(name, RG2_OPEN_M, RG2_OPEN_S)
                if name == "RELEASE":
                    scene.detach_v1_if_attached(remove=False)
            elif name == "GRIP":
                approve(name, f"RG2 closes to {RG2_GRIP_M * 1000:.1f} mm on part V{i}.")
                node.rg2(name, RG2_GRIP_M, RG2_GRIP_S)
                scene.attach_part(part, G, objs[part]["mesh"])
                scene.set_allowed(part, "pickup_plate", True)
            else:
                p = plans[name]
                if name == "INSERT":
                    scene.set_allowed(part, f"fitting_V{i}", True)       # judged by the CAD classifier offline
                node.check_states(name, p["q"])
                approve(name, f"{name}: {len(p['q'])} points, {p['duration_s']:.1f} s "
                              f"(slowed {p['slowdown']:.2f}x to <= {args.max_joint_speed} rad/s).")
                node.arm_stage(name, p)
                if name == "LIFT":
                    scene.set_allowed(part, "pickup_plate", False)
            status["stages"].append({"name": name, "status": "PASS", "wall_s": round(time.monotonic() - t0, 2)})
        status["status"] = "PASS"
        print(f"\nV{i}: all requested stages PASS. Inspect the part and record the physical outcome.")
        return 0
    except BaseException as e:
        status["status"] = "STOPPED"; status["reason"] = str(e)
        print(f"\nSTOPPED: {e}\nInspect the robot and part before anything else. No further commands were sent.",
              file=sys.stderr)
        return 1
    finally:
        status["ended"] = datetime.now().isoformat()
        json.dump(status, open(logdir / "status.json", "w"), indent=1)
        with open(logdir / "joint_log.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["time", "stage"] + JOINTS + ["finger_width"]); w.writerows(node.records)
        print(f"Log -> {logdir}")
        node.destroy_node(); rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", type=int, required=True, choices=[1, 2, 3, 4, 5])
    ap.add_argument("--plans", type=Path, default=default_plans())
    ap.add_argument("--max-joint-speed", type=float, default=0.14)
    ap.add_argument("--from", dest="from_stage", default="HOME_START", choices=ORDER)
    ap.add_argument("--until", default="HOME", choices=ORDER,
                    help="stop after this stage (first runs: --until ALIGN_INSERT, then inspect the alignment)")
    ap.add_argument("--execute", action="store_true", help="REAL ROBOT: staged execution with operator approval")
    a = ap.parse_args()
    if not 0 < a.max_joint_speed <= HARD_MAX_SPEED:
        raise SystemExit(f"--max-joint-speed must be in (0, {HARD_MAX_SPEED}]")
    if ORDER.index(a.from_stage) > ORDER.index(a.until):
        raise SystemExit("--from is after --until")
    plans, G, objs = preflight(a.variant, a.plans, a.max_joint_speed)
    if not a.execute:
        print("\nNo robot motion. Add --execute on the lab laptop (ROS_DOMAIN_ID=0) to run it staged.")
        return 0
    return execute(a.variant, plans, G, objs, a)


if __name__ == "__main__":
    sys.exit(main())
