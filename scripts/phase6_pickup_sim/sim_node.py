"""Shared fake-hardware node for the V1 simulation stages (ROS_DOMAIN_ID=73 only).

make_node() returns a rclpy node with: hardware guard, joint-state recording,
arm/gripper trajectory execution, MoveIt FK, Cartesian feasibility check and
state-validity (collision) checking with attached objects kept.
"""
import csv
import math
import time
from pathlib import Path

import lift_profile as lp
import planning_scene_tools as pst
import scene_model as sm
import ur_kinematics as uk

np = sm.np
ROOT = Path(sm.PKG)
JOINTS = lp.JOINTS


def load_arm(rel, multiplier, max_step):
    """Joint CSV (time + 6 joints) -> {'t','q'} with time scaled by `multiplier`, guarded."""
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
    """(translation mm, rotation deg) between two 4x4 poses."""
    dt = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
    ang = math.degrees(math.acos(np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1)))
    return dt, ang


def densify_joint_path(q, calib, max_tcp_step_m=0.001):
    """Joint-linear sub-samples between consecutive samples so the TCP moves <= max step.

    The joint trajectory controller interpolates between points, so these are the
    in-between states actually executed. Returns list of (segment_index, q).
    """
    out = []
    for i, (a, b) in enumerate(zip(q, q[1:])):
        a = np.asarray(a); b = np.asarray(b)
        d = np.linalg.norm(uk.tcp(b, calib)[:3, 3] - uk.tcp(a, calib)[:3, 3])
        n = max(1, int(math.ceil(d / max_tcp_step_m)))
        for k in range(1, n):
            out.append((i, (a + (b - a) * k / n).tolist()))
    return out


MAX_SPEED = 5.0


def add_speed_arg(ap):
    """--speed N: play every fake-hardware motion back N times faster (watching only).

    Planning, IK, collision and contact checks use the original timestamps and are
    unchanged; only the trajectory times sent to the fake controller are divided by N.
    Not for physical hardware (these scripts are fake-hardware only)."""
    ap.add_argument("--speed", type=float, default=1.0,
                    help=f"fake-hardware playback speed-up, 1-{MAX_SPEED:g} (results unchanged; default 1)")


def set_speed(node, speed):
    if not 1.0 <= speed <= MAX_SPEED:
        raise SystemExit(f"--speed must be between 1 and {MAX_SPEED:g}")
    node.speed = float(speed)
    if speed != 1.0:
        print(f"PLAYBACK x{speed:g} (fake hardware only; planned timing and all checks unchanged)", flush=True)


def make_node(name="v1_sim_SIM_ONLY"):
    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class SimNode(Node):
        def __init__(self):
            super().__init__(name)
            self.q = None; self.msg = None; self.width = None; self.q_at = 0.0
            self.speed = 1.0          # fake-hardware playback speed-up (set_speed)
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

        # ---------------- state
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

        # ---------------- execution
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
                p.time_from_start = Duration(seconds=float(ti) / self.speed).to_msg(); jt.points.append(p)
            self.samples = []; self.t0 = time.monotonic(); self.rec = True
            start = time.monotonic()
            try:
                self.send(self.arm, jt, t[-1] / self.speed + 10)
            finally:
                self.rec = False
            elapsed = time.monotonic() - start
            err = lp.maxdiff(self.fresh(), q[-1])
            if err > end_tol:
                raise RuntimeError(f"{name}: endpoint error {err:.5f} rad")
            shown = f"{t[-1]:.2f} s" + (f" played in {t[-1]/self.speed:.2f} s" if self.speed != 1.0 else "")
            print(f"{name}: PASS ({len(q)} pts, {shown}, endpoint err {err:.6f} rad)", flush=True)
            self.last_elapsed = elapsed
            return list(self.samples), err

        def run_gripper(self, name, width_m):
            self.guard(); self.fresh()
            target, tol = self.width_target(width_m)
            jt = JointTrajectory(); jt.joint_names = ["finger_width"]
            p = JointTrajectoryPoint(); p.positions = [target]; p.time_from_start = Duration(seconds=max(0.5, 3.0 / self.speed)).to_msg()
            jt.points = [p]; self.send(self.grip, jt, 8); self.fresh()
            if abs(self.width - target) > tol:
                raise RuntimeError(f"{name}: gripper did not reach {width_m*1000:.1f} mm")
            print(f"{name}: PASS (RG2 {width_m*1000:.1f} mm)", flush=True)

        # ---------------- MoveIt queries
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

        @staticmethod
        def pose_msg(T):
            ps = Pose(); ps.position.x, ps.position.y, ps.position.z = (float(v) for v in T[:3, 3])
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = (float(v) for v in sm.matrix_to_quat(T[:3, :3]))
            return ps

        def cartesian(self, poses, label="Cartesian path", max_step=0.002):
            """MoveIt Cartesian feasibility check from the CURRENT state (as the physical scripts)."""
            req = GetCartesianPath.Request(); req.header.frame_id = "base"
            req.group_name = "ur_onrobot_manipulator"; req.link_name = "gripper_tcp"
            req.start_state = RobotState(); req.start_state.joint_state = self.msg; req.start_state.is_diff = True
            req.waypoints = [p if isinstance(p, Pose) else self.pose_msg(p) for p in poses]
            req.max_step = max_step; req.jump_threshold = 0.0
            req.avoid_collisions = True
            for f, v in (("max_velocity_scaling_factor", 0.03), ("max_acceleration_scaling_factor", 0.03)):
                if hasattr(req, f):  # newer moveit_msgs only (physical script used 0.03)
                    setattr(req, f, v)
            r = self.rpc(self.cart_cli, req, label, 30)
            jt = r.solution.joint_trajectory
            if r.error_code.val != 1 or r.fraction < 0.999 or len(jt.points) < 2:
                raise RuntimeError(f"{label} incomplete: fraction={r.fraction:.4f}, error={r.error_code.val}")
            return r.fraction, len(jt.points)

        def valid(self, q):
            req = GetStateValidity.Request(); req.group_name = "ur_onrobot_manipulator"
            req.robot_state = RobotState(); req.robot_state.is_diff = True  # keep attached V1
            req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = [float(v) for v in q]
            r = self.rpc(self.valid_cli, req, "state validity", 6)
            return r.valid, sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts})

        def check_path(self, name, t, q, calib, max_tcp_step_m=0.001):
            """Collision-check every sample AND joint-linear in-between states (TCP step <= 1 mm)."""
            bad = []
            for i, qi in enumerate(q):
                ok, c = self.valid(qi)
                if not ok:
                    bad.append({"t_s": round(float(t[i]), 3), "kind": "sample", "contacts": c})
            mids = densify_joint_path(q, calib, max_tcp_step_m)
            for i, qi in mids:
                ok, c = self.valid(qi)
                if not ok:
                    bad.append({"t_s": round(float(t[i]), 3), "kind": "interpolated", "contacts": c})
            print(f"{name}: collision check {len(q)} samples + {len(mids)} interpolated states -> "
                  f"{'none' if not bad else str(len(bad)) + ' invalid'}", flush=True)
            return bad, len(q) + len(mids)

    if not rclpy.ok():
        rclpy.init()
    return rclpy, SimNode()
