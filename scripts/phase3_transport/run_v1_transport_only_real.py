#!/usr/bin/env python3
"""V1 transport-only real UR3e pilot.

Default: offline CSV preflight ONLY.
--check-live: read hardware/controller/current joints and RG2; NO movement.
--execute: guarded, supervised TRANSPORT ONLY; DOES NOT align/insert/release/home.
Keep pendant E-stop accessible. Ctrl+C requests action cancellation, NOT E-stop.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"
CSV_PATH = ROOT / "demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv"
LOG_ROOT = ROOT / "results/physical/v1_transport_only"
JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]
ARM_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
# Validate against exact known successful V1 physical pilot source.
EXPECTED_SHA_PREFIX = "f4335b3ef675"
TIME_SCALE = 2.0
START_TOL = 0.005
END_TOL = 0.003
MAX_STEP = 0.05
MAX_SPEED = 0.20
# Observed actual RG2 width around 10.3 mm for the commanded 8.2-mm grasp.
GRIP_MIN_MM, GRIP_MAX_MM = 7.0, 14.5


def load():
    data = CSV_PATH.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if not digest.startswith(EXPECTED_SHA_PREFIX):
        raise RuntimeError(
            "Transport CSV hash differs from validated V1 pilot; "
            f"expected {EXPECTED_SHA_PREFIX}, got {digest[:12]}")
    with CSV_PATH.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != ["time"] + JOINTS:
            raise RuntimeError("Unexpected CSV columns")
        rows = list(reader)
    if len(rows) != 246:
        raise RuntimeError(f"Expected 246 transport samples, found {len(rows)}")
    ts = [float(r["time"]) for r in rows]
    qs = [[float(r[j]) for j in JOINTS] for r in rows]
    if any(not math.isfinite(t) for t in ts) or any(
        not math.isfinite(v) for row in qs for v in row
    ):
        raise RuntimeError("Nonfinite trajectory data")
    if abs(ts[0]) > 1e-6 or any(b <= a for a, b in zip(ts, ts[1:])):
        raise RuntimeError("Invalid trajectory timestamps")
    ts = [t * TIME_SCALE for t in ts]
    if not 48.9 < ts[-1] < 49.1:
        raise RuntimeError("Unexpected transport duration")
    step = max(abs(b[k]-a[k]) for a,b in zip(qs,qs[1:]) for k in range(6))
    speed = max(abs(qs[i+1][k] - qs[i][k])/(ts[i+1]-ts[i])
                for i in range(len(qs)-1) for k in range(6))
    if step > MAX_STEP or speed > MAX_SPEED:
        raise RuntimeError(f"Joint limits failed: step={step}, speed={speed}")
    return ts, qs, digest, step, speed


def err(a, b):
    return max(abs(x-y) for x,y in zip(a,b))


def fmt(q):
    return "[" + ", ".join(f"{v:+.5f}" for v in q) + "]"


def gripper_mm(value):
    if value is None or not math.isfinite(value):
        raise RuntimeError("No finite RG2 finger_width state")
    # /joint_states reports RG2 finger_width in metres in this installation.
    # The only other accepted encoding is explicit millimetres.
    if 0.0 <= value <= 0.12:
        return value * 1000.0
    if 5.0 <= value <= 120.0:
        return value
    raise RuntimeError(f"Unexpected gripper state units: {value}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-live", action="store_true", help="Read-only hardware/state check")
    mode.add_argument("--execute", action="store_true", help="Supervised physical transport")
    args = parser.parse_args()

    ts, qs, digest, step, speed = load()
    print("\n=========== V1 TRANSPORT ONLY ===========", flush=True)
    print(f"Trajectory: {CSV_PATH}")
    print(f"SHA256: {digest[:12]} (validated pilot file)")
    print(f"Points: {len(qs)} | time: {ts[-1]:.3f} s (x2)")
    print(f"Max step: {step:.6f} rad | max joint speed: {speed:.5f} rad/s")
    print(f"Expected start joints: {fmt(qs[0])}")
    print(f"Expected end joints:   {fmt(qs[-1])}")
    print("No REACH, ALIGN_PICKUP, GRIP, ALIGN_INSERT, INSERT, ROTATE, RELEASE or HOME.")
    if not (args.check_live or args.execute):
        print("\nOFFLINE PREFLIGHT PASS — NO ROBOT MOTION.")
        return

    if os.environ.get("ROS_DOMAIN_ID") != "0":
        raise RuntimeError("Requires ROS_DOMAIN_ID=0 (physical lab setup)")

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
    from action_msgs.msg import GoalStatus

    class CheckNode(Node):
        def __init__(self):
            super().__init__("v1_transport_only_physical")
            self.latest_q = None
            self.latest_grip_mm = None
            self.last_joint_at = 0.0
            self.stopped = False
            self.cancel_requested = False
            self.samples = []
            self.recording = False
            self.t0 = 0.0
            self.create_subscription(JointState, "/joint_states", self.on_state,
                                     qos_profile_sensor_data)
            self.hw = self.create_client(
                ListHardwareComponents, "/controller_manager/list_hardware_components")
            self.controllers = self.create_client(
                ListControllers, "/controller_manager/list_controllers")
            self.action = ActionClient(self, FollowJointTrajectory, ARM_ACTION)

        def on_state(self, msg):
            if not all(j in msg.name for j in JOINTS):
                return
            if len(msg.name) != len(msg.position):
                return
            self.latest_q = [float(msg.position[msg.name.index(j)]) for j in JOINTS]
            self.last_joint_at = time.monotonic()
            if "finger_width" in msg.name:
                self.latest_grip_mm = gripper_mm(
                    float(msg.position[msg.name.index("finger_width")]))
            if self.recording:
                self.samples.append(
                    [time.monotonic() - self.t0] + self.latest_q +
                    [self.latest_grip_mm if self.latest_grip_mm is not None else ""])

        def spin_until(self, predicate, timeout, label):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self.stopped:
                    raise InterruptedError("Operator interrupted: " + label)
                rclpy.spin_once(self, timeout_sec=0.10)
                if predicate():
                    return
            raise RuntimeError("Timed out: " + label)

        def refresh(self):
            self.last_joint_at = 0.0
            self.spin_until(
                lambda: self.last_joint_at > 0.0 and
                self.latest_grip_mm is not None, 4.0, "fresh joint/gripper states")
            if not all(math.isfinite(v) for v in self.latest_q):
                raise RuntimeError("Invalid joint positions")
            return self.latest_q[:], self.latest_grip_mm

        def call_service(self, client, request, label):
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(label + " unavailable")
            future = client.call_async(request)
            self.spin_until(lambda: future.done(), 6.0, label)
            if future.result() is None:
                raise RuntimeError(label + " empty result")
            return future.result()

        def guard(self):
            hw = self.call_service(self.hw, ListHardwareComponents.Request(),
                                   "hardware list").component
            real = [h for h in hw if h.name == "ur3e"]
            if len(real) != 1 or real[0].class_type != \
                    "ur_robot_driver/URPositionHardwareInterface":
                raise RuntimeError("Real UR3e hardware not verified")
            ctl = self.call_service(
                self.controllers, ListControllers.Request(), "controller list").controller
            st = {c.name: c.state.lower() for c in ctl}
            if st.get("scaled_joint_trajectory_controller") != "active":
                raise RuntimeError("Scaled UR arm controller not active")
            if st.get("joint_trajectory_controller") == "active":
                raise RuntimeError("Unexpected second active arm controller")
            if st.get("finger_width_trajectory_controller") != "active":
                raise RuntimeError("RG2 finger controller not active")
            if not self.action.wait_for_server(timeout_sec=5):
                raise RuntimeError("Physical arm action unavailable")

        def check_start(self):
            current, width = self.refresh()
            delta = err(current, qs[0])
            print(f"Live transport start error: {delta:.6f} rad")
            print(f"RG2 measured width: {width:.2f} mm")
            if delta > START_TOL:
                raise RuntimeError(
                    "Not at TRANSPORT start; do not teleport or run REACH again")
            if not GRIP_MIN_MM <= width <= GRIP_MAX_MM:
                raise RuntimeError("RG2 grasp width outside expected physical range")
            return current

    rclpy.init()
    node = CheckNode()
    goal_handle = None
    old_sigint = signal.getsignal(signal.SIGINT)
    def interrupt(_signum, _frame):
        node.stopped = True
    signal.signal(signal.SIGINT, interrupt)

    def cancel_goal():
        if goal_handle is None:
            print("No accepted active goal to cancel.", flush=True)
            return
        print("Requesting arm controller CANCEL (not an E-stop).", flush=True)
        try:
            future = goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done() or future.result() is None or \
                    not future.result().goals_canceling:
                print("CANCELLATION NOT CONFIRMED — use pendant STOP.", flush=True)
            else:
                print("Controller acknowledged cancellation request.", flush=True)
        except Exception as ex:
            print(f"Cancel error ({ex}); use pendant STOP.", flush=True)

    try:
        node.guard()
        live = node.check_start()
        print("Real UR3e/controller/start/gripper: VERIFIED.")
        print("Check actual workpiece is firmly held, fixture is empty, "
              "and whole transport path is clear.")
        if args.check_live:
            print("LIVE PRECHECK PASS — NO ROBOT MOTION.")
            return
        print("\nPHYSICAL TRANSPORT WILL MOVE ROBOT.")
        print("This does NOT check unmodelled fixture collisions. "
              "Supervise with pendant stop accessible.")
        confirm = input("Type EXECUTE_V1_TRANSPORT_ONLY to proceed: ").strip()
        if confirm != "EXECUTE_V1_TRANSPORT_ONLY":
            print("Cancelled before transport; no motion.")
            return

        node.guard()
        live = node.check_start()  # fresh state AFTER human confirmation
        # Same approach as successful V1 physical integrated pilot:
        # substitute the live start as the first trajectory waypoint;
        # retain all subsequent learned path samples and 2x timing.
        points = [live] + qs[1:]
        if err(points[0], points[1]) > MAX_STEP:
            raise RuntimeError("Corrected initial trajectory jump too large")
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        for t, q in zip(ts, points):
            point = JointTrajectoryPoint()
            point.positions = q
            point.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(seconds=4.0).to_msg()

        node.samples = []
        node.t0 = time.monotonic()
        node.recording = True
        future = node.action.send_goal_async(goal)
        node.spin_until(lambda: future.done(), 7.0, "trajectory goal acceptance")
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Transport goal was rejected")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + ts[-1] + 18.0
        print("TRANSPORT RUNNING. Supervise physical path.", flush=True)
        while not result_future.done():
            if node.stopped:
                raise InterruptedError("Operator requested cancellation")
            if time.monotonic() > deadline:
                raise RuntimeError("Transport result timed out")
            rclpy.spin_once(node, timeout_sec=0.10)
            if time.monotonic() - node.last_joint_at > 1.5:
                raise RuntimeError("Joint-state stream lost")
            if node.latest_grip_mm is None or not (
                    GRIP_MIN_MM <= node.latest_grip_mm <= GRIP_MAX_MM):
                raise RuntimeError("Gripper width outside expected range")
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED or \
                wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError("Physical transport controller did not report success")
        node.recording = False
        end_q, end_grip = node.refresh()
        final_error = err(end_q, qs[-1])
        if final_error > END_TOL:
            raise RuntimeError(f"Endpoint joint error {final_error:.6f} rad")
        print(f"TRANSPORT PASS — final joint error {final_error:.6f} rad")
        print(f"Gripper remains closed: {end_grip:.2f} mm")
        print("ROBOT STOPPED AT TRANSPORT END. Inspect alignment before INSERT.")

        run_dir = LOG_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_dir.mkdir(parents=True, exist_ok=False)
        with (run_dir/"joint_states.csv").open("x", newline="") as f:
            w = csv.writer(f)
            w.writerow(["elapsed_s"] + JOINTS + ["gripper_width_mm"])
            w.writerows(node.samples)
        (run_dir/"status.json").write_text(json.dumps({
            "trial_type": "PILOT_TRANSPORT_ONLY",
            "status": "PASS",
            "trajectory_sha256": digest,
            "duration_target_s": ts[-1],
            "endpoint_error_rad": final_error,
            "gripper_end_mm": end_grip
        }, indent=2))
        print(f"Logs: {run_dir}")
    except BaseException:
        if goal_handle is not None:
            cancel_goal()
        raise
    finally:
        node.recording = False
        signal.signal(signal.SIGINT, old_sigint)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, InterruptedError) as ex:
        raise SystemExit(f"TRANSPORT INTERRUPTED: {ex}. Check robot before any new goal.")
    except (RuntimeError, OSError, ValueError) as ex:
        raise SystemExit(f"BLOCKED / STOPPED: {ex}")
