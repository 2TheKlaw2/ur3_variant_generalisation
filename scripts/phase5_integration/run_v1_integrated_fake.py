#!/usr/bin/env python3
"""Guarded V1 sequence for isolated ROS 2 fake hardware only."""

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

if os.environ.get("ROS_DOMAIN_ID") != "73":
    raise SystemExit("BLOCKED: Require ROS_DOMAIN_ID=73")

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

JOINTS = [
    s + "_joint" for s in (
        "shoulder_pan", "shoulder_lift", "elbow",
        "wrist_1", "wrist_2", "wrist_3"
    )
]

# Stage, saved trajectory, timing multiplier, maximum joint step.
STAGES = [
    ("REACH",
     "demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv",
     2.0, 0.10),

    ("ALIGN_PICKUP",
     "config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv",
     2.0, 0.05),

    ("TRANSPORT",
     "demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv",
     2.0, 0.05),

    ("ALIGN_INSERT",
     "config/trajectories/v1/v1_align_to_insertion_joint_plan_001.csv",
     2.0, 0.05),

    ("INSERT",
     "demonstrations/processed/phase4_assembly/v1_insert_joint_sim_13s_CANDIDATE.csv",
     1.0, 0.01),

    ("ROTATE",
     "demonstrations/processed/phase4_assembly/v1_rotate_joint_sim_17s_CANDIDATE.csv",
     1.0, 0.04),
]


def load_arm(path, multiplier, max_allowed_step):
    p = Path(path)
    if not p.is_absolute():
        p = ROOT / p

    if not p.is_file():
        raise RuntimeError(f"Missing trajectory: {p}")

    with p.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or not set(
            ["time"] + JOINTS
        ).issubset(reader.fieldnames):
            raise RuntimeError(f"Unexpected CSV columns: {p}")
        rows = list(reader)

    if not 2 <= len(rows) <= 2000:
        raise RuntimeError(f"Invalid trajectory length: {p}")

    start_time = float(rows[0]["time"])

    times = [
        (float(r["time"]) - start_time) * multiplier
        for r in rows
    ]

    positions = [
        [float(r[j]) for j in JOINTS]
        for r in rows
    ]

    if any(
        not math.isfinite(v)
        for row in positions for v in row
    ) or any(not math.isfinite(t) for t in times):
        raise RuntimeError("Non-finite trajectory data")

    if any(b <= a for a, b in zip(times, times[1:])):
        raise RuntimeError("Invalid trajectory timing")

    if not 0 < times[-1] <= 150:
        raise RuntimeError("Invalid trajectory duration")

    step = max(
        abs(b[k] - a[k])
        for a, b in zip(positions, positions[1:])
        for k in range(6)
    )

    speed = max(
        abs(positions[i+1][k] - positions[i][k])
        / (times[i+1] - times[i])
        for i in range(len(positions)-1)
        for k in range(6)
    )

    if step > max_allowed_step or speed > 0.5:
        raise RuntimeError(
            f"Joint step/speed limit exceeded: {p}"
        )

    return {
        "times": times,
        "positions": positions,
        "step": step,
        "speed": speed,
    }


class FakeController(Node):
    def __init__(self):
        super().__init__("v1_integrated_mock_only")

        self.arm = None
        self.gripper = None
        self.record = False
        self.samples = []
        self.start_time = 0

        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_state,
            qos_profile_sensor_data,
        )

        self.hardware = self.create_client(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
        )

        self.controllers = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )

        self.arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/finger_width_trajectory_controller/follow_joint_trajectory",
        )

    def on_state(self, msg):
        if all(j in msg.name for j in JOINTS):
            self.arm = [
                float(msg.position[msg.name.index(j)])
                for j in JOINTS
            ]

            if self.record:
                self.samples.append(
                    [time.monotonic() - self.start_time]
                    + self.arm
                )

        if "finger_width" in msg.name:
            self.gripper = float(
                msg.position[msg.name.index("finger_width")]
            )

    def request(self, client, srv):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Controller service unavailable")

        future = client.call_async(srv.Request())

        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5
        )

        if not future.done() or future.result() is None:
            raise RuntimeError("Controller service request failed")

        return future.result()

    def guard(self):
        components = self.request(
            self.hardware, ListHardwareComponents
        ).component

        actual = {
            (h.name, h.class_type)
            for h in components
        }

        expected = {
            ("ur3e", "mock_components/GenericSystem"),
            (
                "OnRobotGripper",
                "test_hardware_components/TestSingleJointActuator",
            ),
        }

        if actual != expected:
            raise RuntimeError(
                f"BLOCKED: Unexpected hardware: {actual}"
            )

        arm = next(h for h in components if h.name == "ur3e")
        grip = next(
            h for h in components
            if h.name == "OnRobotGripper"
        )

        arm_interfaces = {
            i.name for i in arm.command_interfaces
        }

        grip_interfaces = {
            i.name for i in grip.command_interfaces
        }

        if not {
            j + "/position" for j in JOINTS
        }.issubset(arm_interfaces):
            raise RuntimeError("Fake UR3e interfaces missing")

        if "finger_width/position" not in grip_interfaces:
            raise RuntimeError("Fake gripper interface missing")

        states = {
            c.name: c.state.lower()
            for c in self.request(
                self.controllers, ListControllers
            ).controller
        }

        if states.get("joint_trajectory_controller") != "active":
            raise RuntimeError("Arm controller inactive")

        if states.get("finger_width_trajectory_controller") != "active":
            raise RuntimeError("Gripper controller inactive")

        if states.get("scaled_joint_trajectory_controller") == "active":
            raise RuntimeError(
                "BLOCKED: Scaled controller active"
            )

    def refresh(self):
        self.arm = None
        self.gripper = None

        deadline = time.monotonic() + 5

        while (
            (self.arm is None or self.gripper is None)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.arm is None or self.gripper is None:
            raise RuntimeError("Missing joint feedback")

    def gripper_target(self, width_mm):
        # Support only a verified metres or millimetres state.
        # Reject unknown or ambiguous joint values.
        if 0.005 <= self.gripper <= 0.120:
            return width_mm / 1000.0, 0.0015

        if 5 <= self.gripper <= 120:
            return float(width_mm), 1.5

        raise RuntimeError(
            f"Unknown gripper units: {self.gripper}"
        )

    def execute(self, client, trajectory, timeout):
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError("Trajectory action unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(
            seconds=2
        ).to_msg()

        sent = client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self, sent, timeout_sec=5
        )

        if (
            not sent.done()
            or sent.result() is None
            or not sent.result().accepted
        ):
            raise RuntimeError("Trajectory goal rejected")

        result = sent.result().get_result_async()

        rclpy.spin_until_future_complete(
            self, result, timeout_sec=timeout
        )

        if not result.done() or result.result() is None:
            raise RuntimeError(
                "Execution timed out; inspect controller before retry"
            )

        reply = result.result()

        if (
            reply.status != GoalStatus.STATUS_SUCCEEDED
            or reply.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError("Controller reported execution failure")

    def arm_stage(self, name, data, log_dir):
        self.guard()
        self.refresh()

        start_error = max(
            abs(a-b)
            for a, b in zip(
                self.arm,
                data["positions"][0]
            )
        )

        if start_error > 0.005:
            raise RuntimeError(
                f"{name} start mismatch: {start_error:.6f} rad"
            )

        label = "ALIGN" if name.startswith("ALIGN_") else name

        print(
            f"\n{name}: {len(data['positions'])} points, "
            f"{data['times'][-1]:.3f} seconds"
        )

        if input(
            f"Type {label} for fake {name}: "
        ).strip() != label:
            raise RuntimeError(f"Cancelled: {name}")

        self.guard()
        self.refresh()

        if max(
            abs(a-b)
            for a, b in zip(
                self.arm, data["positions"][0]
            )
        ) > 0.005:
            raise RuntimeError("Starting joints changed")

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINTS

        for t, q in zip(
            data["times"], data["positions"]
        ):
            point = JointTrajectoryPoint()
            point.positions = q
            point.time_from_start = Duration(
                seconds=t
            ).to_msg()

            trajectory.points.append(point)

        self.samples = []
        self.start_time = time.monotonic()
        self.record = True

        try:
            self.execute(
                self.arm_action,
                trajectory,
                data["times"][-1] + 10,
            )
        finally:
            self.record = False

        self.refresh()

        error = max(
            abs(a-b)
            for a, b in zip(
                self.arm, data["positions"][-1]
            )
        )

        if error > 0.002:
            raise RuntimeError(
                f"{name} endpoint error: {error:.6f} rad"
            )

        with (
            log_dir / f"{name.lower()}.csv"
        ).open("x", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["elapsed_s"] + JOINTS)
            writer.writerows(self.samples)

        print(f"{name}: PASS")
        print(f"Final joint error: {error:.6f} rad")

    def gripper_stage(self, name, width_mm):
        self.guard()
        self.refresh()

        target, tolerance = self.gripper_target(width_mm)

        print(f"\n{name}: RG2 target = {width_mm} mm")

        if input(
            f"Type {name} to continue: "
        ).strip() != name:
            raise RuntimeError(f"Cancelled: {name}")

        self.guard()
        self.refresh()

        updated_target, _ = self.gripper_target(width_mm)

        if abs(updated_target - target) > 1e-9:
            raise RuntimeError("Gripper units changed")

        trajectory = JointTrajectory()
        trajectory.joint_names = ["finger_width"]

        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start = Duration(
            seconds=3
        ).to_msg()

        trajectory.points = [point]

        self.execute(
            self.gripper_action,
            trajectory,
            8,
        )

        self.refresh()

        if abs(self.gripper - target) > tolerance:
            raise RuntimeError(
                f"Gripper did not reach {width_mm} mm"
            )

        print(f"{name}: PASS — {width_mm} mm")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--run",
        action="store_true",
        help="Run the guarded sequence",
    )

    args = parser.parse_args()

    print("\n========== V1 INTEGRATED SIMULATION ==========")
    print("HOME → OPEN 20 → REACH → ALIGN → GRIP 9")
    print("→ TRANSPORT → ALIGN → INSERT → ROTATE → RELEASE 20")

    trajectories = []

    for name, path, multiplier, limit in STAGES:
        data = load_arm(path, multiplier, limit)
        trajectories.append((name, data))

        print(
            f"{name:14s}: "
            f"{len(data['positions']):4d} points, "
            f"{data['times'][-1]:7.2f} s"
        )

    for (a, first), (b, second) in zip(
        trajectories, trajectories[1:]
    ):
        error = max(
            abs(x-y)
            for x, y in zip(
                first["positions"][-1],
                second["positions"][0],
            )
        )

        print(
            f"Handoff {a} → {b}: {error:.6f} rad"
        )

        if error > 0.010:
            raise RuntimeError(
                f"Handoff mismatch: {a} → {b}"
            )

    rclpy.init()
    node = FakeController()

    try:
        node.guard()
        node.refresh()
        node.gripper_target(20)

        print("\nFAKE HARDWARE VERIFIED")

        if not args.run:
            print("PREFLIGHT PASS")
            print("NO ROBOT MOVEMENT")
            return

        if input(
            "\nType START to begin guarded V1 simulation: "
        ).strip() != "START":
            raise RuntimeError("Cancelled before movement")

        log_dir = (
            ROOT
            / "results/simulation/v1_integrated"
            / datetime.now().strftime("%Y%m%d_%H%M%S")
        )

        log_dir.mkdir(parents=True, exist_ok=False)

        node.guard()

        home = (
            ROOT
            / "scripts/phase1_reach/sim_reach_action.py"
        )

        if not home.is_file():
            raise RuntimeError("Existing HOME script missing")

        print("\nSTAGE 1 — HOME")

        # Reuse the existing guarded HOME script.
        subprocess.run(
            [sys.executable, str(home), "--home"],
            cwd=str(ROOT),
            check=True,
        )

        node.refresh()

        print("\nSTAGE 2 — OPEN")
        node.gripper_stage("OPEN", 20)

        for name, data in trajectories:
            if name == "TRANSPORT":
                print("\nGRIP COMPONENT")
                node.gripper_stage("GRIP", 9)

            node.arm_stage(name, data, log_dir)

        print("\nFINAL STAGE — RELEASE")
        node.gripper_stage("RELEASE", 20)

        print("\n========== COMPLETE ==========")
        print("Full V1 command sequence: PASS")
        print(f"Logs: {log_dir}")
        print("FAKE HARDWARE ONLY")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (
        RuntimeError,
        ValueError,
        OSError,
        subprocess.CalledProcessError,
    ) as exc:
        raise SystemExit(f"STOPPED: {exc}")
