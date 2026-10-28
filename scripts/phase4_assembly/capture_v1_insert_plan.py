#!/usr/bin/env python3

import csv
import math
import os
import time
from pathlib import Path

if os.environ.get("ROS_DOMAIN_ID") != "73":
    raise SystemExit("BLOCKED: ROS_DOMAIN_ID must be 73")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.msg import DisplayTrajectory

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

DEMO = ROOT / "demonstrations/processed/v1_insert_001_motion.csv"
OUTPUT = ROOT / (
    "demonstrations/processed/phase4_assembly/"
    "v1_insert_joint_sim_candidate.csv"
)

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

SHORT = [j.removesuffix("_joint") for j in JOINTS]

if OUTPUT.exists():
    raise SystemExit("BLOCKED: Output already exists; refusing overwrite.")

with DEMO.open(newline="") as f:
    rows = list(csv.DictReader(f))

if len(rows) < 100:
    raise SystemExit("BLOCKED: Missing or invalid INSERT demonstration.")

demo_start = [float(rows[0][j]) for j in SHORT]
demo_end = [float(rows[-1][j]) for j in SHORT]


class InsertCapture(Node):

    def __init__(self):
        super().__init__("v1_insert_plan_capture")
        self.finished = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self.callback,
            qos,
        )

    def callback(self, msg):

        for item in msg.trajectory:
            traj = item.joint_trajectory
            n = len(traj.points)

            if n < 7 or n > 40:
                print(f"Ignoring unrelated plan: {n} points")
                continue

            try:
                indices = [traj.joint_names.index(j) for j in JOINTS]

                times = [
                    p.time_from_start.sec +
                    p.time_from_start.nanosec * 1e-9
                    for p in traj.points
                ]

                raw = [
                    [float(p.positions[i]) for i in indices]
                    for p in traj.points
                ]

                if any(
                    not math.isfinite(v)
                    for row in raw for v in row
                ):
                    raise ValueError("Non-finite joint position")

                if abs(times[0]) > 0.15:
                    raise ValueError("Unexpected initial timestamp")

                if any(
                    b <= a for a, b in zip(times, times[1:])
                ):
                    raise ValueError("Invalid trajectory timing")

                if times[-1] > 30:
                    raise ValueError("Unexpected duration")

                # Correct the Wrist 3 angular branch.
                shifts = [
                    2 * math.pi * round(
                        (demo_start[i] - raw[0][i]) /
                        (2 * math.pi)
                    )
                    for i in range(6)
                ]

                if any(abs(s) > 1e-4 for s in shifts[:5]):
                    raise ValueError("Unexpected joint branch shift")

                corrected = [
                    [v + shifts[i] for i, v in enumerate(row)]
                    for row in raw
                ]

                start_error = max(
                    abs(a - b)
                    for a, b in zip(corrected[0], demo_start)
                )

                end_error = max(
                    abs(a - b)
                    for a, b in zip(corrected[-1], demo_end)
                )

                max_step = max(
                    abs(b[i] - a[i])
                    for a, b in zip(corrected, corrected[1:])
                    for i in range(6)
                )

                total_motion = max(
                    abs(a - b)
                    for a, b in zip(corrected[0], corrected[-1])
                )

                if start_error > 0.015:
                    raise ValueError("Incorrect starting configuration")

                if end_error > 0.015:
                    raise ValueError("Incorrect ending configuration")

                if max_step > 0.05:
                    raise ValueError("Joint discontinuity detected")

                if total_motion < 0.012:
                    raise ValueError("Likely pre-positioning plan")

                OUTPUT.parent.mkdir(parents=True, exist_ok=True)

                with OUTPUT.open("x", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["time"] + JOINTS)

                    for t, q in zip(times, corrected):
                        writer.writerow([t] + q)

                print("\n========== V1 INSERT CAPTURED ==========")
                print(f"Joint trajectory points: {n}")
                print(f"Duration: {times[-1]:.3f} s")
                print(f"Start error: {start_error:.6f} rad")
                print(f"End error: {end_error:.6f} rad")
                print(f"Maximum joint step: {max_step:.6f} rad")
                print(f"Wrist 3 correction: {shifts[5]:.5f} rad")
                print(f"Saved: {OUTPUT}")
                print("PLAN ONLY — NO ROBOT MOVEMENT")
                print("NOT APPROVED FOR EXECUTION")

                self.finished = True
                return

            except (ValueError, IndexError) as e:
                print(f"Ignoring nonmatching plan: {e}")


rclpy.init()
node = InsertCapture()

try:
    print("Waiting for V1 INSERT MoveIt plan...", flush=True)

    deadline = time.monotonic() + 120

    while (
        rclpy.ok()
        and not node.finished
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.25)

    if not node.finished:
        print("No matching INSERT trajectory captured.")

finally:
    node.destroy_node()
    rclpy.shutdown()
