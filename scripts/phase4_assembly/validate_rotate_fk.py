#!/usr/bin/env python3

import csv
import math
import os
import time
from pathlib import Path

if os.environ.get("ROS_DOMAIN_ID") != "73":
    raise SystemExit("BLOCKED: Require ROS_DOMAIN_ID=73")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

PLAN = ROOT / (
    "demonstrations/processed/phase4_assembly/"
    "v1_rotate_joint_sim_candidate.csv"
)

DEMO = ROOT / "demonstrations/processed/v1_rotate_001_motion.csv"

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def load(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def quat_angle(a, b):
    dot = abs(sum(x*y for x, y in zip(a, b)))

    norm = math.sqrt(
        sum(x*x for x in a) *
        sum(x*x for x in b)
    )

    if norm == 0:
        raise RuntimeError("Invalid quaternion")

    return math.degrees(
        2 * math.acos(min(1.0, max(0.0, dot/norm)))
    )


def pose_quat(p):
    return [
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ]


plan = load(PLAN)
demo = load(DEMO)

if len(plan) != 71:
    raise SystemExit("BLOCKED: Unexpected ROTATE joint count")

times = [float(r["time"]) for r in plan]

q = [
    [float(r[j]) for j in JOINTS]
    for r in plan
]

if any(
    not math.isfinite(v)
    for row in q for v in row
):
    raise SystemExit("BLOCKED: Non-finite joint values")

if any(b <= a for a, b in zip(times, times[1:])):
    raise SystemExit("BLOCKED: Invalid timestamps")

if q[0][5] > -3:
    raise SystemExit("BLOCKED: Wrist 3 branch incorrect")

max_step = max(
    abs(b[j]-a[j])
    for a, b in zip(q, q[1:])
    for j in range(6)
)

if max_step > 0.04:
    raise SystemExit("BLOCKED: Joint discontinuity")


rclpy.init()
node = Node("v1_rotate_fk_readonly_check")

live = {"q": None}


def joint_callback(msg):
    if all(j in msg.name for j in JOINTS):
        live["q"] = [
            float(msg.position[msg.name.index(j)])
            for j in JOINTS
        ]


subscription = node.create_subscription(
    JointState,
    "/joint_states",
    joint_callback,
    qos_profile_sensor_data,
)

client = node.create_client(
    GetPositionFK,
    "/compute_fk",
)

try:
    deadline = time.monotonic() + 5

    while live["q"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if live["q"] is None:
        raise RuntimeError("No live joint states")

    start_error = max(
        abs(a-b)
        for a, b in zip(live["q"], q[0])
    )

    print(f"\nLive ROTATE start error: {start_error:.6f} rad")

    if start_error > 0.002:
        raise RuntimeError(
            "Robot is not at ROTATE starting pose"
        )

    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError("MoveIt FK service unavailable")

    # Include intermediate joint configurations.
    samples = []

    for i in range(len(q)-1):
        for k in range(5):
            fraction = k / 5

            samples.append([
                a + (b-a)*fraction
                for a, b in zip(q[i], q[i+1])
            ])

    samples.append(q[-1])

    poses = []

    for joints in samples:
        request = GetPositionFK.Request()

        request.header.frame_id = "base"
        request.fk_link_names = ["gripper_tcp"]

        request.robot_state.joint_state.name = JOINTS
        request.robot_state.joint_state.position = joints

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=3,
        )

        if not future.done() or future.result() is None:
            raise RuntimeError("FK request failed")

        result = future.result()

        if result.error_code.val != 1 or not result.pose_stamped:
            raise RuntimeError(
                f"FK error: {result.error_code.val}"
            )

        poses.append(result.pose_stamped[0].pose)

    xyz = [
        [p.position.x, p.position.y, p.position.z]
        for p in poses
    ]

    start = xyz[0]
    end = xyz[-1]

    xy_drift = max(
        math.hypot(
            p[0]-start[0],
            p[1]-start[1],
        )
        for p in xyz
    )

    z_drift = max(
        abs(p[2]-start[2])
        for p in xyz
    )

    min_z = min(p[2] for p in xyz)

    demo_min_z = min(
        float(r["tcp_z"])
        for r in demo
    )

    demo_start = [
        float(demo[0]["tcp_"+a])
        for a in "xyz"
    ]

    demo_end = [
        float(demo[-1]["tcp_"+a])
        for a in "xyz"
    ]

    demo_start_q = [
        float(demo[0]["tcp_q"+a])
        for a in "xyzw"
    ]

    demo_end_q = [
        float(demo[-1]["tcp_q"+a])
        for a in "xyzw"
    ]

    actual_start_q = pose_quat(poses[0])
    actual_end_q = pose_quat(poses[-1])

    actual_rotation = quat_angle(
        actual_start_q,
        actual_end_q,
    )

    demo_rotation = quat_angle(
        demo_start_q,
        demo_end_q,
    )

    start_pos_error = math.dist(
        start, demo_start
    ) * 1000

    end_pos_error = math.dist(
        end, demo_end
    ) * 1000

    start_orient_error = quat_angle(
        actual_start_q, demo_start_q
    )

    end_orient_error = quat_angle(
        actual_end_q, demo_end_q
    )

    print("\n========== V1 ROTATE FK VALIDATION ==========")

    print(f"Joint trajectory points: {len(q)}")
    print(f"FK samples: {len(samples)}")
    print(f"Planned duration: {times[-1]:.3f} s")

    print(f"\nDemonstrated rotation: {demo_rotation:.3f} deg")
    print(f"Planned TCP rotation: {actual_rotation:.3f} deg")

    print(f"\nMaximum XY drift: {xy_drift*1000:.4f} mm")
    print(f"Maximum Z drift: {z_drift*1000:.4f} mm")
    print(f"Minimum TCP height: {min_z*1000:.4f} mm")
    print(f"Human minimum height: {demo_min_z*1000:.4f} mm")

    print(f"\nStart position error: {start_pos_error:.4f} mm")
    print(f"End position error: {end_pos_error:.4f} mm")
    print(f"Start orientation error: {start_orient_error:.4f} deg")
    print(f"End orientation error: {end_orient_error:.4f} deg")

    checks = {
        "Rotation magnitude":
            abs(actual_rotation-demo_rotation) <= 0.5,

        "XY drift":
            xy_drift <= 0.001,

        "Z drift":
            z_drift <= 0.00075,

        "Minimum Z":
            min_z >= demo_min_z-0.00005,

        "Start position":
            start_pos_error <= 0.3,

        "Endpoint position":
            end_pos_error <= 0.3,

        "Start orientation":
            start_orient_error <= 0.5,

        "Endpoint orientation":
            end_orient_error <= 0.5,
    }

    print("\n========== CHECKS ==========")

    for name, passed in checks.items():
        print(f"{name}: {'PASS' if passed else 'FAIL'}")

    print(
        "\nROTATE FK VALIDATION:",
        "PASS" if all(checks.values()) else "FAIL"
    )

    print("NO ROBOT MOVEMENT.")
    print("NOT PHYSICAL EXECUTION APPROVAL.")

finally:
    node.destroy_node()
    rclpy.shutdown()
