#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from action_msgs.msg import GoalStatus

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

CSV_PATH = (
    ROOT / "demonstrations/processed/"
    "v1_reach_full_joint_sim_candidate_001.csv"
)

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CONTROLLER = (
    "/scaled_joint_trajectory_controller/follow_joint_trajectory"
)

TIME_SCALE = 2.0

with CSV_PATH.open() as f:
    rows = list(csv.DictReader(f))

assert len(rows) == 195, "Unexpected number of trajectory samples"

times = [float(r["time"]) for r in rows]
positions = [[float(r[j]) for j in JOINTS] for r in rows]

assert all(
    math.isfinite(v) for row in positions for v in row
), "Invalid trajectory values"

assert all(
    b > a for a, b in zip(times, times[1:])
), "Non-increasing trajectory times"

segment_velocities = [
    [
        (positions[i+1][j] - positions[i][j])
        / (times[i+1] - times[i])
        for j in range(6)
    ]
    for i in range(len(rows)-1)
]

max_step = max(
    abs(positions[i+1][j] - positions[i][j])
    for i in range(len(rows)-1)
    for j in range(6)
)

max_speed = max(
    abs(v) for row in segment_velocities for v in row
) / TIME_SCALE

max_acceleration = max(
    abs(segment_velocities[i+1][j] - segment_velocities[i][j])
    / ((times[i+2] - times[i]) / 2)
    for i in range(len(segment_velocities)-1)
    for j in range(6)
) / (TIME_SCALE ** 2)

print("\n===== PHYSICAL V1 CDMP REACH =====")
print("Samples:", len(rows))
print("Original duration:", round(times[-1]-times[0], 3), "s")
print("Command duration:", round((times[-1]-times[0])*TIME_SCALE, 3), "s")
print("Maximum joint step:", round(max_step, 5), "rad")
print("Maximum commanded speed:", round(max_speed, 5), "rad/s")
print("Approx. sampled acceleration:", round(max_acceleration, 5), "rad/s²")

assert max_step < 0.025, "Joint discontinuity detected"
assert max_speed < 0.12, "Joint velocity exceeds test limit"
assert max_acceleration < 1.0, "Acceleration exceeds test limit"

rclpy.init()
node = rclpy.create_node("physical_v1_cdmp_reach")

# Confirm controller states.
controller_client = node.create_client(
    ListControllers, "/controller_manager/list_controllers"
)

assert controller_client.wait_for_service(timeout_sec=8.0), \
    "Controller manager unavailable"

future = controller_client.call_async(ListControllers.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

assert future.result() is not None, "Cannot read controllers"

controllers = {
    c.name: c.state
    for c in future.result().controller
}

for required in [
    "joint_state_broadcaster",
    "finger_width_trajectory_controller",
    "scaled_joint_trajectory_controller",
]:
    assert controllers.get(required) == "active", \
        f"{required} is not active"

print("Required controllers: ACTIVE")

# Read actual robot state.
state = {"msg": None}

node.create_subscription(
    JointState,
    "/joint_states",
    lambda msg: state.update(msg=msg),
    qos_profile_sensor_data,
)

deadline = time.monotonic() + 8

while state["msg"] is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

assert state["msg"] is not None, "No joint-state feedback"

msg = state["msg"]
actual = dict(zip(msg.name, msg.position))

assert all(j in actual for j in JOINTS), "Missing arm joints"

start_error = max(
    abs(actual[j] - positions[0][i])
    for i, j in enumerate(JOINTS)
)

width = actual.get("finger_width", float("nan"))

print("Actual HOME start error:", round(start_error, 6), "rad")
print("Actual gripper width:", round(width*1000, 2), "mm")

assert start_error < 0.015, "Robot is not at trajectory HOME"
assert math.isfinite(width) and 0.015 <= width <= 0.030, \
    "Gripper width is not in the expected open range"

# Create trajectory from the actual CDMP samples.
trajectory = JointTrajectory()
trajectory.joint_names = JOINTS

t0 = times[0]

for i, row in enumerate(positions):
    point = JointTrajectoryPoint()
    point.positions = row

    # Approximate smooth waypoint velocities.
    if i == 0 or i == len(rows)-1:
        point.velocities = [0.0]*6
    else:
        dt = (times[i+1]-times[i-1])*TIME_SCALE
        point.velocities = [
            (positions[i+1][j]-positions[i-1][j])/dt
            for j in range(6)
        ]

    elapsed = (times[i]-t0)*TIME_SCALE + 0.2
    ns = round(elapsed*1_000_000_000)
    sec, nanosec = divmod(ns, 1_000_000_000)

    point.time_from_start = Duration(
        sec=sec,
        nanosec=nanosec,
    )

    trajectory.points.append(point)

action = ActionClient(node, FollowJointTrajectory, CONTROLLER)

assert action.wait_for_server(timeout_sec=8.0), \
    "Scaled trajectory action unavailable"

print("\nPREFLIGHT PASSED")
print("IMPORTANT: Collision clearance is NOT verified by this script.")
print("Ensure workspace is clear and operator has pendant Stop access.")
print("Set pendant speed slider to 10-20%.")
print("This will physically move the UR3e through the learned REACH.")
print("No grip, transport, insertion or rotation will be commanded.")

confirmation = input("\nType RUN_PHYSICAL_V1_REACH to execute: ")

if confirmation != "RUN_PHYSICAL_V1_REACH":
    print("CANCELLED — robot will not move")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0)

goal = FollowJointTrajectory.Goal()
goal.trajectory = trajectory
goal.goal_time_tolerance = Duration(sec=10)

print("\nSENDING PHYSICAL REACH...")

sent = action.send_goal_async(goal)
rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)

assert sent.result() is not None, "No response from action server"

handle = sent.result()
assert handle.accepted, "Trajectory goal REJECTED"

print("Goal ACCEPTED — monitoring execution.")
print("Use pendant Stop immediately if unexpected motion occurs.")

result_future = handle.get_result_async()

try:
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.5)

except KeyboardInterrupt:
    print("\nCancellation requested.")
    print("Use pendant Stop to stop physical motion immediately.")
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

result = result_future.result()

print("\n===== PHYSICAL REACH RESULT =====")
print("Goal status:", result.status)
print("Controller error code:", result.result.error_code)
print("Message:", result.result.error_string)

if (
    result.status == GoalStatus.STATUS_SUCCEEDED
    and result.result.error_code == 0
):
    print("REACH CONTROLLER RESULT: PASS")
else:
    print("REACH CONTROLLER RESULT: FAIL")
    print("Do not proceed to pickup ALIGN.")

node.destroy_node()
rclpy.shutdown()
