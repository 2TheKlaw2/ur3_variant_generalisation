#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

ALIGN_FILE = ROOT / (
    "config/trajectories/v1/"
    "v1_align_to_grip_joint_plan_001.csv"
)

REACH_FILE = ROOT / (
    "demonstrations/processed/"
    "v1_reach_full_joint_sim_candidate_001.csv"
)

GRIP_POSE = ROOT / "config/saved_poses/v1_grip_pose.yaml"

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

TIME_SCALE = 8.0

def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

def positions(row):
    return [float(row[j]) for j in JOINTS]

def maximum_error(a, b):
    return max(abs(x-y) for x, y in zip(a, b))

align = read_csv(ALIGN_FILE)
reach = read_csv(REACH_FILE)

with GRIP_POSE.open() as f:
    saved_pose = yaml.safe_load(f)

assert len(align) == 12, "Unexpected ALIGN trajectory"
assert saved_pose["arm"]["joint_names"] == JOINTS

times = [float(r["time"]) for r in align]
qs = [positions(r) for r in align]

assert all(math.isfinite(v) for q in qs for v in q)
assert all(b > a for a, b in zip(times, times[1:]))

reach_end = positions(reach[-1])
grip_target = saved_pose["arm"]["positions_rad"]

handoff_error = maximum_error(qs[0], reach_end)
target_error = maximum_error(qs[-1], grip_target)

max_step = max(
    maximum_error(a, b)
    for a, b in zip(qs, qs[1:])
)

max_speed = max(
    abs(qs[i+1][j]-qs[i][j])
    / ((times[i+1]-times[i])*TIME_SCALE)
    for i in range(len(qs)-1)
    for j in range(6)
)

print("\n===== V1 PHYSICAL PICKUP ALIGN =====")
print("Trajectory points:", len(qs))
print("Command duration:", round(times[-1]*TIME_SCALE, 3), "s")
print("REACH handoff error:", round(handoff_error, 6), "rad")
print("Saved grip pose error:", round(target_error, 6), "rad")
print("Maximum joint step:", round(max_step, 6), "rad")
print("Maximum commanded speed:", round(max_speed, 6), "rad/s")

assert handoff_error < 0.002, "REACH/ALIGN mismatch"
assert target_error < 0.002, "ALIGN/grip-pose mismatch"
assert max_step < 0.03, "Large joint step"
assert max_speed < 0.05, "Unexpected joint speed"

rclpy.init()
node = rclpy.create_node("v1_physical_pickup_align")

controller_client = node.create_client(
    ListControllers, "/controller_manager/list_controllers"
)

assert controller_client.wait_for_service(timeout_sec=8.0)

future = controller_client.call_async(ListControllers.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

assert future.result() is not None, "Cannot inspect controllers"

states = {
    c.name: c.state
    for c in future.result().controller
}

for name in (
    "joint_state_broadcaster",
    "scaled_joint_trajectory_controller",
    "finger_width_trajectory_controller",
):
    assert states.get(name) == "active", \
        f"Controller inactive: {name}"

latest = {"msg": None}

node.create_subscription(
    JointState,
    "/joint_states",
    lambda msg: latest.update(msg=msg),
    qos_profile_sensor_data,
)

def read_actual():
    latest["msg"] = None

    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)
        if latest["msg"] is not None:
            break

    assert latest["msg"] is not None, "No robot feedback"

    msg = latest["msg"]
    return dict(zip(msg.name, msg.position))

actual = read_actual()

assert all(j in actual for j in JOINTS), "Missing arm feedback"

current_q = [actual[j] for j in JOINTS]
start_error = maximum_error(current_q, qs[0])

width = actual.get("finger_width", float("nan"))

print("\n===== CURRENT PHYSICAL ROBOT =====")
print("REACH endpoint error:", round(start_error, 6), "rad")
print("Gripper width:", round(width*1000, 2), "mm")

assert start_error < 0.007, \
    "Robot is NOT at the expected REACH endpoint"

assert math.isfinite(width) and 0.015 <= width <= 0.030, \
    "Unexpected gripper opening"

trajectory = JointTrajectory()
trajectory.joint_names = JOINTS

for i, q in enumerate(qs):
    point = JointTrajectoryPoint()
    point.positions = q

    if i == 0 or i == len(qs)-1:
        point.velocities = [0.0]*6
    else:
        dt = (times[i+1]-times[i-1])*TIME_SCALE
        point.velocities = [
            (qs[i+1][j]-qs[i-1][j])/dt
            for j in range(6)
        ]

    elapsed = times[i]*TIME_SCALE + 0.3
    ns = round(elapsed*1e9)
    sec, nanosec = divmod(ns, 1_000_000_000)

    point.time_from_start = Duration(
        sec=sec,
        nanosec=nanosec,
    )

    trajectory.points.append(point)

client = ActionClient(
    node,
    FollowJointTrajectory,
    "/scaled_joint_trajectory_controller/follow_joint_trajectory",
)

assert client.wait_for_server(timeout_sec=8.0), \
    "Physical arm controller unavailable"

print("\n===== PREFLIGHT PASSED =====")
print("Stage: REACH endpoint -> V1 pickup ALIGN")
print("The arm WILL MOVE. The gripper will NOT close.")
print("Environmental collision clearance is not checked by this script.")
print("Confirm the swept path is clear and the operator is at the pendant.")

confirmation = input("\nType ALIGN to move the PHYSICAL robot: ").strip()

if confirmation != "ALIGN":
    print("CANCELLED")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0)

actual = read_actual()
current_q = [actual[j] for j in JOINTS]

assert maximum_error(current_q, qs[0]) < 0.007, \
    "Robot moved before execution"

goal = FollowJointTrajectory.Goal()
goal.trajectory = trajectory
goal.goal_time_tolerance = Duration(sec=30)

sent = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)

assert sent.result() is not None, "Action server did not respond"

handle = sent.result()
assert handle.accepted, "ALIGN goal rejected"

print("\nALIGN ACCEPTED — physical robot moving")

result_future = handle.get_result_async()

try:
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.2)
except KeyboardInterrupt:
    print("Cancellation requested — use pendant Stop if motion continues")
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
    raise SystemExit(1)

result = result_future.result()

print("\n===== ALIGN RESULT =====")
print("Goal status:", result.status)
print("Error code:", result.result.error_code)
print("Message:", result.result.error_string)

if (
    result.status != GoalStatus.STATUS_SUCCEEDED
    or result.result.error_code != 0
):
    raise SystemExit("ALIGN FAILED — do not grip")

actual = read_actual()
final_q = [actual[j] for j in JOINTS]
final_error = maximum_error(final_q, grip_target)

print("Final joint error:", round(final_error, 6), "rad")

if final_error < 0.007:
    print("\nPHYSICAL V1 PICKUP ALIGN: PASS")
else:
    print("\nALIGN ENDPOINT NOT VERIFIED — DO NOT GRIP")

node.destroy_node()
rclpy.shutdown()
