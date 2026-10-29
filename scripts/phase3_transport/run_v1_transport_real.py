#!/usr/bin/env python3

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from action_msgs.msg import GoalStatus

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"

CSV_FILE = ROOT / (
    "demonstrations/processed/phase3_transport/"
    "v1_transport_joint_sim_candidate.csv"
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

TIME_SCALE = 2.0

def max_error(a, b):
    return max(abs(x-y) for x, y in zip(a, b))

with CSV_FILE.open() as f:
    rows = list(csv.DictReader(f))

with GRIP_POSE.open() as f:
    grip_pose = yaml.safe_load(f)

assert len(rows) == 246, "Unexpected transport trajectory"
assert grip_pose["arm"]["joint_names"] == JOINTS

times = [float(r["time"]) for r in rows]
positions = [
    [float(r[j]) for j in JOINTS]
    for r in rows
]

assert all(math.isfinite(t) for t in times)
assert all(math.isfinite(v) for q in positions for v in q)
assert all(b > a for a, b in zip(times, times[1:]))

grip_q = grip_pose["arm"]["positions_rad"]

handoff_error = max_error(positions[0], grip_q)

max_step = max(
    max_error(a, b)
    for a, b in zip(positions, positions[1:])
)

max_speed = max(
    abs(positions[i+1][j] - positions[i][j])
    / ((times[i+1]-times[i])*TIME_SCALE)
    for i in range(len(rows)-1)
    for j in range(6)
)

print("\n========== PHYSICAL V1 TRANSPORT ==========")
print("Trajectory samples:", len(rows))
print("Original duration:", round(times[-1]-times[0], 3), "s")
print("Command duration:", round((times[-1]-times[0])*TIME_SCALE, 3), "s")
print("Grip-to-transport handoff:", round(handoff_error, 6), "rad")
print("Maximum joint step:", round(max_step, 6), "rad")
print("Maximum commanded speed:", round(max_speed, 6), "rad/s")

assert handoff_error < 0.005, "Saved grip pose mismatch"
assert max_step < 0.025, "Large joint step"
assert max_speed < 0.12, "Unexpected transport speed"

rclpy.init()
node = rclpy.create_node("v1_physical_transport")

# Verify physical controllers.
controllers = node.create_client(
    ListControllers,
    "/controller_manager/list_controllers",
)

assert controllers.wait_for_service(timeout_sec=8.0), \
    "Controller manager unavailable"

future = controllers.call_async(ListControllers.Request())
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
        f"Controller not active: {name}"

# Receive and record physical joint feedback.
latest = {"msg": None}
samples = []
record = {"enabled": False}

def on_joint_state(msg):
    latest["msg"] = msg

    if record["enabled"]:
        actual = dict(zip(msg.name, msg.position))

        if all(j in actual for j in JOINTS):
            samples.append(
                [
                    time.monotonic() - record["start"],
                    actual.get("finger_width", float("nan")),
                ]
                + [actual[j] for j in JOINTS]
            )

subscription = node.create_subscription(
    JointState,
    "/joint_states",
    on_joint_state,
    qos_profile_sensor_data,
)

def read_actual():
    latest["msg"] = None
    deadline = time.monotonic() + 8.0

    while latest["msg"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    assert latest["msg"] is not None, "No physical joint feedback"

    return dict(zip(
        latest["msg"].name,
        latest["msg"].position,
    ))

actual = read_actual()

assert all(j in actual for j in JOINTS), \
    "Missing physical arm joints"

actual_q = [actual[j] for j in JOINTS]
start_error = max_error(actual_q, positions[0])

width = actual.get("finger_width", float("nan"))

print("\n===== PHYSICAL START CHECK =====")
print("Transport start error:", round(start_error, 6), "rad")
print("Measured gripper width:", round(width*1000, 2), "mm")

assert start_error < 0.007, \
    "Robot is not at pickup ALIGN endpoint"

assert math.isfinite(width) and 0.005 <= width <= 0.025, \
    "Gripper does not report a plausible grip width"

# Build the saved learned trajectory.
trajectory = JointTrajectory()
trajectory.joint_names = JOINTS

t0 = times[0]

for i, q in enumerate(positions):
    point = JointTrajectoryPoint()
    point.positions = q

    if i == 0 or i == len(rows)-1:
        point.velocities = [0.0]*6
    else:
        dt = (times[i+1]-times[i-1])*TIME_SCALE
        point.velocities = [
            (positions[i+1][j]-positions[i-1][j])/dt
            for j in range(6)
        ]

    elapsed = (times[i]-t0)*TIME_SCALE + 0.3
    ns = round(elapsed * 1e9)
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
    "Physical arm action unavailable"

print("\n===== PREFLIGHT PASSED =====")
print("STAGE: V1 PICKUP -> LEARNED TRANSPORT")
print("The physical robot WILL MOVE with the component.")
print("Gripper opening is not a measurement of holding force.")
print("The script does not check environmental collisions.")
print("Ensure the complete swept path is clear.")
print("Keep the operator at the pendant, ready to STOP.")
print("No insertion or rotation will follow.")

confirmation = input(
    "\nType TRANSPORT to move the physical UR3e: "
).strip()

if confirmation != "TRANSPORT":
    print("CANCELLED — no movement")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0)

# Recheck physical state immediately before sending.
actual = read_actual()
actual_q = [actual[j] for j in JOINTS]
width = actual.get("finger_width", float("nan"))

assert max_error(actual_q, positions[0]) < 0.007, \
    "Robot moved before transport"

assert math.isfinite(width) and 0.005 <= width <= 0.025, \
    "Gripper state changed before transport"

goal = FollowJointTrajectory.Goal()
goal.trajectory = trajectory
goal.goal_time_tolerance = Duration(sec=30)

print("\nSENDING PHYSICAL TRANSPORT...")

sent = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)

assert sent.result() is not None, "Action server unavailable"

handle = sent.result()
assert handle.accepted, "Transport goal rejected"

print("TRANSPORT ACCEPTED — monitoring physical movement")

record["start"] = time.monotonic()
record["enabled"] = True

result_future = handle.get_result_async()

try:
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.2)

except KeyboardInterrupt:
    print("\nCancellation requested.")
    print("Use pendant Stop if physical movement continues.")
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
    raise SystemExit(1)

finally:
    record["enabled"] = False

result = result_future.result()

print("\n===== TRANSPORT RESULT =====")
print("Goal status:", result.status)
print("Error code:", result.result.error_code)
print("Message:", result.result.error_string)

# Save actual physical execution data, including failed runs.
log_dir = ROOT / "results/physical/v1_transport"
log_dir.mkdir(parents=True, exist_ok=True)

log_file = log_dir / (
    "v1_transport_" +
    datetime.now().strftime("%Y%m%d_%H%M%S") +
    ".csv"
)

with log_file.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(
        ["elapsed_s", "finger_width_m"] + JOINTS
    )
    writer.writerows(samples)

print("Physical joint log:", log_file)

if (
    result.status != GoalStatus.STATUS_SUCCEEDED
    or result.result.error_code != 0
):
    raise SystemExit("TRANSPORT FAILED — do not continue")

actual = read_actual()
actual_q = [actual[j] for j in JOINTS]

final_error = max_error(
    actual_q,
    positions[-1],
)

print("Final joint error:", round(final_error, 6), "rad")
print("Final gripper width:",
      round(actual.get("finger_width", float("nan"))*1000, 2),
      "mm")

if final_error < 0.007:
    print("\nPHYSICAL V1 TRANSPORT: PASS")
else:
    print("\nENDPOINT ERROR — DO NOT CONTINUE")

node.destroy_node()
rclpy.shutdown()
