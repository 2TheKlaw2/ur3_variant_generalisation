#!/usr/bin/env python3
import csv, math, os, time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '73':
    raise SystemExit('BLOCKED: ROS_DOMAIN_ID must be 73.')

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
PLAN = ROOT / 'demonstrations/processed/phase4_assembly/v1_insert_joint_sim_candidate.csv'
DEMO = ROOT / 'demonstrations/processed/v1_insert_001_motion.csv'

JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

with PLAN.open(newline='') as f:
    plan = list(csv.DictReader(f))
with DEMO.open(newline='') as f:
    demo = list(csv.DictReader(f))

if not 7 <= len(plan) <= 40:
    raise SystemExit('BLOCKED: Unexpected INSERT trajectory length.')

times = [float(r['time']) for r in plan]
q = [[float(r[j]) for j in JOINTS] for r in plan]

if any(not math.isfinite(x) for row in q for x in row):
    raise SystemExit('BLOCKED: Invalid joint position.')

if any(b <= a for a, b in zip(times, times[1:])):
    raise SystemExit('BLOCKED: Invalid trajectory timing.')

max_step = max(
    abs(b[j]-a[j])
    for a,b in zip(q,q[1:])
    for j in range(6)
)

if max_step > 0.05 or q[0][-1] > -3:
    raise SystemExit('BLOCKED: Joint jump or Wrist 3 mismatch.')

rclpy.init()
node = Node('v1_insert_fk_readonly_check')

live = {'q': None}

def callback(msg):
    if all(j in msg.name for j in JOINTS):
        live['q'] = [
            msg.position[msg.name.index(j)]
            for j in JOINTS
        ]

node.create_subscription(
    JointState, '/joint_states',
    callback, qos_profile_sensor_data
)

client = node.create_client(GetPositionFK, '/compute_fk')

try:
    deadline = time.monotonic() + 5

    while live['q'] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if live['q'] is None:
        raise RuntimeError('No live joint states received.')

    start_error = max(
        abs(a-b) for a,b in zip(live['q'],q[0])
    )

    print(f'Live start error: {start_error:.6f} rad')

    if start_error > 0.005:
        raise RuntimeError('Robot is not at INSERT starting pose.')

    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('/compute_fk service unavailable.')

    # Sample intermediate joint positions, not just waypoints.
    samples = []

    for i in range(len(q)-1):
        for k in range(10):
            fraction = k / 10
            samples.append([
                a + (b-a)*fraction
                for a,b in zip(q[i],q[i+1])
            ])

    samples.append(q[-1])
    poses = []

    for joints in samples:
        req = GetPositionFK.Request()
        req.header.frame_id = 'base'
        req.fk_link_names = ['gripper_tcp']
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = joints

        future = client.call_async(req)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=3
        )

        if not future.done() or future.result() is None:
            raise RuntimeError('FK request timed out.')

        reply = future.result()

        if reply.error_code.val != 1 or not reply.pose_stamped:
            raise RuntimeError(
                f'FK failed: {reply.error_code.val}'
            )

        poses.append(reply.pose_stamped[0].pose)

    xyz = [
        [p.position.x, p.position.y, p.position.z]
        for p in poses
    ]

    start, end = xyz[0], xyz[-1]
    lowest = min(p[2] for p in xyz)

    xy_drift = max(
        math.hypot(p[0]-start[0],p[1]-start[1])
        for p in xyz
    )

    demo_floor = min(float(r['tcp_z']) for r in demo)

    target_end = [
        float(demo[-1]['tcp_'+s]) for s in 'xyz'
    ]

    end_error = math.dist(end,target_end)

    quat = poses[-1].orientation
    q_end = [quat.x,quat.y,quat.z,quat.w]
    q_demo = [
        float(demo[-1]['tcp_q'+s]) for s in 'xyzw'
    ]

    dot = abs(sum(a*b for a,b in zip(q_end,q_demo)))
    norm = math.sqrt(
        sum(v*v for v in q_end) *
        sum(v*v for v in q_demo)
    )

    angle = math.degrees(
        2*math.acos(min(1,max(0,dot/norm)))
    )

    depth = (start[2]-end[2])*1000

    print('\n========== V1 INSERT FK VALIDATION ==========')
    print(f'Joint points: {len(q)}')
    print(f'FK samples: {len(xyz)}')
    print(f'Planned duration: {times[-1]:.3f} s')
    print(f'TCP insertion depth: {depth:.3f} mm')
    print(f'Lowest planned Z: {lowest*1000:.4f} mm')
    print(f'Demonstrated minimum Z: {demo_floor*1000:.4f} mm')
    print(f'Max XY drift: {xy_drift*1000:.4f} mm')
    print(f'Endpoint position error: {end_error*1000:.4f} mm')
    print(f'Endpoint orientation error: {angle:.4f} deg')

    checks = {
        'Depth limit': lowest >= demo_floor-0.00005,
        'Insertion depth': 4.8 <= depth <= 5.8,
        'XY drift': xy_drift <= 0.001,
        'Endpoint position': end_error <= 0.0003,
        'Endpoint orientation': angle <= 0.5
    }

    for name, passed in checks.items():
        print(f'{name}: {"PASS" if passed else "FAIL"}')

    print(
        '\nFK VALIDATION:',
        'PASS' if all(checks.values()) else 'FAIL'
    )
    print('NO ROBOT MOVEMENT.')
    print('NOT PHYSICAL EXECUTION APPROVAL.')

finally:
    node.destroy_node()
    rclpy.shutdown()
