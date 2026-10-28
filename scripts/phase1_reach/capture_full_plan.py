#!/usr/bin/env python3
"""Read-only ROS subscriber: capture FULL reach MoveIt joint plan and unwrap to home.

Wait until the PLAN-ONLY C++ planner publishes /display_planned_path. Uses the
saved demonstration home, NOT a live robot position. No ROS action or service
for moving hardware is used. The output is a candidate, not an approved path.
"""
import argparse
import csv
import math
import time
from pathlib import Path

JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path,
                   default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    p.add_argument('--output', type=Path)
    p.add_argument('--min-points', type=int, default=150,
                   help='Full V1 reach previously produced 195; reject short preview plans')
    p.add_argument('--timeout', type=float, default=90.0)
    a = p.parse_args()
    root = a.project.expanduser().resolve()
    output = a.output or root/'demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv'
    home_file = root/'config/saved_poses/v1_home_for_moveit.yaml'
    import yaml
    home = yaml.safe_load(home_file.read_text())
    names = home['arm']['joint_names']
    assert names == JOINTS, 'Home arm joint order does not match standard order'
    home_q = [float(v) for v in home['arm']['positions_rad']]
    if output.exists():
        raise SystemExit(f'Refusing to overwrite {output}')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from moveit_msgs.msg import DisplayTrajectory

    class Collector(Node):
        def __init__(self):
            super().__init__('phase1_v1_full_plan_capture')
            self.done = False
            self.create_subscription(
                DisplayTrajectory, '/display_planned_path', self.on_plan,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))

        def on_plan(self, msg):
            if not msg.trajectory:
                return
            trajectory = msg.trajectory[0].joint_trajectory
            if len(trajectory.points) < a.min_points:
                self.get_logger().info('Ignoring other trajectory with %d points' %
                                       len(trajectory.points))
                return
            try:
                if not all(j in trajectory.joint_names for j in JOINTS):
                    raise ValueError('Missing six arm joint names')
                indexes = [trajectory.joint_names.index(j) for j in JOINTS]
                t = [q.time_from_start.sec + q.time_from_start.nanosec*1e-9
                     for q in trajectory.points]
                raw = [[float(q.positions[i]) for i in indexes]
                       for q in trajectory.points]
                if not all(math.isfinite(x) for x in t + [v for row in raw for v in row]):
                    raise ValueError('Non-finite trajectory numbers')
                if t[0] < -1e-9 or any(b <= x for x,b in zip(t,t[1:])):
                    raise ValueError('Invalid trajectory timestamps')
                shifts = [2*math.pi * round((home_q[i]-raw[0][i])/(2*math.pi))
                          for i in range(6)]
                q = [[v+shifts[i] for i,v in enumerate(row)] for row in raw]
                start_error = max(abs(v-h) for v,h in zip(q[0],home_q))
                if start_error >= 0.015:
                    raise ValueError('Start differs from taught home by %.6f rad' % start_error)
                max_step = max(abs(b[i]-x[i]) for x,b in zip(q,q[1:]) for i in range(6))
                if max_step >= 0.05:
                    raise ValueError('Large consecutive joint step: %.5f rad' % max_step)
                output.parent.mkdir(parents=True,exist_ok=True)
                with output.open('x',newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time']+JOINTS)
                    for timestamp, positions in zip(t,q):
                        writer.writerow([f'{timestamp:.9f}']+[f'{v:.12f}' for v in positions])
                print('\nFULL V1 PLAN CANDIDATE SAVED:', output, flush=True)
                print('Waypoints:',len(q),'Duration:',round(t[-1]-t[0],3),'s',flush=True)
                print('Start difference from saved home:', round(start_error,8),'rad',flush=True)
                print('Branch shifts:',[round(v,5) for v in shifts],flush=True)
                print('Max joint step:',round(max_step,6),'rad',flush=True)
                print('End joints:',[round(v,5) for v in q[-1]],flush=True)
                print('NO ROBOT MOVEMENT COMMANDED; OFFLINE VALIDATION STILL REQUIRED.',flush=True)
            except Exception as exc:
                print('FAILED TO CAPTURE FULL PLAN:',exc,flush=True)
            self.done = True

    rclpy.init()
    node = Collector()
    try:
        print('Listening for full V1 reach PLAN-ONLY result...')
        deadline = time.monotonic()+a.timeout
        while rclpy.ok() and not node.done and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.5)
        if not node.done:
            raise SystemExit('Timeout: no full V1 reach trajectory was received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
