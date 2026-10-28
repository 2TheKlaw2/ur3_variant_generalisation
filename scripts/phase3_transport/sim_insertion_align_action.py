#!/usr/bin/env python3
"""Execute the previously reviewed V1 insertion-ready alignment in FAKE UR3e hardware ONLY.

Prerequisites:
 - Successful align_to_insertion_plan_only run (collision-checked, plan-only).
 - Fake robot remains at the learned transport endpoint, with V1 gripper near 9 mm.
 - This script does not plan, reposition, return home, or command a physical robot.
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'phase1_reach'))
from sim_fake_guard import await_result, verify_fake_only

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
PLAN = Path('/tmp/v1_align_to_insertion_joint_plan_001.csv')
TRANSPORT = ROOT / ('demonstrations/processed/phase3_transport/'
                    'v1_transport_joint_sim_candidate.csv')
DEFAULT_LOG = ROOT / 'results/simulation/v1_insertion_align_sim_actual_001.csv'
JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def read_csv(path):
    if not path.is_file():
        raise RuntimeError(f'Missing joint CSV: {path}')
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != ['time', *JOINTS]:
            raise RuntimeError(f'Unexpected joint CSV header in {path}: {reader.fieldnames}')
        rows = list(reader)
    if not rows:
        raise RuntimeError(f'Empty joint trajectory: {path}')
    t = [float(r['time']) for r in rows]
    q = [[float(r[j]) for j in JOINTS] for r in rows]
    if any(not math.isfinite(v) for v in t + [x for row in q for x in row]):
        raise RuntimeError(f'Nonfinite plan data: {path}')
    return t, q


def check_plan(slowdown):
    t, q = read_csv(PLAN)
    rt, rq = read_csv(TRANSPORT)
    del rt
    if not 5 <= len(q) <= 50:
        raise RuntimeError(f'Unexpected short alignment point count: {len(q)}')
    if not 0 <= t[0] < 0.15 or not 0.1 < t[-1] <= 8:
        raise RuntimeError('Unexpected short alignment timing')
    if any(b <= a for a, b in zip(t, t[1:])):
        raise RuntimeError('Non-increasing joint waypoint time')
    if max(abs(a-b) for a, b in zip(q[0], rq[-1])) > 0.015:
        raise RuntimeError('Plan does NOT start at the validated transport endpoint')
    max_total = max(abs(a-b) for a, b in zip(q[0], q[-1]))
    if max_total > 0.25:
        raise RuntimeError(f'Alignment plan is not a short joint motion: {max_total:.4f} rad')
    max_step = max(abs(b[i]-a[i]) for a, b in zip(q, q[1:]) for i in range(6))
    if max_step > 0.05:
        raise RuntimeError(f'Joint discontinuity: {max_step:.5f} rad')
    if any(abs(x) > 2*math.pi+0.1 for row in q for x in row):
        raise RuntimeError('Joint value exceeds numerical review limit')
    dt = [(b-a)*slowdown for a, b in zip(t,t[1:])]
    if min(dt) <= 0.0001:
        raise RuntimeError('Invalid waypoint timing')
    for i, name in enumerate(JOINTS):
        v = [(b[i]-a[i])/d for a,b,d in zip(q,q[1:],dt)]
        accel = [(v[k+1]-v[k])/(0.5*(dt[k]+dt[k+1])) for k in range(len(v)-1)]
        if max(map(abs, v)) > 0.3 or (accel and max(map(abs,accel)) > 0.4):
            raise RuntimeError('Velocity/acceleration too high for fake short alignment: '+name)
    return t, q


class Monitor(Node):
    def __init__(self):
        super().__init__('v1_insertion_alignment_fake_only')
        self.joints = {}
        self.received = 0.0
        self.recording = False
        self.start_time = 0.0
        self.samples = []
        self.create_subscription(JointState, '/joint_states', self.on_joint_state,
                                 qos_profile_sensor_data)

    def on_joint_state(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.received = time.monotonic()
        if self.recording and all(n in self.joints for n in JOINTS):
            self.samples.append([self.received - self.start_time] +
                                [self.joints[n] for n in JOINTS] +
                                [self.joints.get('finger_width', float('nan'))])

    def fresh(self):
        self.received = 0.0
        until = time.monotonic() + 6.0
        while self.received == 0 and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.received == 0 or time.monotonic() - self.received > 0.5:
            raise RuntimeError('No fresh fake /joint_states')
        if any(n not in self.joints for n in JOINTS):
            raise RuntimeError('Missing simulated arm joints')
        return [self.joints[n] for n in JOINTS]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--slowdown', type=float, default=2.0)
    p.add_argument('--log', type=Path, default=DEFAULT_LOG)
    args = p.parse_args()
    if not 1 <= args.slowdown <= 5:
        p.error('Slowdown must be in [1, 5]')
    if args.log.exists():
        p.error('Log already exists, refusing to overwrite: '+str(args.log))
    t, q = check_plan(args.slowdown)
    print('\nFAKE V1 INSERTION-READY ALIGNMENT')
    print(f'Joint points: {len(q)}, duration at slowdown {args.slowdown:g}: {t[-1]*args.slowdown:.3f} s')
    print('No pre-positioning, no return home, no physical robot command.')
    rclpy.init()
    node = Monitor()
    goal_handle = None
    try:
        verify_fake_only(node)
        live = node.fresh()
        delta = max(abs(a-b) for a,b in zip(live,q[0]))
        print(f'Current fake start error: {delta:.7f} rad')
        if delta > 0.015:
            raise RuntimeError('Virtual UR3e is NOT at completed transport endpoint')
        width = node.joints.get('finger_width')
        if width is None or not 0.007 <= width <= 0.012:
            raise RuntimeError('V1 fake gripper must remain near its 9 mm setting')
        print(f'Fake RG2 width: {1000*width:.2f} mm')
        action = ActionClient(node, FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')
        if not action.wait_for_server(timeout_sec=7):
            raise RuntimeError('Fake arm trajectory action unavailable')
        print('This action moves ONLY the virtual robot joints.')
        if input('Type ALIGN to simulate insertion alignment: ').strip() != 'ALIGN':
            raise RuntimeError('Not confirmed; no movement')
        verify_fake_only(node)
        live = node.fresh()
        if max(abs(a-b) for a,b in zip(live,q[0])) > 0.015:
            raise RuntimeError('Virtual UR3e moved before execution')
        width = node.joints.get('finger_width')
        if width is None or not 0.007 <= width <= 0.012:
            raise RuntimeError('Gripper state changed before execution')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for sec, joints in zip(t,q):
            ns = round(sec*args.slowdown*1e9)
            pt = JointTrajectoryPoint()
            pt.positions = list(joints)
            pt.time_from_start = Duration(sec=ns//1_000_000_000,
                                          nanosec=ns%1_000_000_000)
            goal.trajectory.points.append(pt)
        node.start_time = time.monotonic()
        node.recording = True
        goal_handle = await_result(node, action.send_goal_async(goal), 10)
        if not goal_handle.accepted:
            raise RuntimeError('Fake controller rejected insertion alignment goal')
        res = await_result(node, goal_handle.get_result_async(),
                           max(15, 4*t[-1]*args.slowdown+10))
        node.recording = False
        if res.status != GoalStatus.STATUS_SUCCEEDED or res.result.error_code != 0:
            raise RuntimeError('Fake insertion alignment action failed: '+res.result.error_string)
        end = node.fresh()
        error = max(abs(a-b) for a,b in zip(end,q[-1]))
        print(f'FAKE INSERTION ALIGNMENT SUCCESS. Max endpoint joint error: {error:.7f} rad')
        if error > 0.015:
            raise RuntimeError('Fake insertion alignment endpoint mismatch')
        print('Insertion-ready alignment reached in mock arm state; physical clearance NOT verified.')
    except Exception as exc:
        print('SIM INSERTION ALIGNMENT STOPPED:', exc)
        if goal_handle is not None and goal_handle.accepted:
            try:
                await_result(node, goal_handle.cancel_goal_async(), 4)
            except Exception:
                print('Goal cancellation unconfirmed; inspect mock controller')
        raise SystemExit(1)
    finally:
        node.recording = False
        if node.samples:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            with args.log.open('x', newline='') as f:
                wr = csv.writer(f)
                wr.writerow(['elapsed_s', *JOINTS, 'finger_width'])
                wr.writerows(node.samples)
            print('Saved mock joint state log:', args.log)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
