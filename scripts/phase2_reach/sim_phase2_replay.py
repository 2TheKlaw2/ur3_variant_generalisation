#!/usr/bin/env python3
"""Phase 2 fake-hardware-only V1 adapted reach replay (never physical robot)."""
import argparse
import csv
import math
import time
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# Reuse Phase 1's tested fail-closed simulation guard from its sibling folder.
PHASE1_DIR = Path(__file__).resolve().parents[1] / 'phase1_reach'
if not (PHASE1_DIR / 'sim_fake_guard.py').is_file():
    raise RuntimeError('Phase 1 fake-hardware guard missing: ' + str(PHASE1_DIR))
sys.path.insert(0, str(PHASE1_DIR))
from sim_fake_guard import await_result, verify_fake_only

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']


class Watch(Node):
    def __init__(self):
        super().__init__('phase2_fake_adapted_reach')
        self.q = {}
        self.stamp = 0.0
        self.log_on = False
        self.start = 0.0
        self.logged = []
        self.create_subscription(JointState,'/joint_states',self.callback,qos_profile_sensor_data)

    def callback(self,msg):
        self.q = dict(zip(msg.name,msg.position))
        self.stamp = time.monotonic()
        if self.log_on and all(j in self.q for j in JOINTS):
            self.logged.append([self.stamp-self.start]+[self.q[j] for j in JOINTS])

    def fresh(self):
        self.stamp=0.0
        deadline=time.monotonic()+6
        while self.stamp==0.0 and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.1)
        if self.stamp==0 or not all(j in self.q for j in JOINTS):
            raise RuntimeError('Missing fresh fake arm joint states')
        return [float(self.q[j]) for j in JOINTS]


def validate_path(path,home_q,grip_q,slowdown):
    with path.open(newline='') as f:
        r=csv.DictReader(f)
        if r.fieldnames!=['time']+JOINTS:
            raise RuntimeError('Unexpected joint CSV header')
        records=list(r)
    if len(records)<150:
        raise RuntimeError('Expected complete learned V1 reach, not the short preview')
    t=[float(x['time']) for x in records]
    q=[[float(x[j]) for j in JOINTS] for x in records]
    if not all(math.isfinite(v) for v in t+[z for row in q for z in row]):
        raise RuntimeError('Non-finite joint values')
    if t[0]<0 or not 1<t[-1]-t[0]<180:
        raise RuntimeError('Unexpected time range')
    dt=[(b-a)*slowdown for a,b in zip(t,t[1:])]
    if min(dt)<=0.0001:
        raise RuntimeError('Non-increasing or extremely close timestamps')
    if max(abs(x-y) for x,y in zip(q[0],home_q))>0.015:
        raise RuntimeError('Adapted trajectory start does not match saved home')
    if max(abs(x-y) for x,y in zip(q[-1],grip_q))>0.20:
        raise RuntimeError('Adapted trajectory endpoint far from saved grip joints')
    if max(abs(b[i]-a[i]) for a,b in zip(q,q[1:]) for i in range(6))>=0.05:
        raise RuntimeError('Consecutive-joint discontinuity')
    for j,name in enumerate(JOINTS):
        v=[(b[j]-a[j])/interval for a,b,interval in zip(q,q[1:],dt)]
        acc=[(v[i+1]-v[i])/(0.5*(dt[i]+dt[i+1])) for i in range(len(v)-1)]
        if max(map(abs,v))>0.22 or max(map(abs,acc))>0.35:
            raise RuntimeError('Project review threshold exceeded for '+name)
    return t,q


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project',type=Path,
                    default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    ap.add_argument('--replay',type=Path,required=True,
                    help='CANDIDATE joint path from MoveIt plan and Phase 1 capture')
    ap.add_argument('--slowdown',type=float,default=2.0)
    ap.add_argument('--log',type=Path,required=True)
    a=ap.parse_args()
    if not 1<=a.slowdown<=5:
        ap.error('slowdown must be in [1,5]')
    root=a.project.expanduser().resolve()
    with (root/'config/saved_poses/v1_home_for_moveit.yaml').open() as f:
        home=yaml.safe_load(f)
    with (root/'config/saved_poses/v1_grip_pose.yaml').open() as f:
        grip=yaml.safe_load(f)
    if home['arm']['joint_names'] != JOINTS or grip['arm']['joint_names'] != JOINTS:
        raise RuntimeError('Saved pose joint name ordering mismatch')
    if grip.get('tool_frame')!='gripper_tcp' or grip.get('base_frame')!='base':
        raise RuntimeError('Saved grip frame mismatch')
    home_q=[float(x) for x in home['arm']['positions_rad']]
    grip_q=[float(x) for x in grip['arm']['positions_rad']]
    path=a.replay.expanduser().resolve()
    log=a.log.expanduser().resolve()
    if log.exists():
        raise RuntimeError('Refusing to overwrite an earlier simulated recording')
    if not path.is_file():
        raise RuntimeError('Missing planned joint trajectory: '+str(path))
    t,q=validate_path(path,home_q,grip_q,a.slowdown)
    print('Adapted V1 candidate:',len(t),'points, commanded duration',round(t[-1]*a.slowdown,2),'s')
    print('Target joints from v1_grip_pose.yaml:',[round(x,5) for x in grip_q])
    print('Actual MoveIt end joints:',[round(x,5) for x in q[-1]])
    print('No gripper-close action will be sent.')

    rclpy.init()
    node=Watch()
    accepted=None
    try:
        verify_fake_only(node)
        current=node.fresh()
        if max(abs(x-y) for x,y in zip(current,home_q))>0.015:
            raise RuntimeError('Fake robot is not at V1 home; use Phase1 FAKE HOME action')
        client=ActionClient(node,FollowJointTrajectory,
                            '/joint_trajectory_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=8):
            raise RuntimeError('Sim-only action server unavailable')
        print('\nThis action will move ONLY FAKE ROS hardware in isolated domain 73.')
        if input('Type CONFIRM_FAKE_ADAPTED_REACH: ').strip()!='CONFIRM_FAKE_ADAPTED_REACH':
            raise RuntimeError('Execution not authorized')
        verify_fake_only(node)
        current=node.fresh()
        if max(abs(x-y) for x,y in zip(current,home_q))>0.015:
            raise RuntimeError('Sim robot moved from saved home')
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=JOINTS
        for tt,pose in zip(t,q):
            ns=round(tt*a.slowdown*1e9)
            point=JointTrajectoryPoint()
            point.positions=pose
            point.time_from_start=Duration(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
            goal.trajectory.points.append(point)
        node.start=time.monotonic()
        node.log_on=True
        accepted=await_result(node,client.send_goal_async(goal),10)
        if not accepted.accepted:
            raise RuntimeError('Fake controller rejected goal')
        result=await_result(node,accepted.get_result_async(),max(50,3*t[-1]*a.slowdown+20))
        node.log_on=False
        if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
            raise RuntimeError('Fake trajectory failed: '+result.result.error_string)
        actual=node.fresh()
        max_err=max(abs(x-y) for x,y in zip(actual,q[-1]))
        if max_err>0.035:
            raise RuntimeError(f'Fake endpoint joint error too large: {max_err:.5f}')
        print('\nSIM ADAPTED V1 REACH SUCCESS; max final joint error:',round(max_err,7),'rad')
        print('This does NOT establish physical grip accuracy, collision safety or cross-variant generalisation.')
    except Exception as e:
        print('SIMULATION STOPPED:',e)
        if accepted is not None and accepted.accepted:
            try:
                await_result(node,accepted.cancel_goal_async(),4)
            except Exception:
                print('Cancellation unconfirmed; inspect fake controller')
        raise SystemExit(1)
    finally:
        node.log_on=False
        if node.logged:
            log.parent.mkdir(parents=True,exist_ok=True)
            with log.open('x',newline='') as f:
                w=csv.writer(f)
                w.writerow(['elapsed_s']+JOINTS)
                w.writerows(node.logged)
            print('Saved mock joint states:',log)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
