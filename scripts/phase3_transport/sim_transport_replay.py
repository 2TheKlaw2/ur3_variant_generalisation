#!/usr/bin/env python3
"""Guarded V1 learned transport replay on ROS 2 FAKE hardware ONLY.

Never positions the robot automatically; requires fake arm to be at the
recorded transport start and mock V1 gripper width to be closed.
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'phase1_reach'))
from sim_fake_guard import await_result, verify_fake_only

JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint']


class Monitor(Node):
    def __init__(self):
        super().__init__('phase3_fake_transport_only')
        self.state={};self.received=0.0;self.log=[];self.recording=False;self.t0=0
        self.create_subscription(JointState,'/joint_states',self.callback,qos_profile_sensor_data)
    def callback(self,msg):
        self.state=dict(zip(msg.name,msg.position))
        self.received=time.monotonic()
        if self.recording and all(j in self.state for j in JOINTS):
            self.log.append([self.received-self.t0]+[self.state[j] for j in JOINTS])
    def fresh(self):
        self.received=0
        deadline=time.monotonic()+6
        while self.received==0 and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.1)
        if self.received==0 or time.monotonic()-self.received>0.5:
            raise RuntimeError('No fresh fake joint states')
        if not all(j in self.state for j in JOINTS):
            raise RuntimeError('Fake arm joints missing')
        return [self.state[j] for j in JOINTS]


def load_plan(path,slowdown):
    if not path.is_file():raise ValueError('Missing candidate: '+str(path))
    with path.open(newline='') as f:
        r=csv.DictReader(f)
        if r.fieldnames!=['time']+JOINTS:raise ValueError('Invalid joint CSV header')
        rows=list(r)
    if len(rows)<50:raise ValueError('Too few transport plan points')
    times=[float(r['time']) for r in rows]
    qs=[[float(r[j]) for j in JOINTS] for r in rows]
    if any(not math.isfinite(x) for x in times+[u for row in qs for u in row]):
        raise ValueError('Nonfinite joint trajectory')
    if times[0]<0 or not 1<times[-1]-times[0]<180:
        raise ValueError('Invalid duration')
    dt=[(b-a)*slowdown for a,b in zip(times,times[1:])]
    if any(d<=0.0001 for d in dt):raise ValueError('Invalid timestep')
    if any(abs(v)>2*math.pi+0.1 for row in qs for v in row):
        raise ValueError('Numerical joint angle outside review range')
    if max(abs(b[i]-a[i]) for a,b in zip(qs,qs[1:]) for i in range(6))>0.05:
        raise ValueError('Trajectory joint discontinuity')
    for i,name in enumerate(JOINTS):
        v=[(b[i]-a[i])/d for a,b,d in zip(qs,qs[1:],dt)]
        acc=[(v[k+1]-v[k])/(0.5*(dt[k]+dt[k+1])) for k in range(len(v)-1)]
        if max(map(abs,v))>0.22 or max(map(abs,acc))>0.35:
            raise ValueError('Review velocity/acceleration threshold exceeded: '+name)
    return times,qs


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    p.add_argument('--slowdown',type=float,default=2.0)
    args=p.parse_args()
    if not 1<=args.slowdown<=5:p.error('Slowdown must be 1–5')
    root=args.project.expanduser().resolve()
    source=root/'demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv'
    out=root/'results/simulation/v1_transport_sim_actual_001.csv'
    if out.exists():raise SystemExit('Output already exists; refusing overwrite')
    times,qs=load_plan(source,args.slowdown)
    rclpy.init();node=Monitor(); handle=None
    try:
        verify_fake_only(node)
        real=node.fresh()
        diff=max(abs(x-y) for x,y in zip(real,qs[0]))
        if diff>0.015:
            raise RuntimeError(f'Fake arm is NOT at TRANSPORT start (max {diff:.5f}rad). '
                               'Do not teleport/skip reach-to-transport handoff.')
        if 'finger_width' not in node.state or not 0.007 <= node.state['finger_width'] <= 0.012:
            raise RuntimeError('Fake V1 gripper is not near its tested 9 mm grip width')
        action=ActionClient(node,FollowJointTrajectory,
                            '/joint_trajectory_controller/follow_joint_trajectory')
        if not action.wait_for_server(timeout_sec=7):
            raise RuntimeError('Mock joint trajectory action unavailable')
        print('\nFAKE V1 LEARNED TRANSPORT REPLAY')
        print('Joint points:',len(times),'duration:',round(times[-1]*args.slowdown,2),'s')
        print('Fake arm start error:',round(diff,7),'rad')
        print('Fake gripper width:',round(node.state['finger_width']*1000,2),'mm')
        print('No automatic home or pre-positioning; current live start is used.')
        print('No physical hardware, no object contact/attachment simulation.')
        if input('Type TRANSPORT to simulate: ').strip()!='TRANSPORT':
            raise RuntimeError('Simulation not authorised')
        verify_fake_only(node)
        now=node.fresh()
        if max(abs(x-y) for x,y in zip(now,qs[0]))>0.015:
            raise RuntimeError('Virtual robot moved away from expected transport start')
        if 'finger_width' not in node.state or not 0.007 <= node.state['finger_width'] <= 0.012:
            raise RuntimeError('Fake V1 gripper changed since confirmation')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=JOINTS
        for t,q in zip(times,qs):
            ns=round(t*args.slowdown*1e9)
            point=JointTrajectoryPoint();point.positions=q
            point.time_from_start=Duration(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
            goal.trajectory.points.append(point)
        node.t0=time.monotonic();node.recording=True
        handle=await_result(node,action.send_goal_async(goal),10)
        if not handle.accepted:raise RuntimeError('Mock trajectory rejected')
        result=await_result(node,handle.get_result_async(),max(50,3*times[-1]*args.slowdown+20))
        node.recording=False
        if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
            raise RuntimeError('Fake transport action failed: '+result.result.error_string)
        end=node.fresh()
        err=max(abs(x-y) for x,y in zip(end,qs[-1]))
        if err>0.035:raise RuntimeError(f'Fake transport endpoint error {err:.6f} rad')
        print('FAKE TRANSPORT REPLAY SUCCESS. Final joint error:',round(err,7),'rad')
        print('NOT a physical-grip, contact or insertion validation.')
    except Exception as e:
        print('SIM TRANSPORT STOPPED:',e)
        if handle is not None and handle.accepted:
            try:await_result(node,handle.cancel_goal_async(),4)
            except Exception:print('Cancellation unconfirmed — inspect fake controller')
        raise SystemExit(1)
    finally:
        node.recording=False
        if node.log:
            out.parent.mkdir(parents=True,exist_ok=True)
            with out.open('x',newline='') as f:
                w=csv.writer(f);w.writerow(['elapsed_s']+JOINTS);w.writerows(node.log)
            print('Mock arm state log:',out)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
