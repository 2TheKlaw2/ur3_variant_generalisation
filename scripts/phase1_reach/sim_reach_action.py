#!/usr/bin/env python3
"""Execute V1 reach only on isolated ROS 2 fake hardware (ROS_DOMAIN_ID=73).

--home: place virtual UR3e at taught home; this is a mock-only transition.
--replay: replay already corrected and offline-reviewed full CDMP joint plan.
No real-hardware / scaled-controller action is used. No MoveIt preposition.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

from sim_fake_guard import await_result, verify_fake_only

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
HOME = [-0.675926510488, -1.34777768076, -0.527696251869,
        -2.8273965321, 1.55948805809, -3.87329799334]
# Rough expected full-reach endpoint based on the recorded human demo. This
# guards against accidentally loading a different skill's joint CSV.
REACH_END = [-0.604107205068, -1.94130577664, -1.33224058151,
             -1.43884961427, 1.51409721375, -3.70253354708]


class Monitor(Node):
    def __init__(self):
        super().__init__('phase1_fake_reach_only')
        self.joint_state = {}
        self.received = 0.0
        self.recording = False
        self.start_time = 0.0
        self.log = []
        self.create_subscription(JointState,'/joint_states',self.on_joint,qos_profile_sensor_data)

    def on_joint(self,msg):
        self.joint_state = dict(zip(msg.name, msg.position))
        self.received = time.monotonic()
        if self.recording and all(j in self.joint_state for j in JOINTS):
            self.log.append([self.received-self.start_time] +
                            [self.joint_state[j] for j in JOINTS])

    def get_fresh_positions(self):
        self.received = 0.0
        deadline = time.monotonic()+6
        while self.received==0 and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.1)
        if self.received==0 or time.monotonic()-self.received>0.5:
            raise RuntimeError('Live fake joint states missing or stale')
        if not all(j in self.joint_state for j in JOINTS):
            raise RuntimeError('Missing simulated arm joint(s)')
        return [self.joint_state[j] for j in JOINTS]


def read_full_plan(path):
    with path.open(newline='') as f:
        reader=csv.DictReader(f)
        if reader.fieldnames != ['time']+JOINTS:
            raise RuntimeError('Unexpected CSV header')
        data=list(reader)
    if len(data)<150:
        raise RuntimeError('Expected full V1 reach; too few waypoints')
    t=[float(row['time']) for row in data]
    q=[[float(row[j]) for j in JOINTS] for row in data]
    if any(not math.isfinite(x) for x in t+[x for row in q for x in row]):
        raise RuntimeError('Invalid numeric values')
    if t[0]<0 or any(b<=a for a,b in zip(t,t[1:])):
        raise RuntimeError('Invalid waypoint times')
    if max(abs(a-b) for a,b in zip(q[0],HOME))>=0.015:
        raise RuntimeError('Trajectory does not start at saved V1 home')
    if max(abs(b[j]-a[j]) for a,b in zip(q,q[1:]) for j in range(6))>=0.05:
        raise RuntimeError('Joint discontinuity')
    if not 1<t[-1]-t[0]<180:
        raise RuntimeError('Unexpected planned trajectory duration')
    if any(abs(x)>2*math.pi+0.1 for row in q for x in row):
        raise RuntimeError('Joint outside broad numerical review bounds')
    if max(abs(x-y) for x,y in zip(q[-1], REACH_END))>=0.20:
        raise RuntimeError('Full reach endpoint is not near the demonstrated endpoint')
    return t,q


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--home', action='store_true', help='Move fake robot to saved taught home')
    mode.add_argument('--replay', type=Path, help='Full V1 joint plan after offline checks')
    p.add_argument('--slowdown',type=float,default=2.0)
    p.add_argument('--log',type=Path,help='New CSV output for virtual robot joint states')
    a=p.parse_args()
    if not (1.0<=a.slowdown<=5.0):
        p.error('--slowdown must be between 1.0 and 5.0')
    if a.log and a.log.exists():
        p.error('Log exists; not overwriting')

    if a.home:
        # Virtual preposition from current mock arm configuration; not a
        # collision-checked plan and must NEVER be used for real hardware.
        times=[10.0]
        positions=[HOME]
        goal_config=HOME
    else:
        times,positions=read_full_plan(a.replay)
        dt=[(t_next-t_prev)*a.slowdown for t_prev,t_next in zip(times,times[1:])]
        for j,name in enumerate(JOINTS):
            v=[(b[j]-a[j])/d for a,b,d in zip(positions,positions[1:],dt)]
            acc=[(v[i+1]-v[i])/(0.5*(dt[i]+dt[i+1]))
                 for i in range(len(v)-1)]
            if max(map(abs,v))>=0.22 or max(map(abs,acc))>=0.35:
                raise RuntimeError('Velocity/acceleration review check failed: '+name)
        goal_config=positions[-1]
        duration=(times[-1]-times[0])*a.slowdown
        print('Fake replay:',len(times),'points over',round(duration,2),'s')

    rclpy.init()
    node=Monitor()
    accepted_handle=None
    try:
        verify_fake_only(node)
        actual=node.get_fresh_positions()
        if a.replay:
            error=max(abs(x-y) for x,y in zip(actual,HOME))
            if error>0.015:
                raise RuntimeError('Fake robot not at taught home (%.5f rad)'%error)

        client=ActionClient(node,FollowJointTrajectory,
                            '/joint_trajectory_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=7):
            raise RuntimeError('Mock trajectory action server not found')

        print('Mode:', 'FAKE HOME' if a.home else 'FAKE FULL V1 REACH')
        print('This program WILL move the virtual robot state, not physical hardware.')
        phrase='CONFIRM_FAKE_HOME' if a.home else 'CONFIRM_FAKE_V1_REACH'
        if input('Type '+phrase+' to simulate movement: ').strip()!=phrase:
            raise RuntimeError('Not confirmed')
        verify_fake_only(node)
        actual=node.get_fresh_positions()
        if a.replay and max(abs(x-y) for x,y in zip(actual,HOME))>0.015:
            raise RuntimeError('Fake robot moved away from home')
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=JOINTS
        for t,q in zip(times,positions):
            ns=round(t*a.slowdown*1e9)
            pt=JointTrajectoryPoint()
            pt.positions=q
            pt.time_from_start=Duration(sec=ns//1_000_000_000,
                                        nanosec=ns%1_000_000_000)
            goal.trajectory.points.append(pt)
        node.start_time=time.monotonic()
        node.recording=True
        accepted_handle=await_result(node,client.send_goal_async(goal))
        if not accepted_handle.accepted:
            raise RuntimeError('Mock action rejected trajectory')
        result=await_result(node,accepted_handle.get_result_async(),max(50,3*times[-1]*a.slowdown+20))
        node.recording=False
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError('Mock action failed: '+result.result.error_string)
        final=node.get_fresh_positions()
        e=max(abs(x-y) for x,y in zip(final,goal_config))
        print('SIM ACTION SUCCESS. Max endpoint joint error:',round(e,7),'rad')
        if e>0.035:
            raise RuntimeError('Mock endpoint error >0.035 rad')
    except Exception as exc:
        print('SIMULATOR ACTION STOPPED:', exc)
        if accepted_handle is not None and accepted_handle.accepted:
            try:
                await_result(node,accepted_handle.cancel_goal_async(),3)
            except Exception:
                print('Cancellation unconfirmed; stop fake controller if needed')
        raise SystemExit(1)
    finally:
        node.recording=False
        if a.log and node.log:
            a.log.parent.mkdir(parents=True,exist_ok=True)
            with a.log.open('x',newline='') as f:
                writer=csv.writer(f)
                writer.writerow(['elapsed_s']+JOINTS)
                writer.writerows(node.log)
            print('Saved simulated joint states:',a.log)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
