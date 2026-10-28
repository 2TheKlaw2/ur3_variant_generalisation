#!/usr/bin/env python3
"""Capture PLAN-ONLY MoveIt V1 transport joint path, on correct joint branch.

Subscribes to /display_planned_path; NEVER sends a controller action.
"""
import argparse
import csv
import math
import time
from pathlib import Path

JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint']
CSV_JOINTS=['shoulder_pan','shoulder_lift','elbow',
            'wrist_1','wrist_2','wrist_3']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    p.add_argument('--min-points',type=int,default=50)
    p.add_argument('--timeout',type=float,default=90)
    a=p.parse_args()
    root=a.project.expanduser().resolve()
    demo=root/'demonstrations/processed/v1_transport_001_motion.csv'
    target=root/'demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv'
    if target.exists(): raise SystemExit('Output already exists; refusing overwrite')
    with demo.open(newline='') as f:
        rec=csv.DictReader(f)
        if not rec.fieldnames or any(j not in rec.fieldnames for j in CSV_JOINTS):
            raise SystemExit('Missing demonstrated transport joint names')
        first=next(rec)
    start=[float(first[j]) for j in CSV_JOINTS]
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from moveit_msgs.msg import DisplayTrajectory

    class Collector(Node):
        def __init__(self):
            super().__init__('phase3_transport_plan_collector')
            self.finished=False
            self.create_subscription(
                DisplayTrajectory,'/display_planned_path',self.on_plan,
                QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
        def on_plan(self,msg):
            if not msg.trajectory: return
            traj=msg.trajectory[0].joint_trajectory
            if len(traj.points)<a.min_points:
                print('Ignoring short/unrelated trajectory:',len(traj.points),flush=True)
                return
            try:
                if not all(j in traj.joint_names for j in JOINTS):
                    raise ValueError('Missing arm joints in trajectory')
                idx=[traj.joint_names.index(j) for j in JOINTS]
                t=[p.time_from_start.sec + p.time_from_start.nanosec*1e-9 for p in traj.points]
                raw=[[float(p.positions[i]) for i in idx] for p in traj.points]
                if any(not math.isfinite(x) for x in t+[v for row in raw for v in row]):
                    raise ValueError('Nonfinite values')
                if t[0]<0 or any(b<=x for x,b in zip(t,t[1:])):
                    raise ValueError('Bad trajectory times')
                shifts=[2*math.pi*round((start[i]-raw[0][i])/(2*math.pi)) for i in range(6)]
                corrected=[[value+shifts[k] for k,value in enumerate(row)] for row in raw]
                err=max(abs(a-b) for a,b in zip(corrected[0],start))
                if err>0.02:
                    raise ValueError(f'Plan start differs from RECORDED transport start: {err:.5f} rad')
                max_step=max(abs(b[j]-a[j]) for a,b in zip(corrected,corrected[1:]) for j in range(6))
                if max_step>0.05: raise ValueError(f'Joint discontinuity {max_step:.5f} rad')
                target.parent.mkdir(parents=True,exist_ok=True)
                with target.open('x',newline='') as f:
                    w=csv.writer(f);w.writerow(['time']+JOINTS)
                    for ti,qi in zip(t,corrected):w.writerow([f'{ti:.9f}']+[f'{v:.12f}' for v in qi])
                print('\nPHASE 3 TRANSPORT PLAN CAPTURED (SIM CANDIDATE)')
                print('Points:',len(t),'Duration:',round(t[-1]-t[0],3),'s')
                print('Start error:',round(err,7),'rad, max joint step:',round(max_step,7),'rad')
                print('Continuous-joint branch shifts:',[round(x,5) for x in shifts])
                print('Saved:',target)
                print('NO ROBOT ACTION SENT. DO NOT EXECUTE WITHOUT FULL VALIDATION.')
            except Exception as e:
                print('Transport candidate capture rejected:',e)
            self.finished=True

    rclpy.init();node=Collector()
    try:
        print('Waiting for PLAN-ONLY V1 transport trajectory...',flush=True)
        end=time.monotonic()+a.timeout
        while rclpy.ok() and not node.finished and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.4)
        if not node.finished:raise SystemExit('No matching transport plan received')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
