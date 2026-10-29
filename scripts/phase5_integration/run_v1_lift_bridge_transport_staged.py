#!/usr/bin/env python3
"""Physical UR3e V1: guarded lifted-pickup and faster transport experiment.

Default is planning-only and NEVER moves the robot. With --execute: plan,
review, then separately approve LIFT, BRIDGE, TRANSPORT_TAIL, then STOP.
This program does not reset E-stop, open gripper, insert, or go home.
Only run from live learned TRANSPORT start while securely holding V1.
Physical fixtures/workpiece might be absent from MoveIt. Observe and use pendant STOP.
"""
import copy
import argparse
import signal
import csv
import hashlib
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
PATH = ROOT / 'demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv'
EXPECTED_SHA = 'f4335b3ef675'
JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
LIFT_M = .030
JOIN_TOL = .0055


def maxdiff(a,b): return max(abs(x-y) for x,y in zip(a,b))


def data():
    sha=hashlib.sha256(PATH.read_bytes()).hexdigest()
    if not sha.startswith(EXPECTED_SHA):
        raise RuntimeError('TRANSPORT file hash changed; refusing to plan from unknown path')
    with PATH.open(newline='') as f:
        r=csv.DictReader(f)
        if r.fieldnames != ['time']+JOINTS:
            raise RuntimeError('Transport CSV header changed')
        q=[[float(x[j]) for j in JOINTS] for x in r]
    if len(q)!=246:raise RuntimeError('Transport CSV sample count changed')
    return sha,q


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='0':
        raise RuntimeError('Requires ROS_DOMAIN_ID=0')
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',
                    help='Send guarded real arm goals after three separate approvals')
    args=ap.parse_args()
    sha,transport=data()
    if args.execute and not sys.stdin.isatty():
        raise RuntimeError('Interactive terminal required for physical execution')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.signals import SignalHandlerOptions
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from action_msgs.msg import GoalStatus
    from moveit_msgs.srv import GetStateValidity
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState,DisplayTrajectory,RobotTrajectory
    from moveit_msgs.srv import GetPositionFK,GetCartesianPath
    from tf2_ros import Buffer,TransformListener

    class Planner(Node):
        def __init__(self):
            super().__init__('v1_raised_bridge_PLAN_ONLY')
            self.joint=None;self.joint_msg=None;self.joint_at=0
            self.width=None;self.width_at=0
            self.create_subscription(JointState,'/joint_states',self.cb,qos_profile_sensor_data)
            self.hw=self.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components')
            self.ctl=self.create_client(ListControllers,'/controller_manager/list_controllers')
            self.fk=self.create_client(GetPositionFK,'/compute_fk')
            self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path')
            self.arm=ActionClient(self,FollowJointTrajectory,
              '/scaled_joint_trajectory_controller/follow_joint_trajectory')
            self.valid=self.create_client(GetStateValidity,'/check_state_validity')
            self.pending_goal=None
            self.active_handle=None
            self.result_future=None
            self.samples=[]
            self.recording=False
            self.stage=''
            self.log_start=time.monotonic()
            self.tfbuf=Buffer()
            self.tf=TransformListener(self.tfbuf,self)
            self.preview=self.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        def cb(self,msg):
            if len(msg.name)!=len(msg.position):return
            now=time.monotonic()
            if 'finger_width' in msg.name:
                self.width=float(msg.position[msg.name.index('finger_width')])
                self.width_at=now
            if all(j in msg.name for j in JOINTS):
                q=[float(msg.position[msg.name.index(j)]) for j in JOINTS]
                if not all(math.isfinite(v) for v in q):return
                self.joint=q;self.joint_msg=msg;self.joint_at=now
                if self.recording:
                    self.samples.append([round(now-self.log_start,6),self.stage]+q+
                      [self.width*1000 if self.width is not None else ''])
        def wait(self,fn,seconds,label):
            stop=time.monotonic()+seconds
            while time.monotonic()<stop:
                rclpy.spin_once(self,timeout_sec=.1)
                if fn():return
            raise RuntimeError('Timeout '+label)
        def rpc(self,client,req,label,timeout=15):
            if not client.wait_for_service(timeout_sec=6):
                raise RuntimeError('Missing '+label+' service')
            fut=client.call_async(req)
            self.wait(lambda:fut.done(),timeout,label)
            if fut.result() is None:raise RuntimeError(label+' failed')
            return fut.result()
        def fresh(self):
            t=time.monotonic()
            self.wait(lambda:self.joint_at>t and self.width_at>t,5,'joint states and RG2')
            return self.joint[:]
        def guard(self):
            hs=self.rpc(self.hw,ListHardwareComponents.Request(),'real hardware').component
            actual={(h.name,h.class_type) for h in hs}
            if ('ur3e','ur_robot_driver/URPositionHardwareInterface') not in actual:
                raise RuntimeError('Not verified real UR3e hardware')
            if ('OnRobotGripper','onrobot_driver/RGHardwareInterface') not in actual:
                raise RuntimeError('Not verified physical OnRobot RG2')
            cs=self.rpc(self.ctl,ListControllers.Request(),'controllers').controller
            states={c.name:c.state.lower() for c in cs}
            if states.get('scaled_joint_trajectory_controller')!='active':
                raise RuntimeError('Scaled arm controller is inactive')
            if states.get('joint_trajectory_controller')=='active':
                raise RuntimeError('Another arm controller active')
            if states.get('finger_width_trajectory_controller')!='active':
                raise RuntimeError('RG2 width controller inactive')
            if not self.arm.wait_for_server(timeout_sec=6):
                raise RuntimeError('Physical arm action unavailable')
        def fkpose(self,q):
            req=GetPositionFK.Request();req.header.frame_id='base'
            req.fk_link_names=['gripper_tcp']
            req.robot_state=RobotState()
            req.robot_state.joint_state.name=JOINTS
            req.robot_state.joint_state.position=list(q)
            ret=self.rpc(self.fk,req,'MoveIt FK',timeout=6)
            if ret.error_code.val!=1 or len(ret.pose_stamped)!=1:
                raise RuntimeError('MoveIt FK failed')
            return ret.pose_stamped[0].pose
        def cartesian(self,from_full_state,poses,label):
            req=GetCartesianPath.Request()
            req.header.frame_id='base';req.group_name='ur_onrobot_manipulator'
            req.link_name='gripper_tcp';req.start_state=RobotState()
            req.start_state.joint_state=from_full_state
            req.waypoints=poses;req.max_step=.002
            req.jump_threshold=0.0;req.avoid_collisions=True
            req.max_velocity_scaling_factor=.03
            req.max_acceleration_scaling_factor=.03
            ret=self.rpc(self.cart,req,label,timeout=22)
            jt=ret.solution.joint_trajectory
            if ret.error_code.val!=1 or ret.fraction<.999 or len(jt.points)<3:
                raise RuntimeError(f'{label} incomplete: fraction={ret.fraction:.4f}, '
                                   f'error={ret.error_code.val}')
            if set(jt.joint_names)!=set(JOINTS):
                raise RuntimeError('Unrecognised MoveIt joint names')
            ix=[jt.joint_names.index(j) for j in JOINTS]
            return ret,[[float(p.positions[i]) for i in ix] for p in jt.points]
        def correct(self,raw,reference,label):
            offsets=[]
            for i,j in enumerate(JOINTS):
                n=round((raw[0][i]-reference[i])/(2*math.pi))
                if abs(raw[0][i]-n*2*math.pi-reference[i])>.012:
                    raise RuntimeError(label+': IK start mismatch')
                if n!=0 and j!='wrist_3_joint':
                    raise RuntimeError(label+': 2pi branch for unexpected joint '+j)
                offsets.append(n)
            corrected=[[v-n*2*math.pi for v,n in zip(row,offsets)] for row in raw]
            if any(maxdiff(a,b)>.05 for a,b in zip(corrected,corrected[1:])):
                raise RuntimeError(label+': excessively large joint step')
            return corrected,offsets


        def guard_gripped(self, expected=None, tol=.0055):
            self.guard()
            live=self.fresh()
            if expected is not None and maxdiff(live,expected)>tol:
                raise RuntimeError(f'Live joint start error {maxdiff(live,expected):.5f}rad; '
                                   'cannot connect planned stages')
            if not .007<=self.width<=.0145:
                raise RuntimeError(f'RG2 width outside observed V1 grip range: '
                                   f'{self.width*1000:.2f}mm')
            return live

        def verify_path(self, name, q, t, speed_limit=.14):
            if len(q)!=len(t) or len(q)<3 or abs(t[0])>1e-6:
                raise RuntimeError(name+': invalid trajectory size/timing')
            if any(not math.isfinite(v) for row in q for v in row):
                raise RuntimeError(name+': nonfinite joint values')
            if any(b<=a for a,b in zip(t,t[1:])):
                raise RuntimeError(name+': nonmonotonic time')
            jumps=[maxdiff(a,b) for a,b in zip(q,q[1:])]
            rates=[v/(tb-ta) for v,ta,tb in zip(jumps,t,t[1:])]
            if max(jumps)>.045 or max(rates)>speed_limit:
                raise RuntimeError(f'{name}: joint step/speed guard failed; '
                                   f'max step={max(jumps):.4f}rad; '
                                   f'max speed={max(rates):.4f}rad/s')
            print(f'{name}: {len(q)} points, {t[-1]:.2f}s, '
                  f'peak step {max(jumps):.4f}rad, peak joint speed '
                  f'{max(rates):.4f}rad/s',flush=True)

        def times(self,ret,corrected,label,minimum_seconds):
            pts=ret.solution.joint_trajectory.points
            secs=[x.time_from_start.sec+x.time_from_start.nanosec*1e-9
                  for x in pts]
            if len(secs)!=len(corrected) or any(b<=a for a,b in zip(secs,secs[1:])):
                raise RuntimeError(label+': MoveIt trajectory times invalid')
            secs=[x-secs[0] for x in secs]
            if secs[-1]<.001:
                raise RuntimeError(label+': duration too short')
            peak=max(maxdiff(a,b)/(tb-ta) for a,b,ta,tb in
                     zip(corrected,corrected[1:],secs,secs[1:]))
            factor=max(1,minimum_seconds/secs[-1],peak/.12)
            return [x*factor for x in secs]

        def validity(self,name,q):
            # Sample trajectory states; not a replacement for real fixture clearance.
            selected=sorted(set(round(i*(len(q)-1)/min(60,len(q)-1))
                                for i in range(min(60,len(q)-1)+1)))
            for i in selected:
                req=GetStateValidity.Request()
                req.group_name='ur_onrobot_manipulator'
                req.robot_state=RobotState()
                req.robot_state.joint_state=copy.deepcopy(self.joint_msg)
                for j,v in zip(JOINTS,q[i]):
                    req.robot_state.joint_state.position[
                        req.robot_state.joint_state.name.index(j)]=v
                response=self.rpc(self.valid,req,f'{name} validity sample {i}',timeout=5)
                if not response.valid:
                    raise RuntimeError(f'{name}: MoveIt collision/state invalid at row {i}')
            print(f'{name}: {len(selected)} MoveIt states valid '
                  '(unmodelled geometry NOT checked)',flush=True)

        def cancel(self):
            handle=self.active_handle
            if handle is None and self.pending_goal is not None:
                until=time.monotonic()+4
                while time.monotonic()<until and not self.pending_goal.done():
                    rclpy.spin_once(self,timeout_sec=.1)
                if self.pending_goal.done():
                    handle=self.pending_goal.result()
            if handle is None or not handle.accepted:
                print('No accepted goal to cancel. Verify pendant state.',flush=True)
                return
            if self.result_future and self.result_future.done():
                return
            print('REQUESTING ARM CONTROLLER CANCELLATION; not E-stop.',flush=True)
            try:
                cf=handle.cancel_goal_async()
                until=time.monotonic()+6
                while time.monotonic()<until and not cf.done():
                    rclpy.spin_once(self,timeout_sec=.1)
                if not cf.done() or cf.result() is None or not cf.result().goals_canceling:
                    print('CANCEL UNCONFIRMED — use pendant STOP if moving.',flush=True)
                    return
                rf=self.result_future or handle.get_result_async()
                until=time.monotonic()+8
                while time.monotonic()<until and not rf.done():
                    rclpy.spin_once(self,timeout_sec=.1)
                if not rf.done() or rf.result() is None or \
                   rf.result().status!=GoalStatus.STATUS_CANCELED:
                    print('Cancel accepted, terminal state not verified; use pendant.',flush=True)
                else:
                    print('Controller cancellation confirmed; NOT an E-stop.',flush=True)
            except Exception as exc:
                print('Cancel problem:',exc,'— use pendant STOP.',flush=True)

        def execute_segment(self,name,q,t,expected):
            self.verify_path(name,q,t)
            live=self.guard_gripped(expected)
            self.recording=True
            self.stage=name
            traj=JointTrajectory()
            traj.joint_names=list(JOINTS)
            # Replace only first sample with FRESH joints (do not warp the path).
            points=[live]+[list(row) for row in q[1:]]
            if maxdiff(points[0],points[1])>.045:
                raise RuntimeError(name+': live-to-next-point jump too large')
            self.verify_path(name,points,t)
            for secs,positions in zip(t,points):
                pt=JointTrajectoryPoint()
                pt.positions=positions
                pt.time_from_start=Duration(seconds=float(secs)).to_msg()
                traj.points.append(pt)
            goal=FollowJointTrajectory.Goal()
            goal.trajectory=traj
            goal.goal_time_tolerance=Duration(seconds=4.0).to_msg()
            self.pending_goal=self.arm.send_goal_async(goal)
            self.wait(lambda:self.pending_goal.done(),7,name+' goal response')
            self.active_handle=self.pending_goal.result()
            if self.active_handle is None or not self.active_handle.accepted:
                raise RuntimeError(name+': controller rejected arm goal')
            self.result_future=self.active_handle.get_result_async()
            until=time.monotonic()+t[-1]+18
            print(f'{name}: PHYSICAL MOTION SENT. Monitor with pendant.',flush=True)
            while not self.result_future.done():
                if time.monotonic()>until:
                    raise RuntimeError(name+': controller execution timed out')
                rclpy.spin_once(self,timeout_sec=.12)
                if time.monotonic()-self.joint_at>1.5:
                    raise RuntimeError(name+': live /joint_states lost')
                if self.width_at<=0 or time.monotonic()-self.width_at>2.0 or \
                   not .007<=self.width<=.0145:
                    raise RuntimeError(name+': RG2 width feedback lost or grasp changed')
            result=self.result_future.result()
            if result is None or result.status!=GoalStatus.STATUS_SUCCEEDED or \
               result.result.error_code!=FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(name+': controller did not succeed')
            self.active_handle=None
            self.pending_goal=None
            self.result_future=None
            current=self.guard_gripped(q[-1])
            print(f'{name}: PASS; final joint err={maxdiff(current,q[-1]):.6f}rad; '
                  f'RG2={self.width*1000:.2f}mm',flush=True)
            return current

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Planner()
    original_sigint=signal.getsignal(signal.SIGINT)
    def sigint(_sig,_frame):
        raise KeyboardInterrupt('Ctrl+C requested')
    signal.signal(signal.SIGINT,sigint)
    try:
        print('V1 RAISED TRANSPORT BRIDGE — '+('GUARDED EXECUTE' if args.execute else 'PLAN ONLY — NO MOTION'),flush=True)
        node.guard()
        live=node.fresh()
        start_err=maxdiff(live,transport[0])
        width_m=node.width
        print('Current start error (rad):',round(start_err,6))
        print('Measured RG2 width (mm):',round(width_m*1000,2))
        if start_err>.006:
            raise RuntimeError('Arm is not at original TRANSPORT start; no plan produced')
        if not .007<=width_m<=.0145:
            raise RuntimeError('RG2 does not appear to hold V1')
        node.wait(lambda:node.tfbuf.can_transform('base','gripper_tcp',Time()),6,'base gripper TF')
        tf=node.tfbuf.lookup_transform('base','gripper_tcp',Time()).transform
        origin=(tf.translation.x,tf.translation.y,tf.translation.z)
        rot=(tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w)
        fk0=node.fkpose(live)
        if math.dist(origin,(fk0.position.x,fk0.position.y,fk0.position.z))>.005:
            raise RuntimeError('Measured TF disagrees with MoveIt FK >5mm')
        poses=[]
        for k in range(1,16):
            p=Pose()
            p.position.x,p.position.y=origin[:2]
            p.position.z=origin[2]+LIFT_M*k/15
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=rot
            poses.append(p)
        lift_ret,lift_raw=node.cartesian(node.joint_msg,poses,'30mm vertical lift')
        lift,lift_offsets=node.correct(lift_raw,live,'LIFT')
        if maxdiff(lift[-1],live)>.35:
            raise RuntimeError('LIFT exceeds conservative joint movement guard')
        lift_pose=node.fkpose(lift[-1])
        measured_z=lift_pose.position.z-origin[2]
        measured_xy=math.hypot(lift_pose.position.x-origin[0],lift_pose.position.y-origin[1])
        if abs(measured_z-LIFT_M)>.002 or measured_xy>.002:
            raise RuntimeError('Planned lift TCP movement not vertical')
        print(f'LIFT planned: {len(lift)} pts; +Z={measured_z*1000:.2f}mm, '
              f'XY drift={measured_xy*1000:.2f}mm, 2pi turns={lift_offsets}',flush=True)

        print('Searching original learned TRANSPORT for later raised join...',flush=True)
        join_candidates=[]
        high=-1.0
        for i in range(8,len(transport),2):
            p=node.fkpose(transport[i])
            dz=p.position.z-origin[2]
            high=max(high,dz)
            if dz >= LIFT_M+.001:
                join_candidates.append((i,p,dz))
        if not join_candidates:
            raise RuntimeError('No later learned transport point is >=31mm above pickup; '
                               f'max learned rise={high*1000:.1f}mm. '
                               'Need to record/replan a new transport. NO MOTION.')
        print(f'Found {len(join_candidates)} original trajectory join candidates '
              f'(max rise {high*1000:.1f}mm). Testing collision-aware Cartesian bridge.',flush=True)
        solutions=[]
        # Plan from planned LIFT final robot state (not the current physical joint state).
        virt=copy.deepcopy(node.joint_msg)
        for j,value in zip(JOINTS,lift[-1]):
            virt.position[virt.name.index(j)]=value
        for idx,p,delta_z in join_candidates[:25]:
            # First cross the pickup row at >=30mm raised, then meet the original
            # raised transport waypoint WITHOUT returning to original pickup height.
            horizontal=Pose()
            horizontal.position.x=p.position.x
            horizontal.position.y=p.position.y
            horizontal.position.z=origin[2]+LIFT_M
            horizontal.orientation.x,horizontal.orientation.y,\
              horizontal.orientation.z,horizontal.orientation.w=rot
            try:
                br_ret,raw=node.cartesian(virt,[horizontal,p],f'bridge to row {idx}')
                br,off=node.correct(raw,lift[-1],f'BRIDGE row {idx}')
                joint_gap=maxdiff(br[-1],transport[idx])
                if joint_gap>JOIN_TOL:
                    print(f'  candidate row {idx}: joint handoff {joint_gap:.4f}rad (>limit)')
                    continue
                # Recheck FK samples along bridge to prevent an unexpected dip.
                samples=sorted(set(round(k*(len(br)-1)/20) for k in range(21)))
                lowest=min(node.fkpose(br[k]).position.z-origin[2] for k in samples)
                if lowest < LIFT_M-.002:
                    print(f'  candidate row {idx}: path dips to {lowest*1000:.1f}mm')
                    continue
                if maxdiff(lift[-1],br[-1])>.9:
                    print(f'  candidate row {idx}: very long joint-space bridge')
                    continue
                print(f'  VALID CANDIDATE: row={idx}; original rise={delta_z*1000:.1f}mm; '
                      f'bridge={len(br)} pts; lowest Z={lowest*1000:.1f}mm; '
                      f'joint join err={joint_gap:.5f}rad; offsets={off}',flush=True)
                solutions.append((idx,br_ret,br,len(br),lowest,joint_gap,delta_z))
                break
            except (RuntimeError,ValueError) as exc:
                print(f'  candidate row {idx}: {exc}',flush=True)
        if not solutions:
            raise RuntimeError('No verifiable elevated Cartesian bridge into learned '
                               'transport. STOP — plan or demonstrate a new raised transport.')
        idx,bridge_ret,bridge,nbridge,lowest,joint_gap,delta_z=solutions[0]
        d=DisplayTrajectory()
        d.model_id='ur_onrobot'
        d.trajectory_start=lift_ret.start_state
        d.trajectory.extend([lift_ret.solution,bridge_ret.solution])
        node.preview.publish(d)
        # Spin briefly to increase the likelihood RViz receives the preview.
        for _ in range(8):rclpy.spin_once(node,timeout_sec=.1)
        folder=ROOT/'results/physical/v1_lift_bridge_plan_only'/datetime.now().strftime('%Y%m%d_%H%M%S')
        folder.mkdir(parents=True,exist_ok=False)
        (folder/'plan_diagnostic.json').write_text(json.dumps({
            'status':'PLANNED_NOT_YET_EXECUTED',
            'transport_sha256':sha,'lift_target_mm':30,
            'join_row':idx,'join_rise_mm':delta_z*1000,
            'bridge_min_height_above_pickup_mm':lowest*1000,
            'join_joint_error_rad':joint_gap,
            'warning':'Physical fixtures, pickup parts, carried component may be absent in MoveIt',
        },indent=2))
        print(f'\nPLANNED candidate complete. Selected TRANSPORT join row {idx}.')
        print('NO MOTION YET. Review RViz AND the physical fixture/row before any execution.')
        print('Diagnostic:',folder/'plan_diagnostic.json',flush=True)
        # Revalidation BEFORE ANY ACTION GOAL. Candidate must have the same
        # characteristics as the previously reviewed successful plan.
        if idx!=24:
            raise RuntimeError(f'Unexpected join row {idx} (expected 24). NO MOTION')
        if maxdiff(lift[-1],bridge[0])>.0055:
            raise RuntimeError('Lift->bridge joint discontinuity')
        if maxdiff(bridge[-1],transport[idx])>.0055:
            raise RuntimeError('Bridge->learned transport joint discontinuity')
        # Verify actual planned lift and bridge are still the same geometry.
        for name,path in [('LIFT',lift),('BRIDGE',bridge)]:
            samples=sorted(set(round(k*(len(path)-1)/30) for k in range(31)))
            low=min(node.fkpose(path[k]).position.z-origin[2] for k in samples)
            if name=='BRIDGE' and low < .028:
                raise RuntimeError(f'Bridge dips too low: {low*1000:.2f}mm')
        # Build time-parameterised actions conservatively.
        lift_t=node.times(lift_ret,lift,'LIFT',5.0)
        bridge_t=node.times(bridge_ret,bridge,'BRIDGE',6.0)
        # Original CSV timestamps are 0.1s apart; x1.5 from row 24 onwards.
        with PATH.open(newline='') as f:
            rows=list(csv.DictReader(f))
        raw_t=[float(row['time']) for row in rows]
        tail=transport[idx:]
        tail_t=[(x-raw_t[idx])*1.5 for x in raw_t[idx:]]
        node.verify_path('LIFT',lift,lift_t)
        node.verify_path('BRIDGE',bridge,bridge_t)
        node.verify_path('TRANSPORT_TAIL',tail,tail_t)
        for name,path in [('LIFT',lift),('BRIDGE',bridge),
                          ('TRANSPORT_TAIL',tail)]:
            node.validity(name,path)
        if maxdiff(node.guard_gripped(transport[0]),lift[0])>.0055:
            raise RuntimeError('Live start differs from planned LIFT')
        # Updated RViz preview shows the EXACT corrected joint trajectories
        # intended for execution, including the learned transport remainder.
        preview=DisplayTrajectory()
        preview.model_id='ur_onrobot'
        preview.trajectory_start=RobotState()
        preview.trajectory_start.joint_state=copy.deepcopy(node.joint_msg)
        for path,t in ((lift,lift_t),(bridge,bridge_t),(tail,tail_t)):
            rt=RobotTrajectory()
            rt.joint_trajectory.joint_names=list(JOINTS)
            for joints,sec in zip(path,t):
                pt=JointTrajectoryPoint()
                pt.positions=list(joints)
                pt.time_from_start=Duration(seconds=float(sec)).to_msg()
                rt.joint_trajectory.points.append(pt)
            preview.trajectory.append(rt)
        node.preview.publish(preview)
        for _ in range(12):
            rclpy.spin_once(node,timeout_sec=.1)
        print('RViz preview: LIFT + BRIDGE + TRANSPORT_TAIL (full path).',flush=True)
        print('Note: RViz cannot display collisions with unmodelled physical parts.',flush=True)
        if not args.execute:
            print('\nFULL NO-MOTION PREFLIGHT PASS. NO ARM OR GRIPPER GOALS SENT.')
            print('Inspect the whole RViz preview and real fixture before executing.')
            return
        print('\nREADY FOR THREE SEPARATELY CONFIRMED PHYSICAL MOVEMENTS.')
        print('Do not start without checking the COMPLETE RViz path AND actual')
        print('five pickup parts, attached component, trolley and destination.')
        print('MoveIt validity cannot protect against unmodelled objects.')
        stage_plans=[
            ('LIFT',lift,lift_t,transport[0],
             '30 mm upward lift with held V1, no side contact at pickup'),
            ('BRIDGE',bridge,bridge_t,lift[-1],
             'raised bridge across pickup area to transport row 24'),
            ('TRANSPORT_TAIL',tail,tail_t,bridge[-1],
             'faster x1.5 learned transport remainder to placement area; STOP')]
        output={'trial_type':'PILOT_STAGE_BY_STAGE_RAISED_TRANSPORT',
                'status':'IN_PROGRESS','join_row':idx,
                'transport_sha256':sha,'stages':[],
                'note':'controller and measured joint PASS != physical assembly PASS'}
        for name,path,times,expected,description in stage_plans:
            print('\nSTAGE:',name,'|',description)
            print('Inspect REAL held piece and all fixtures. Pendant stop accessible.')
            if input('Type RUN_'+name+' after clearance review: ').strip()!='RUN_'+name:
                raise RuntimeError(name+': operator cancelled (no next motion)')
            node.guard_gripped(expected)
            start=time.monotonic()
            node.execute_segment(name,path,times,expected)
            output['stages'].append({'name':name,'status':'CONTROLLER_PASS',
                                     'elapsed_s':round(time.monotonic()-start,3)})
            (folder/'status.json').write_text(json.dumps(output,indent=2))
        output['status']='MOTION_PASS_PHYSICAL_ALIGNMENT_UNVERIFIED'
        (folder/'status.json').write_text(json.dumps(output,indent=2))
        print('\nTRANSPORT FINISHED. STOPPED. V1 STILL GRIPPED.')
        print('Inspect fixture alignment. No insert/release/return HOME commanded.')
        print('Logs:',folder,flush=True)

    except BaseException as exc:
        print('STOPPED / BLOCKED:',str(exc),flush=True)
        node.cancel()
        raise
    finally:
        node.recording=False
        try:
            if 'folder' in locals() and folder.is_dir() and node.samples:
                with (folder/'physical_joint_states.csv').open('w',newline='') as f:
                    w=csv.writer(f)
                    w.writerow(['elapsed_s','stage']+JOINTS+['gripper_width_mm'])
                    w.writerows(node.samples)
                if args.execute and ('output' not in locals() or
                                      output.get('status')=='IN_PROGRESS'):
                    stopped={'trial_type':'PILOT_STAGE_BY_STAGE_RAISED_TRANSPORT',
                             'status':'INTERRUPTED_OR_BLOCKED',
                             'completed_stages':output.get('stages',[]) if 'output' in locals() else [],
                             'warning':'Inspect live robot state before commanding motion'}
                    (folder/'status.json').write_text(json.dumps(stopped,indent=2))
        except Exception as log_error:
            print('Log-write error:',log_error,flush=True)
        signal.signal(signal.SIGINT,original_sigint)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,RuntimeError,OSError,ValueError) as exc:
        sys.exit('V1 BRIDGE STOPPED/BLOCKED: '+str(exc))
