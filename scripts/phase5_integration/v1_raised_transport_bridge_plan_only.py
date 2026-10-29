#!/usr/bin/env python3
"""Physical UR3e V1: READ-ONLY planning of raised pickup-to-transport bridge.

Runs live ROS services, publishes a MoveIt preview and writes a diagnostic JSON.
NEVER sends arm or gripper goals. Does not clear E-stop or recover the arm.
Start only while stationary at learned TRANSPORT start, securely holding V1.
"""
import copy
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
    sha,transport=data()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState,DisplayTrajectory
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

    rclpy.init()
    node=Planner()
    try:
        print('V1 RAISED TRANSPORT BRIDGE — PLAN ONLY — NO ROBOT COMMANDS',flush=True)
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
                solutions.append((idx,br_ret,len(br),lowest,joint_gap,delta_z))
                break
            except (RuntimeError,ValueError) as exc:
                print(f'  candidate row {idx}: {exc}',flush=True)
        if not solutions:
            raise RuntimeError('No verifiable elevated Cartesian bridge into learned '
                               'transport. STOP — plan or demonstrate a new raised transport.')
        idx,bridge_ret,nbridge,lowest,joint_gap,delta_z=solutions[0]
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
            'status':'PLAN_ONLY_CANDIDATE_NOT_EXECUTED',
            'transport_sha256':sha,'lift_target_mm':30,
            'join_row':idx,'join_rise_mm':delta_z*1000,
            'bridge_min_height_above_pickup_mm':lowest*1000,
            'join_joint_error_rad':joint_gap,
            'warning':'Physical fixtures, pickup parts, carried component may be absent in MoveIt',
        },indent=2))
        print(f'\nPLAN-ONLY candidate complete. Selected TRANSPORT join row {idx}.')
        print('NO MOTION SENT. Review RViz AND the physical fixture/row before any execution.')
        print('Diagnostic:',folder/'plan_diagnostic.json',flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    try:main()
    except (RuntimeError,OSError,ValueError) as exc:
        sys.exit('PLAN-ONLY BLOCKED: '+str(exc))
