#!/usr/bin/env python3
"""Standalone open/close simulated RG2 finger_width via FAKE ROS hardware only.

Not a grasp/contact/force simulation and not an automated pick sequence.
"""
import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'phase1_reach'))
from sim_fake_guard import verify_fake_only, await_result


class Status(Node):
    def __init__(self):
        super().__init__('phase3_fake_gripper_only')
        self.width=None
        self.create_subscription(JointState,'/joint_states',self.state,qos_profile_sensor_data)
    def state(self,msg):
        if 'finger_width' in msg.name:
            self.width=msg.position[msg.name.index('finger_width')]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=('open','close_v1'),required=True)
    a=ap.parse_args()
    width={'open':0.050,'close_v1':0.009}[a.mode]
    rclpy.init(); node=Status(); handle=None
    try:
        verify_fake_only(node)
        client=node.create_client(ListControllers,'/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Controller manager unavailable')
        resp=await_result(node,client.call_async(ListControllers.Request()),5)
        status={c.name:c.state for c in resp.controller}
        if status.get('finger_width_trajectory_controller')!='active':
            raise RuntimeError('Mock finger_width_trajectory_controller not active: inspect launch')
        action=ActionClient(node,FollowJointTrajectory,
                            '/finger_width_trajectory_controller/follow_joint_trajectory')
        if not action.wait_for_server(timeout_sec=7):
            raise RuntimeError('Mock gripper action server unavailable')
        print('FAKE RG2',a.mode,'width',width*1000,'mm over 3 s')
        print('NOTE: No 16 N force control/contact simulation; position only.')
        phrase = 'OPEN' if a.mode == 'open' else 'GRIP'
        if input(f'Type {phrase} to proceed: ').strip() != phrase:
            raise RuntimeError('Not authorised')
        verify_fake_only(node)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=['finger_width']
        pt=JointTrajectoryPoint()
        pt.positions=[width]
        pt.time_from_start=Duration(sec=3,nanosec=0)
        goal.trajectory.points=[pt]
        handle=await_result(node,action.send_goal_async(goal),10)
        if not handle.accepted:raise RuntimeError('Mock controller rejected goal')
        res=await_result(node,handle.get_result_async(),25)
        if res.status!=GoalStatus.STATUS_SUCCEEDED or res.result.error_code!=0:
            raise RuntimeError('Mock gripper motion failed: '+res.result.error_string)
        # Fresh state is not guaranteed to arrive before result.
        deadline=__import__('time').monotonic()+2
        while __import__('time').monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        if node.width is None or not math.isfinite(node.width) or abs(node.width-width)>0.005:
            raise RuntimeError('Mock gripper joint did not reach expected width')
        print('SIM ONLY gripper position reached:',round(node.width*1000,3),'mm')
    except Exception as e:
        print('SIM GRIPPER TEST STOPPED:',e)
        raise SystemExit(1)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
