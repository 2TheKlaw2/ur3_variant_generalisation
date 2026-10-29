#!/usr/bin/env python3
"""Plan-only V1 post-release UR3e retreat. NEVER commands arm movement.

Requires a live MoveIt /compute_cartesian_path service and /joint_states,
TF base -> gripper_tcp. Verify collision geometry/clearance physically.
"""

import csv
import math
import os
import sys
import time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '0':
    raise SystemExit('BLOCKED: ROS_DOMAIN_ID must be 0 for real-robot observation.')

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer, TransformListener

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
ROTATE = ROOT / ('demonstrations/processed/phase4_assembly/'
                 'v1_rotate_joint_sim_17s_CANDIDATE.csv')
JOINTS = [f'{name}_joint' for name in
          ('shoulder_pan', 'shoulder_lift', 'elbow',
           'wrist_1', 'wrist_2', 'wrist_3')]


def read_rotate_endpoint():
    with ROTATE.open(newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != ['time'] + JOINTS:
            raise RuntimeError('Unexpected ROTATE trajectory columns.')
        rows = list(reader)
        if len(rows) != 71:
            raise RuntimeError('Unexpected ROTATE trajectory length.')
        return [float(rows[-1][j]) for j in JOINTS]


class RetreatPreview(Node):
    def __init__(self):
        super().__init__('v1_retreat_plan_only')
        self.last_joint_state = None
        self.last_joint_time = None
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian = self.create_client(GetCartesianPath,
                                            '/compute_cartesian_path')
        self.display = self.create_publisher(DisplayTrajectory,
                                              '/display_planned_path', 1)

    def on_joint(self, msg):
        if len(msg.name) == len(msg.position) and all(
                j in msg.name for j in JOINTS):
            self.last_joint_state = msg
            self.last_joint_time = time.monotonic()

    def wait_live(self, timeout_s=8.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.15)
            if (self.last_joint_state is not None and
                self.tf_buffer.can_transform('base', 'gripper_tcp', Time())):
                return
        raise RuntimeError('Missing live joint states or base->gripper_tcp TF.')

    def run(self):
        expected = read_rotate_endpoint()
        self.wait_live()
        if time.monotonic() - self.last_joint_time > 1.0:
            raise RuntimeError('Joint state is stale.')
        observed = [self.last_joint_state.position[
            self.last_joint_state.name.index(j)] for j in JOINTS]
        if not all(math.isfinite(v) for v in observed):
            raise RuntimeError('Non-finite joint position.')
        error = max(abs(a-b) for a, b in zip(observed, expected))
        print(f'Live joint error vs V1 ROTATE endpoint: {error:.6f} rad')
        if error > 0.01:
            raise RuntimeError('Robot is not at validated V1 ROTATE endpoint.')

        transform = self.tf_buffer.lookup_transform('base', 'gripper_tcp', Time())
        xyz = transform.transform.translation
        q = transform.transform.rotation
        print('Current gripper_tcp (base frame): '
              f'x={xyz.x:.6f}, y={xyz.y:.6f}, z={xyz.z:.6f} m')
        print('Current quaternion (xyzw): '
              f'[{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]')

        # Candidate ONLY: +base Z, no rotation. Not an approved physical path.
        # Small waypoints expose any partial-path failures.
        waypoints = []
        for lift_m in (0.005, 0.010, 0.015, 0.020, 0.025, 0.030):
            pose = Pose()
            pose.position.x = xyz.x
            pose.position.y = xyz.y
            pose.position.z = xyz.z + lift_m
            pose.orientation.x = q.x
            pose.orientation.y = q.y
            pose.orientation.z = q.z
            pose.orientation.w = q.w
            waypoints.append(pose)

        if not self.cartesian.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('/compute_cartesian_path is not available.')

        request = GetCartesianPath.Request()
        request.header.frame_id = 'base'
        request.group_name = 'ur_onrobot_manipulator'
        request.link_name = 'gripper_tcp'
        request.start_state = RobotState()
        request.start_state.joint_state = self.last_joint_state
        request.waypoints = waypoints
        request.max_step = 0.002
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.03
        request.max_acceleration_scaling_factor = 0.03

        future = self.cartesian.call_async(request)
        end = time.monotonic() + 15.0
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cartesian planning service timed out.')

        response = future.result()
        traj = response.solution.joint_trajectory
        print(f'Cartesian fraction: {response.fraction:.4f}')
        print(f'MoveIt error code: {response.error_code.val}')
        print(f'Planned joint trajectory points: {len(traj.points)}')
        if response.error_code.val != 1 or response.fraction < 0.999:
            raise RuntimeError('Retreat planning incomplete. NO ROBOT MOVEMENT.')
        if not traj.points:
            raise RuntimeError('Empty retreat trajectory.')
        if set(traj.joint_names) != set(JOINTS):
            raise RuntimeError('Unexpected joint names in retreat trajectory.')
        q_index = [traj.joint_names.index(j) for j in JOINTS]
        path = [[pt.positions[i] for i in q_index] for pt in traj.points]
        if any(len(pt.positions) != len(traj.joint_names) for pt in traj.points):
            raise RuntimeError('Malformed joint trajectory.')
        # A Cartesian IK solution can represent the same revolute-joint
        # orientation with an additional 2*pi turn. That is NOT a safe
        # controller command. Diagnose each branch before inspecting any
        # corrected plan; NEVER execute this preview.
        tau = 2.0 * math.pi
        print('\n========== RAW JOINT BRANCH DIAGNOSTIC ==========')
        print('joint                          live      raw_first   raw_last   offset_turns')
        offsets = []
        non_wrist_wrap = False
        for idx, name in enumerate(JOINTS):
            raw_first = path[0][idx]
            raw_last = path[-1][idx]
            turns = round((raw_first - observed[idx]) / tau)
            residual = raw_first - turns*tau - observed[idx]
            print(f'{name:27s} {observed[idx]:+9.5f} {raw_first:+11.5f} '
                  f'{raw_last:+10.5f} {turns:+5d}  '
                  f'(corrected start error {residual:+.5f})')
            if abs(residual) > 0.03:
                raise RuntimeError(
                    f'{name}: raw first point is not equivalent to live state '
                    'within 0.03 rad. No branch correction allowed.')
            if turns != 0 and name != 'wrist_3_joint':
                non_wrist_wrap = True
            offsets.append(turns)

        if non_wrist_wrap:
            raise RuntimeError(
                'Unexpected 2*pi representation on a joint other than wrist_3. '
                'Review joint model and full trajectory before normalising.')

        # Constant 2*pi branch correction only; physical pose remains the
        # same for a revolute joint. This is a proposed display trajectory,
        # NOT a controller trajectory or an execution authorisation.
        corrected = [
            [value - tau*turns for value, turns in zip(row, offsets)]
            for row in path
        ]
        if not all(math.isfinite(x) for row in corrected for x in row):
            raise RuntimeError('Non-finite corrected trajectory.')
        start_error = max(abs(a-b) for a,b in zip(corrected[0],observed))
        max_total = max(abs(a-b) for row in corrected
                        for a,b in zip(row,observed))
        max_step = max((abs(a-b) for prev,nxt in zip(corrected, corrected[1:])
                        for a,b in zip(prev,nxt)), default=0.0)
        print('\n========== CORRECTED PLAN-ONLY DIAGNOSTIC ==========')
        print(f'Joint offsets (turns): {offsets}')
        print(f'Corrected first-point error: {start_error:.6f} rad')
        print(f'Corrected max change from live: {max_total:.6f} rad')
        print(f'Corrected max consecutive joint step: {max_step:.6f} rad')
        print('Corrected endpoint:')
        for name, final in zip(JOINTS, corrected[-1]):
            print(f'  {name}: {final:+.6f} rad')

        if start_error > 0.03 or max_total > 0.35 or max_step > 0.05:
            raise RuntimeError('Corrected retreat joint-change guard exceeded.')
        # Explicitly check conventional UR3e configuration range. This is
        # only an extra diagnostic guard, not a replacement for joint-model
        # bounds/scene collision checking and real robot safety approval.
        if any(abs(v) > tau + 0.001 for row in corrected for v in row):
            raise RuntimeError('Corrected joint value outside +/-2*pi guard.')
        if len(traj.points) != len(corrected):
            raise RuntimeError('Trajectory length changed.')

        for pt, row in zip(traj.points, corrected):
            for joint_idx, name in enumerate(JOINTS):
                pt.positions[traj.joint_names.index(name)] = row[joint_idx]

        msg = DisplayTrajectory()
        msg.model_id = 'ur_onrobot'
        msg.trajectory_start = response.start_state
        msg.trajectory.append(response.solution)
        self.display.publish(msg)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        print('\nRETREAT REPRESENTATION CHECK: PASS (PLAN ONLY)')
        print('Candidate: +30 mm along base Z while maintaining current TCP orientation.')
        print('WARNING: MoveIt may not contain the physical fixture/part collision geometry.')
        print('Visual geometry and verified fixture clearance are REQUIRED before motion.')
        print('Any eventual executor must revalidate branch, start state and collision clearance.')
        print('THIS SCRIPT DOES NOT SEND A TRAJECTORY TO THE ROBOT.')


def main():
    rclpy.init()
    node = RetreatPreview()
    try:
        node.run()
    except Exception as exc:
        print(f'BLOCKED: {exc}', file=sys.stderr)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
