#!/usr/bin/env python3
"""V1 post-release retreat: PLAN by default, optional guarded REAL execution.

Requires the known UR3e /scaled_joint_trajectory_controller, live robot
joint states and TF, and a live MoveIt Cartesian service. Never assumes the
assembly fixture is present in the planning scene. Run with no flags first;
only use --execute after checking the path and real physical clearance.
"""

import argparse
import csv
from datetime import datetime
import math
import os
import signal
import sys
import time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '0':
    raise SystemExit('BLOCKED: ROS_DOMAIN_ID must be 0 for real-robot observation.')

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetCartesianPath
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
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
    def __init__(self, execute_requested=False):
        super().__init__('v1_retreat_guarded')
        self.execute_requested = execute_requested
        self.stop_requested = False
        self.goal_inflight = False
        self.record_active = False
        self.samples = []
        self.record_start = 0.0
        self.last_joint_state = None
        self.last_joint_time = None
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian = self.create_client(GetCartesianPath,
                                            '/compute_cartesian_path')
        self.display = self.create_publisher(
            DisplayTrajectory, '/display_planned_path',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.hardware = self.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def on_joint(self, msg):
        if len(msg.name) == len(msg.position) and all(
                j in msg.name for j in JOINTS):
            self.last_joint_state = msg
            self.last_joint_time = time.monotonic()
            if self.record_active:
                self.samples.append([self.last_joint_time - self.record_start] + [
                    float(msg.position[msg.name.index(j)]) for j in JOINTS])

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
        self.start_xyz = (xyz.x, xyz.y, xyz.z)
        self.start_quat = (q.x, q.y, q.z, q.w)
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

        # A constant integer-turn translation must have NO effect on
        # velocities or accelerations, but all trajectory time stamps must
        # already be usable for a physical FollowJointTrajectory goal.
        times = [(p.time_from_start.sec +
                  p.time_from_start.nanosec * 1e-9) for p in traj.points]
        if len(times) < 3 or not all(math.isfinite(t) for t in times):
            raise RuntimeError('Missing/invalid trajectory timestamps.')
        if any(b <= a for a, b in zip(times, times[1:])):
            raise RuntimeError('Non-increasing trajectory time stamps.')
        if not 0.2 <= times[-1] <= 120.0:
            raise RuntimeError('Unexpected trajectory duration.')
        vmax = max(abs(corrected[k+1][j] - corrected[k][j]) /
                   (times[k+1] - times[k])
                   for k in range(len(times)-1) for j in range(6))
        print(f'Plan duration: {times[-1]:.3f} s')
        print(f'Maximum segment joint speed: {vmax:.5f} rad/s')
        if vmax > 0.15:
            raise RuntimeError('Joint speed guard exceeded.')

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
        if not self.execute_requested:
            print('PLAN ONLY: no trajectory sent to robot. (Use --execute only after reviewing clearance.)')
            try:
                input('Keep RViz open and inspect the trajectory. Press ENTER to close this preview: ')
            except EOFError:
                pass
            return

        self.execute_checked(traj, observed, corrected, offsets, times[-1])


    def request(self, client, req, timeout=6.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Required controller-manager service unavailable.')
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.15)
        if not future.done() or future.result() is None:
            raise RuntimeError('Controller-manager request timed out.')
        return future.result()

    def live_joints(self, timeout_s=3.0):
        # After a blocking input() prompt, ROS callbacks have not been spun.
        # Require a NEW joint-state callback received after this function
        # starts, rather than relying on a pre-confirmation cached sample.
        requested_at = time.monotonic()
        deadline = requested_at + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.last_joint_state is not None and
                    self.last_joint_time is not None and
                    self.last_joint_time >= requested_at and
                    time.monotonic() - self.last_joint_time < 0.5):
                values = [float(self.last_joint_state.position[
                    self.last_joint_state.name.index(j)]) for j in JOINTS]
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Non-finite live joint state.')
                return values
        raise RuntimeError('No NEW joint state received within 3 seconds. '
                           'No trajectory sent; check /joint_states.')

    def execute_checked(self, traj, initial, corrected, offsets, duration):
        if offsets != [0, 0, 0, 0, 0, 1]:
            raise RuntimeError('Physical branch guard requires known +2pi wrist_3 only.')
        hw = self.request(self.hardware, ListHardwareComponents.Request()).component
        print('Hardware:', [(h.name, h.class_type) for h in hw])
        if not any('ur_robot_driver/URPositionHardwareInterface' in h.class_type
                   for h in hw):
            raise RuntimeError('Actual UR driver hardware is not verified. No execution.')
        if any('mock_components' in h.class_type or
               'test_hardware_components' in h.class_type for h in hw):
            raise RuntimeError('Fake/test hardware detected. No REAL execution.')

        states = {c.name: c.state.lower()
                  for c in self.request(self.controllers, ListControllers.Request()).controller}
        print('Arm controller:', states.get('scaled_joint_trajectory_controller'))
        if (states.get('scaled_joint_trajectory_controller') != 'active'
                or states.get('joint_trajectory_controller') == 'active'):
            raise RuntimeError('Unexpected active joint controller. No execution.')
        if not self.arm.wait_for_server(timeout_sec=5):
            raise RuntimeError('Scaled trajectory action not available.')

        print('\n===== PHYSICAL RETREAT OPERATOR REVIEW =====')
        print('Distance: +30 mm in BASE Z; TCP orientation stays fixed.')
        print('RG2 must be OPEN to 20 mm and disengaged from V1.')
        print('Inspect the full RViz path AND actual RG2, fixture, component, trolley.')
        print('MoveIt may NOT contain actual fixture collision geometry.')
        print('Ensure gripper can lift without catching the part or interfering with fixture.')
        print('Only one arm command source, no freedrive, keep pendant STOP in reach.')
        print('Monitor the entire motion physically. Stop immediately if anything is unexpected.')
        print('If any doubt, CANCEL here, do not execute.')
        approval = input('Type EXECUTE_V1_RETREAT_30MM only after these checks: ').strip()
        if approval != 'EXECUTE_V1_RETREAT_30MM':
            print('CANCELLED. No robot movement.')
            return

        # The exact plan is invalid if the arm has moved while the user reviewed it.
        latest = self.live_joints()
        latest_err = max(abs(a-b) for a,b in zip(initial,latest))
        print(f'Latest live-state drift since planning: {latest_err:.6f} rad')
        if latest_err > 0.002:
            raise RuntimeError('Robot moved while planning was reviewed. Replan required.')
        # Reconfirm hardware/controller state immediately before sending.
        cstates = {c.name: c.state.lower()
                   for c in self.request(self.controllers, ListControllers.Request()).controller}
        if cstates.get('scaled_joint_trajectory_controller') != 'active':
            raise RuntimeError('Scaled controller inactive. No execution.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(seconds=5).to_msg()
        if self.stop_requested:
            raise RuntimeError('Interrupted before retreat command')
        print('Sending ONE guarded +30 mm retreat trajectory to the real robot.')
        self.record_start = time.monotonic()
        self.samples = []
        self.record_active = True
        handle = None
        outcome = None
        self.goal_inflight = True
        try:
            send = self.arm.send_goal_async(goal)
            deadline = time.monotonic() + 6.0
            while rclpy.ok() and not send.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not send.done() or send.result() is None:
                raise RuntimeError('Retreat acceptance outcome UNKNOWN; check pendant/controller')
            handle=send.result()
            if not handle.accepted:
                raise RuntimeError('Joint trajectory goal rejected')
            outcome = handle.get_result_async()
            if self.stop_requested:
                raise RuntimeError('Ctrl+C during retreat goal acceptance')
            deadline = time.monotonic() + max(60.0, duration * 5.0 + 20.0)
            while rclpy.ok() and not outcome.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.stop_requested:
                    raise RuntimeError('Ctrl+C during physical retreat')
                # Check latest live sample only; do not block while waiting
                # for another callback inside this loop.
                if self.last_joint_time is None or time.monotonic()-self.last_joint_time>1.0:
                    raise RuntimeError('Retreat live joint-state stream lost')
                live = [float(self.last_joint_state.position[
                    self.last_joint_state.name.index(j)]) for j in JOINTS]
                path_error = min(max(abs(a-b) for a,b in zip(live,row))
                                 for row in corrected)
                if path_error > 0.10:
                    raise RuntimeError('Large deviation from retreat path')
            if not outcome.done() or outcome.result() is None:
                raise RuntimeError('Retreat result timed out or is UNKNOWN')
            result = outcome.result()
            if (result.status != GoalStatus.STATUS_SUCCEEDED or
                    result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise RuntimeError('Controller reported retreat failure: '
                                   f'status={result.status} code={result.result.error_code}')
            if self.stop_requested:
                raise RuntimeError('Interrupted after retreat result')
            final = self.live_joints()
            err = max(abs(a-b) for a,b in zip(final,corrected[-1]))
            print(f'Final joint endpoint error: {err:.6f} rad')
            if err > 0.015:
                raise RuntimeError('Retreat ended outside expected joint tolerance.')

            # Independently confirm actual TCP displacement in BASE coordinates.
            tf = self.tf_buffer.lookup_transform('base', 'gripper_tcp', Time())
            v = tf.transform.translation
            qr = tf.transform.rotation
            dxyz = (v.x - self.start_xyz[0],
                    v.y - self.start_xyz[1],
                    v.z - self.start_xyz[2])
            dot = abs(sum(a*b for a,b in zip(self.start_quat,
                        (qr.x,qr.y,qr.z,qr.w))))
            angle = 2*math.degrees(math.acos(min(1.0,max(0.0,dot))))
            print(f'Observed TCP delta (mm): {[round(x*1000,2) for x in dxyz]}')
            print(f'Observed orientation change: {angle:.3f} deg')
            if abs(dxyz[2]-0.030) > 0.003 or math.hypot(*dxyz[:2]) > 0.003 or angle > 3:
                raise RuntimeError('Final TCP pose differs from requested 30 mm vertical retreat.')
            print('\nPHYSICAL RETREAT: CONTROLLER SUCCESS + JOINT/TCP ENDPOINT PASS')
            print('Visually verify the gripper is clear. HOME IS NOT COMMANDED.')
        except BaseException:
            self.stop_requested=True
            if handle is not None and (outcome is None or not outcome.done()):
                print('RETREAT: requesting controller cancellation (NOT an E-stop).',
                      file=sys.stderr,flush=True)
                try:
                    cancel=handle.cancel_goal_async()
                    until=time.monotonic()+6.0
                    while not cancel.done() and time.monotonic()<until:
                        rclpy.spin_once(self,timeout_sec=.1)
                    accepted=(cancel.done() and cancel.result() is not None and
                              bool(cancel.result().goals_canceling))
                    if accepted:
                        if outcome is None:
                            outcome=handle.get_result_async()
                        until=time.monotonic()+8.0
                        while not outcome.done() and time.monotonic()<until:
                            rclpy.spin_once(self,timeout_sec=.1)
                        accepted=(outcome.done() and outcome.result() is not None and
                                  outcome.result().status==GoalStatus.STATUS_CANCELED)
                    if accepted:
                        print('RETREAT: controller CANCELED (verify physical stop).',
                              file=sys.stderr,flush=True)
                    else:
                        print('RETREAT: cancel result UNKNOWN; use pendant stop if moving.',
                              file=sys.stderr,flush=True)
                except BaseException as exc:
                    print(f'RETREAT: cancel FAILED: {exc}. Use pendant stop if moving.',
                          file=sys.stderr,flush=True)
            elif handle is None and self.goal_inflight:
                print('RETREAT: goal acceptance UNKNOWN; verify physical robot state.',
                      file=sys.stderr,flush=True)
            raise
        finally:
            self.goal_inflight=False
            self.record_active = False
            folder = ROOT / 'results/physical/v1_remaining'
            folder.mkdir(parents=True, exist_ok=True)
            filename = folder / ('retreat_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
            with filename.open('x', newline='') as fp:
                writer = csv.writer(fp)
                writer.writerow(['elapsed_s'] + JOINTS)
                writer.writerows(self.samples)
            print(f'Physical retreat joint log ({len(self.samples)} samples): {filename}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Enable a physical trajectory ONLY with live guards and typed confirmation.')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RetreatPreview(execute_requested=args.execute)
    def retreat_sigint(_signum,_frame):
        if node.goal_inflight:
            node.stop_requested=True
            print('\nCtrl+C: RETREAT cancellation requested; controller response pending. '
                  'Use pendant stop if motion persists.', file=sys.stderr, flush=True)
        else:
            raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,retreat_sigint)
    try:
        node.run()
    except BaseException as exc:
        print(f'RETREAT STOPPED: {type(exc).__name__}: {exc}', file=sys.stderr)
        raise SystemExit(130 if isinstance(exc,KeyboardInterrupt) else 1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
