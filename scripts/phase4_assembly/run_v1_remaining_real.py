#!/usr/bin/env python3
"""Individual supervised V1 steps on a physical UR3e; never runs a full sequence."""
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
JOINTS = [f'{name}_joint' for name in
          ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')]
STAGES = {
    'align_insert': ('config/trajectories/v1/v1_align_to_insertion_joint_plan_001.csv', 10, 8.0),
    'insert': ('demonstrations/processed/phase4_assembly/v1_insert_joint_sim_13s_CANDIDATE.csv', 9, 2.0),
    'rotate': ('demonstrations/processed/phase4_assembly/v1_rotate_joint_sim_17s_CANDIDATE.csv', 71, 2.0),
}
ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
FORCE_TOPIC = '/force_torque_sensor_broadcaster/wrench'


def err(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def load_stage(stage):
    rel, count, scale = STAGES[stage]
    path = ROOT / rel
    with path.open(newline='') as f:
        rows = list(csv.DictReader(f))
    if len(rows) != count:
        raise RuntimeError(f'{stage}: expected {count} points, got {len(rows)}')
    t = [float(r['time']) for r in rows]
    q = [[float(r[j]) for j in JOINTS] for r in rows]
    if (not all(math.isfinite(v) for p in q for v in p)
            or not all(math.isfinite(v) for v in t)
            or not all(b > a for a, b in zip(t, t[1:]))):
        raise RuntimeError('Invalid trajectory data')
    step = max(err(a, b) for a, b in zip(q, q[1:]))
    speed = max(abs((q[i+1][j] - q[i][j]) /
                    ((t[i+1] - t[i]) * scale))
                for i in range(len(q)-1) for j in range(6))
    print(f'File: {path.name} | points: {len(q)} | slowed duration: {(t[-1]-t[0])*scale:.2f}s')
    acc = max(abs(((q[i+1][j]-q[i][j])/(t[i+1]-t[i]) -
                   (q[i][j]-q[i-1][j])/(t[i]-t[i-1])) /
                  ((t[i+1]-t[i-1])/2)) / (scale*scale)
              for i in range(1, len(q)-1) for j in range(6))
    print(f'Max step: {step:.5f} rad | speed: {speed:.5f} rad/s | acceleration: {acc:.5f} rad/s²')
    if step > 0.03 or speed > 0.12 or acc > 0.05:
        raise RuntimeError('Trajectory step/speed/acceleration check failed')
    return t, q, scale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stage', choices=list(STAGES) + ['release'])
    args = ap.parse_args()
    stage = args.stage
    contact = stage in ('insert', 'rotate')
    if stage != 'release':
        t, q, scale = load_stage(stage)
    else:
        # Release only if actually at the end of our saved ROTATE trajectory.
        _, q_rotate, _ = load_stage('rotate')

    rclpy.init()
    node = rclpy.create_node('physical_v1_' + stage)
    state = {'joints': None, 'wrench': None, 'wrench_when': 0.0}
    rows_out = []
    recording = {'on': False, 'start': 0.0}

    def six_wrench(w):
        return [w.force.x, w.force.y, w.force.z,
                w.torque.x, w.torque.y, w.torque.z]

    def joints_cb(msg):
        state['joints'] = dict(zip(msg.name, msg.position))
        if recording['on'] and all(j in state['joints'] for j in JOINTS):
            x = state['joints']
            wr = state['wrench'] if state['wrench'] is not None else [float('nan')] * 6
            rows_out.append([time.monotonic()-recording['start'], x.get('finger_width', float('nan'))]
                            + [x[j] for j in JOINTS] + list(wr))

    def wrench_cb(msg):
        state['wrench'] = six_wrench(msg.wrench)
        state['wrench_when'] = time.monotonic()

    node.create_subscription(JointState, '/joint_states', joints_cb, qos_profile_sensor_data)
    if contact:
        node.create_subscription(WrenchStamped, FORCE_TOPIC, wrench_cb, qos_profile_sensor_data)

    def fresh_joints():
        state['joints'] = None
        deadline = time.monotonic() + 6
        while state['joints'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.15)
        if state['joints'] is None:
            raise RuntimeError('Joint-state feedback missing')
        return state['joints']

    srv = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not srv.wait_for_service(timeout_sec=7):
        raise RuntimeError('Controller manager unavailable')
    future = srv.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=8)
    if future.result() is None:
        raise RuntimeError('Controller status unavailable')
    ctrls = {c.name: c.state for c in future.result().controller}
    needed = ['joint_state_broadcaster', 'scaled_joint_trajectory_controller',
              'finger_width_trajectory_controller']
    if any(ctrls.get(c) != 'active' for c in needed):
        raise RuntimeError('Required controller inactive: ' + str(ctrls))

    x = fresh_joints()
    if any(j not in x for j in JOINTS) or 'finger_width' not in x:
        raise RuntimeError('Missing robot joint feedback')
    current = [x[j] for j in JOINTS]
    target_start = q_rotate[-1] if stage == 'release' else q[0]
    start_err = err(current, target_start)
    width = x['finger_width']
    print(f'Actual starting joint error: {start_err:.6f} rad; gripper: {width*1000:.2f} mm')
    if start_err > 0.007:
        raise RuntimeError('Robot not at required starting pose; do not bypass')
    if not math.isfinite(width) or not (0.005 <= width <= 0.017):
        raise RuntimeError('Gripper opening unexpected; check the held part')

    if contact:
        deadline = time.monotonic() + 5
        while state['wrench'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['wrench'] is None or not all(math.isfinite(v) for v in state['wrench']):
            raise RuntimeError(f'No valid F/T data on {FORCE_TOPIC}; do not start contact movement')
        baseline = list(state['wrench'])
        print('F/T baseline [Fx,Fy,Fz,Tx,Ty,Tz]:', [round(v, 3) for v in baseline])
    else:
        baseline = None

    if stage == 'release':
        trajectory = JointTrajectory(joint_names=['finger_width'])
        point = JointTrajectoryPoint()
        point.positions = [0.020]
        point.time_from_start = Duration(sec=3)
        trajectory.points = [point]
        action_name = '/finger_width_trajectory_controller/follow_joint_trajectory'
    else:
        trajectory = JointTrajectory(joint_names=JOINTS)
        t0 = t[0]
        for i, angles in enumerate(q):
            point = JointTrajectoryPoint()
            point.positions = list(angles)
            if i in (0, len(q)-1):
                point.velocities = [0.0]*6
            else:
                dt = (t[i+1] - t[i-1]) * scale
                point.velocities = [(q[i+1][j]-q[i-1][j])/dt for j in range(6)]
            ns = round(((t[i]-t0)*scale + 0.30) * 1e9)
            sec, nanosec = divmod(ns, 1_000_000_000)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(point)
        action_name = ACTION

    client = ActionClient(node, FollowJointTrajectory, action_name)
    if not client.wait_for_server(timeout_sec=7):
        raise RuntimeError('Trajectory action unavailable')

    print('\nTHIS PHYSICALLY COMMANDS:', stage.upper())
    print('No MoveIt environmental collision check. Arm controller speed is scaled by the pendant.')
    if contact:
        print('Contact motion: monitor the part, fixture and robot F/T live.')
        print('Software force-delta stop is provisional; not a certified safety limit.')
    if stage == 'release':
        print('CAUTION: Only release if the part is fully seated and supported by the fixture.')
    print('Operator must stay outside robot envelope with pendant Stop available.')
    if input(f'Type {stage.upper()} to run: ').strip() != stage.upper():
        print('CANCELLED')
        return

    x = fresh_joints()
    if err([x[j] for j in JOINTS], target_start) > 0.007:
        raise RuntimeError('Robot moved since initial preflight')
    if not math.isfinite(x['finger_width']) or not (0.005 <= x['finger_width'] <= 0.017):
        raise RuntimeError('Gripper opening changed')
    if contact:
        state['wrench'] = None
        deadline = time.monotonic() + 5.0
        while state['wrench'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['wrench'] is None or not all(math.isfinite(v) for v in state['wrench']):
            raise RuntimeError('No fresh valid F/T data before movement')
        baseline = list(state['wrench'])
        print('Refreshed F/T baseline:', [round(v, 3) for v in baseline])

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = Duration(sec=40)
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
    if sent.result() is None or not sent.result().accepted:
        raise RuntimeError('Goal not accepted')
    handle = sent.result()
    reply = handle.get_result_async()
    recording['start'] = time.monotonic()
    recording['on'] = True
    abort_reason = None
    try:
        while not reply.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if contact:
                wr = state['wrench']
                if wr is None or time.monotonic() - state['wrench_when'] > 1.0:
                    abort_reason = 'F/T feedback lost'
                elif max(abs(wr[i]-baseline[i]) for i in range(3)) > 15.0:
                    abort_reason = 'F/T force change exceeded provisional 15 N limit'
                elif max(abs(wr[i]-baseline[i]) for i in range(3,6)) > 1.5:
                    abort_reason = 'F/T torque change exceeded provisional 1.5 Nm limit'
                gripper_now = (state['joints'] or {}).get('finger_width', float('nan'))
                if not math.isfinite(gripper_now) or abs(gripper_now - width) > 0.005:
                    abort_reason = 'Gripper opening changed unexpectedly'
                if abort_reason:
                    print('ABORT:', abort_reason, '— use pendant Stop if movement persists')
                    cancel = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
                    break
    except KeyboardInterrupt:
        print('STOP REQUESTED — USE PENDANT STOP IF MOVING')
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        abort_reason = 'Operator cancelled'
    finally:
        recording['on'] = False

    folder = ROOT / 'results/physical/v1_remaining'
    folder.mkdir(parents=True, exist_ok=True)
    log_path = folder / f'{stage}_{datetime.now():%Y%m%d_%H%M%S}.csv'
    with log_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['elapsed_s','finger_width_m'] + JOINTS +
                        ['Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm'])
        writer.writerows(rows_out)
    print('Physical feedback log:', log_path)
    if abort_reason:
        raise RuntimeError(abort_reason + '; do not continue')
    if not reply.done() or reply.result() is None:
        raise RuntimeError('Result unavailable; do not continue')
    result = reply.result()
    print('Goal status:', result.status, 'error:', result.result.error_code)
    print('Controller message:', result.result.error_string)
    if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
        raise RuntimeError('Controller did not complete step')

    x = fresh_joints()
    if stage == 'release':
        print('Measured RG2 opening:', round(x['finger_width']*1000, 2), 'mm')
        if not math.isfinite(x['finger_width']) or abs(x['finger_width'] - 0.020) > 0.004:
            raise RuntimeError('Release width not verified')
    else:
        endpoint_err = err([x[j] for j in JOINTS], q[-1])
        print('Final arm joint error:', round(endpoint_err, 6), 'rad')
        print('Measured gripper:', round(x['finger_width']*1000, 2), 'mm')
        if endpoint_err > 0.007:
            raise RuntimeError('Target joint pose not reached')
    print('STAGE PASSED:', stage.upper())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, AssertionError) as exc:
        raise SystemExit('STOP: ' + str(exc))
