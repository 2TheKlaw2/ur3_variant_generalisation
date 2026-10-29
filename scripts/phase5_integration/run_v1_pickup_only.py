#!/usr/bin/env python3
"""Guarded V1 physical PICKUP-ONLY diagnosis, derived from validated pilot runner (ROS2 Humble).

Default: OFFLINE CSV PREFLIGHT, NO ROBOT MOVEMENT. PILOT BUILD.
--execute: real hardware required; stage-by-stage physical operator confirmation.
--execute --auto: one whole-cycle confirmation for SUPERVISED physical pilot only.

IMPORTANT: this is pilot code, not the formal L0 implementation.
IMPORTANT: the repository CSVs were also used in simulation. Verify each is
identical to the trajectory used in the individually validated physical tests,
and verify the x2 timing multiplier before allowing execution. Use --auto only after separately validating the changed RG2 grip/open settings.

For --pickup-only: require HOME -> OPEN40 -> REACH -> ALIGN_PICKUP -> STOP.
Physical recovery from an E-stop must happen separately and safely first.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import secrets
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
JOINTS = [f'{x}_joint' for x in (
    'shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')]
STAGES = [
    ('REACH', 'demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv'),
    ('ALIGN_PICKUP', 'config/trajectories/v1/v1_align_to_grip_joint_plan_001.csv'),
    ('TRANSPORT', 'demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv'),
    ('ALIGN_INSERT', 'config/trajectories/v1/v1_align_to_insertion_joint_plan_001.csv'),
    ('INSERT', 'demonstrations/processed/phase4_assembly/v1_insert_joint_sim_13s_CANDIDATE.csv'),
    ('ROTATE', 'demonstrations/processed/phase4_assembly/v1_rotate_joint_sim_17s_CANDIDATE.csv'),
]
# Baseline files confirmed by the successful 2026-10-29 integrated physical run.
# Fail closed if any physical motion CSV was edited after that run.
EXPECTED_SHA12 = {
    'REACH': 'dd5c175c9b1e',
    'ALIGN_PICKUP': '69dfcecc5ebd',
    'TRANSPORT': 'f4335b3ef675',
    'ALIGN_INSERT': '6372a2397601',
    'INSERT': 'fe2efc060452',
    'ROTATE': 'c7916592ed76',
}
RETREAT = ROOT / 'scripts/phase4_assembly/v1_retreat_guarded_auto.py'
HOME = ROOT / 'config/saved_poses/v1_home_for_moveit.yaml'
ARM_ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
GRIPPER_ACTION = '/finger_width_trajectory_controller/follow_joint_trajectory'
# V1 RG2 pilot settings, updated after the 2026-10-29 successful run.
# Keep trajectory CSVs unchanged; GRIP 8.2 mm is a NEW setting to validate.
RG2_OPEN_M = 0.040
RG2_GRIP_M = 0.0082
RG2_OPEN_DURATION_S = 3.0
RG2_GRIP_DURATION_S = 2.0
MAX_JOINT_STEP = .09   # maximum adjacent CSV sample jump, rad
MAX_SEGMENT_SPEED = .35  # rad/s after 2x retiming, conservative software check
START_TOL = .006  # maximum live-to-CSV start discrepancy (rad)
END_TOL = .012  # max arm discrepancy at stage endpoint (rad)
TAU = 2 * math.pi


def read_motion(path):
    if not path.is_file():
        raise RuntimeError(f'Missing file: {path}')
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != ['time'] + JOINTS:
            raise RuntimeError(f'Unexpected trajectory header: {path}')
        rows = list(reader)
    if len(rows) < 2:
        raise RuntimeError(f'Too few points: {path}')
    t = [float(r['time']) for r in rows]
    q = [[float(r[j]) for j in JOINTS] for r in rows]
    if not all(math.isfinite(v) for row in q for v in row):
        raise RuntimeError(f'Invalid joint data: {path}')
    if not all(math.isfinite(v) for v in t) or abs(t[0]) > 0.001:
        raise RuntimeError(f'Invalid times: {path}')
    if any(b <= a for a,b in zip(t,t[1:])):
        raise RuntimeError(f'Non-monotonic times: {path}')
    if any(abs(v) > TAU + .001 for row in q for v in row):
        raise RuntimeError(f'Joint outside conservative +/-2pi software guard: {path}')
    # 2x matches the physically executed 17s ROTATE -> 34s; must verify
    # separately for all other files before any integrated physical run.
    t = [2*v for v in t]
    max_step = max(abs(b[j]-a[j]) for a,b in zip(q,q[1:]) for j in range(6))
    max_speed = max(abs(q[k+1][j]-q[k][j])/(t[k+1]-t[k])
                    for k in range(len(t)-1) for j in range(6))
    if max_step > MAX_JOINT_STEP or max_speed > MAX_SEGMENT_SPEED:
        raise RuntimeError(f'Joint step/speed guard failed: {path} '
                           f'step={max_step:.4f}, speed={max_speed:.4f}')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(path=str(path), name=path.stem, t=t, q=q,
                max_step=max_step, max_speed=max_speed,
                sha256=digest)


def load_home():
    # Keep non-ROS preflight usable without adding a YAML dependency.
    text = HOME.read_text()
    import re
    found = re.search(r'positions_rad:\s*((?:\s*-\s*-?\d+(?:\.\d+)?\s*){6})', text)
    if not found or '192.168.0.192' not in text:
        raise RuntimeError('HOME file missing expected UR3e IP or six joint positions')
    q = [float(v) for v in re.findall(r'-\s*(-?\d+(?:\.\d+)?)', found.group(1))]
    if len(q) != 6:
        raise RuntimeError('Cannot parse saved HOME')
    return q


def err(a,b):
    return max(abs(x-y) for x,y in zip(a,b))


def preflight():
    home = load_home()
    plans = {name:read_motion(ROOT / relative) for name,relative in STAGES}
    print('\n=========== V1 COMBINED PREFLIGHT ===========')
    for name in plans:
        p = plans[name]
        print(f'{name:14s} {len(p["q"]):4d} points '
              f'{p["t"][-1]:7.2f}s @ x2  max_step={p["max_step"]:.5f} '
              f'max_speed={p["max_speed"]:.4f}  SHA256={p["sha256"][:12]}')
    for name, plan in plans.items():
        if plan['sha256'][:12] != EXPECTED_SHA12[name]:
            raise RuntimeError(f'{name}: saved physical trajectory changed since validated V1 run; review required')
    print('All six motion CSV hashes match the 2026-10-29 physical run.')
    print('RG2 updated pilot targets: OPEN/RELEASE=40.0 mm @ 3.0 s; GRIP=8.2 mm @ 2.0 s')
    print('NOTE: New RG2 widths and faster GRIP are not yet validated together physically.')
    print('\nHandoff errors (rad):')
    prev_name,prev_end = 'SAVED_HOME',home
    for name in plans:
        gap=err(prev_end,plans[name]['q'][0])
        print(f'  {prev_name:14s} -> {name:14s}: {gap:.6f}')
        if gap > START_TOL:
            raise RuntimeError(f'Unacceptable {prev_name}->{name} handoff')
        prev_name,prev_end = name,plans[name]['q'][-1]
    if not RETREAT.is_file():
        raise RuntimeError(f'Missing validated retreat script: {RETREAT}')
    print('\nRETREAT script:',RETREAT)
    print('HOME file:',HOME)
    print('\nNOTICE: These paths are repository candidates; compare with the files'
          ' and retiming ACTUALLY used during the physical V1 runs.')
    print('PLAN/SCENE CLEARANCE NOT PROVED by CSV-only checks.')
    print('No robot motion sent by this preflight.\n')
    return home, plans


def confirm_updated_gripper_pilot():
    """Fail closed until changed RG2 targets passed a prior staged physical cycle.

    A PASS log only verifies software completion. Operator must STILL physically
    confirm that the actual part was securely gripped and correctly assembled.
    """
    folder = ROOT / 'results/physical/v1_integrated'
    if not folder.exists():
        raise RuntimeError('AUTO BLOCKED: no staged 40mm/8.2mm pilot record; run --execute first')
    expected_stages = ['OPEN', 'REACH', 'ALIGN_PICKUP', 'GRIP', 'TRANSPORT',
                       'ALIGN_INSERT', 'INSERT', 'ROTATE', 'RELEASE', 'RETREAT', 'HOME']
    for file in sorted(folder.glob('*/status.json'), reverse=True):
        try:
            r = json.loads(file.read_text())
            g = r.get('rg2_targets') or {}
            names = [item.get('name') for item in r.get('stages', [])]
            if (r.get('status') == 'PASS'
                    and r.get('trial_type') == 'PILOT_NOT_FORMAL_L0'
                    and not r.get('auto_mode', False)
                    and all(abs(float(g.get(k, -1))-v) < 0.001 for k,v in (
                        ('open_mm', 40.0), ('release_mm', 40.0),
                        ('grip_mm', 8.2), ('open_duration_s', 3.0),
                        ('grip_duration_s', 2.0)))
                    and names == expected_stages
                    and all(item.get('status') == 'PASS' for item in r['stages'])):
                print(f'Staged RG2 settings pilot found: {file}')
                print('Operator must still independently confirm secure grasp and assembly success.')
                return
        except (ValueError, OSError, TypeError, KeyError, AttributeError):
            continue
    raise RuntimeError('AUTO BLOCKED: no complete staged physical PASS using OPEN/RELEASE 40mm '
                       'and GRIP 8.2mm/2s. First run --execute with per-stage confirmations '
                       'and visually verify grasp, insertion, release and clearance.')


def run_actual(home,plans,home_only=False,home_only_plan=False,auto=False,pickup_only=False):
    if os.environ.get('ROS_DOMAIN_ID') != '0':
        raise RuntimeError('ROS_DOMAIN_ID must be 0 for the physical UR3e')
    # Lazily import ROS only when physical execution is explicitly requested.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
    from action_msgs.msg import GoalStatus
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState, DisplayTrajectory, RobotTrajectory

    class Robot(Node):
        def __init__(self):
            super().__init__('v1_physical_one_sequence')
            self.q=None; self.width=None; self.last_at=0; self.width_at=0; self.monitor=False
            self.records=[]; self.stage=''
            self.stop_requested=False; self.goal_inflight=False; self.active_goal_handle=None
            self.create_subscription(JointState,'/joint_states',self.joint_callback,
                                     qos_profile_sensor_data)
            self.hw=self.create_client(ListHardwareComponents,
                                      '/controller_manager/list_hardware_components')
            self.ctrl=self.create_client(ListControllers,
                                        '/controller_manager/list_controllers')
            self.state_valid=self.create_client(GetStateValidity,
                                                 '/check_state_validity')
            self.arm=ActionClient(self,FollowJointTrajectory,ARM_ACTION)
            self.grip=ActionClient(self,FollowJointTrajectory,GRIPPER_ACTION)
            self.display=self.create_publisher(DisplayTrajectory,
                                                '/display_planned_path',10)

        def joint_callback(self,msg):
            # UR joint and RG2 joint states may be published in separate messages.
            if len(msg.name) != len(msg.position):
                return
            now = time.monotonic()
            if 'finger_width' in msg.name:
                w = float(msg.position[msg.name.index('finger_width')])
                if math.isfinite(w):
                    self.width = w
                    self.width_at = now
            if not all(j in msg.name for j in JOINTS):
                return
            q = [float(msg.position[msg.name.index(j)]) for j in JOINTS]
            if any(not math.isfinite(x) for x in q):
                return
            self.q=q;self.last_at=now
            if self.monitor:
                self.records.append([datetime.now().isoformat(timespec='milliseconds'),
                                     self.stage]+q+[self.width])

        def fresh(self,sec=5):
            after=time.monotonic();deadline=after+sec
            while time.monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=.12)
                if self.q is not None and self.last_at>=after:
                    return list(self.q)
            raise RuntimeError('No NEW live joint state; no next command')

        def request(self,client,req,timeout=8):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('Required ROS service unavailable')
            fut=client.call_async(req);deadline=time.monotonic()+timeout
            while not fut.done() and time.monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=.1)
            if not fut.done() or fut.result() is None:
                raise RuntimeError('ROS service request timed out')
            return fut.result()

        def hardware(self):
            hw=self.request(self.hw,ListHardwareComponents.Request()).component
            types=[h.class_type for h in hw]
            if not any(t=='ur_robot_driver/URPositionHardwareInterface' for t in types):
                raise RuntimeError('Real URPositionHardwareInterface not verified')
            if not any(h.name=='OnRobotGripper' and
                       h.class_type=='onrobot_driver/RGHardwareInterface' for h in hw):
                raise RuntimeError('Real OnRobot RG2 not verified')
            if any('mock_components' in t or 'test_hardware_components' in t
                   for t in types):
                raise RuntimeError('Mock hardware present: REJECTED')
            states={c.name:c.state.lower() for c in self.request(
                self.ctrl,ListControllers.Request()).controller}
            if states.get('scaled_joint_trajectory_controller')!='active':
                raise RuntimeError('Scaled arm controller is not active')
            if states.get('joint_trajectory_controller')=='active':
                raise RuntimeError('Another arm position controller is active')
            if states.get('finger_width_trajectory_controller')!='active':
                raise RuntimeError('RG2 controller not active')
            if not self.arm.wait_for_server(timeout_sec=5):
                raise RuntimeError('Physical arm action unavailable')
            if not self.grip.wait_for_server(timeout_sec=5):
                raise RuntimeError('Real gripper action unavailable')

        def cancel_active_goal(self,handle,label,result_future=None):
            # Request cancellation and WAIT for controller acknowledgment.
            # This is not an E-stop and is not proof of a physical standstill.
            print(f'\n{label}: REQUESTING CONTROLLER CANCELLATION. '
                  'Keep pendant stop accessible.', file=sys.stderr, flush=True)
            try:
                cf=handle.cancel_goal_async()
                until=time.monotonic()+6.0
                while not cf.done() and time.monotonic()<until:
                    rclpy.spin_once(self,timeout_sec=.1)
                if not cf.done() or cf.result() is None:
                    print(f'{label}: CANCEL ACK UNKNOWN. Use pendant stop if moving.',
                          file=sys.stderr, flush=True)
                    return False
                if not cf.result().goals_canceling:
                    print(f'{label}: CANCEL NOT ACKNOWLEDGED. Verify stop on pendant.',
                          file=sys.stderr, flush=True)
                    return False
                # Wait briefly for the corresponding terminal result as well.
                if result_future is None:
                    result_future=handle.get_result_async()
                until=time.monotonic()+8.0
                while not result_future.done() and time.monotonic()<until:
                    rclpy.spin_once(self,timeout_sec=.1)
                if not result_future.done() or result_future.result() is None:
                    print(f'{label}: Cancel accepted but terminal result UNKNOWN.',
                          file=sys.stderr, flush=True)
                    return False
                outcome=result_future.result()
                if outcome.status != GoalStatus.STATUS_CANCELED:
                    print(f'{label}: Controller terminal status={outcome.status}; '
                          'verify physical state before any further movement.',
                          file=sys.stderr, flush=True)
                    return False
                print(f'{label}: CONTROLLER CANCEL CONFIRMED (not an E-stop).',
                      file=sys.stderr, flush=True)
                return True
            except BaseException as exc:
                print(f'{label}: CANCEL VERIFICATION FAILED: {exc}. '
                      'Use pendant stop if moving.', file=sys.stderr, flush=True)
                return False

        def action(self,client,trajectory,label,timeout,expected_start=None):
            if self.stop_requested:
                raise RuntimeError(f'{label}: interrupted before command')
            self.hardware()
            fresh_q=self.fresh()  # requires a NEW callback, not cached /joint_states
            if expected_start is not None:
                start_error=err(fresh_q,expected_start)
                print(f'{label}: live start error {start_error:.6f} rad')
                if start_error>START_TOL:
                    raise RuntimeError(f'{label}: live start outside {START_TOL:.3f} rad; '
                                       'no command sent')
                # The first point must be exactly the latest measured joint state.
                # Do NOT translate the remaining learned/CDMP points: that would
                # change the taught assembly geometry. Validate handoff instead.
                if label == 'HOME':
                    # Preserve the successful physical HOME command: one saved
                    # goal at 30s. Its starting state is the current controller
                    # position, independently verified above and in home().
                    if len(trajectory.points) != 1:
                        raise RuntimeError('HOME must contain one target point')
                    t_home = (trajectory.points[0].time_from_start.sec +
                              trajectory.points[0].time_from_start.nanosec * 1e-9)
                    if abs(t_home - 30.0) > 0.01:
                        raise RuntimeError('Unexpected HOME timing')
                    delta = err(fresh_q, trajectory.points[0].positions)
                    if delta / t_home > MAX_SEGMENT_SPEED:
                        raise RuntimeError('HOME average speed guard exceeded')
                    print(f'HOME: verified fresh start; preserving 30s target command')
                else:
                    if (len(trajectory.points) < 2 or
                            trajectory.points[0].time_from_start.sec != 0 or
                            trajectory.points[0].time_from_start.nanosec != 0):
                        raise RuntimeError(f'{label}: invalid initial time')
                    # Keep the rest of the learnt path unchanged. Check the
                    # resulting live->second-point step before sending.
                    trajectory.points[0].positions = list(fresh_q)
                    p1 = trajectory.points[1]
                    dt = p1.time_from_start.sec + p1.time_from_start.nanosec*1e-9
                    if dt <= 0:
                        raise RuntimeError(f'{label}: invalid first segment timing')
                    delta = err(fresh_q, p1.positions)
                    if delta > MAX_JOINT_STEP or delta/dt > MAX_SEGMENT_SPEED:
                        raise RuntimeError(f'{label}: live first segment too large/fast: '
                                           f'{delta:.6f} rad in {dt:.4f}s')
                    print(f'{label}: first trajectory point = FRESH measured joints')
            if self.stop_requested:
                raise RuntimeError(f'{label}: interrupted before send')
            goal=FollowJointTrajectory.Goal(); goal.trajectory=trajectory
            goal.goal_time_tolerance=Duration(seconds=5).to_msg()
            self.stage=label;self.monitor=True;self.goal_inflight=True
            handle=None;done=None
            try:
                future=client.send_goal_async(goal)
                until=time.monotonic()+8
                # If Ctrl+C occurs before acceptance, wait for acceptance so
                # an accepted goal can still be cancelled explicitly.
                while not future.done() and time.monotonic()<until:
                    rclpy.spin_once(self,timeout_sec=.1)
                if not future.done() or future.result() is None:
                    raise RuntimeError(f'{label}: acceptance UNKNOWN; check controller/pendant')
                handle=future.result()
                if not handle.accepted:
                    raise RuntimeError(f'{label}: goal rejected')
                self.active_goal_handle=handle
                done=handle.get_result_async()
                if self.stop_requested:
                    raise RuntimeError(f'{label}: Ctrl+C during goal acceptance')
                until=time.monotonic()+timeout
                while not done.done() and time.monotonic()<until:
                    rclpy.spin_once(self,timeout_sec=.1)
                    if self.stop_requested:
                        raise RuntimeError(f'{label}: Ctrl+C requested')
                    if time.monotonic()-self.last_at>1.0:
                        raise RuntimeError(f'{label}: live-state lost')
                if not done.done() or done.result() is None:
                    raise RuntimeError(f'{label}: controller outcome timed out/unknown')
                r=done.result()
                if (r.status!=GoalStatus.STATUS_SUCCEEDED or
                    r.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    raise RuntimeError(f'{label}: controller failure: '
                                       f'{r.result.error_code} {r.result.error_string}')
                if self.stop_requested:
                    raise RuntimeError(f'{label}: interrupted while completing')
            except BaseException:
                self.stop_requested=True
                if handle is not None and (done is None or not done.done()):
                    self.cancel_active_goal(handle,label,done)
                elif handle is None:
                    print(f'{label}: goal acceptance could not be resolved. '
                          'Verify stopped state on pendant.', file=sys.stderr,flush=True)
                raise
            finally:
                self.monitor=False;self.goal_inflight=False
                self.active_goal_handle=None
            return self.fresh()

        def arm_stage(self,label,plan):
            q0=self.fresh()
            if err(q0,plan['q'][0])>START_TOL:
                raise RuntimeError(f'{label}: current joints differ from START '
                                   f'({err(q0,plan["q"][0]):.5f} rad)')
            # Samples checked against the current MoveIt planning scene.
            # This does NOT cover unmodelled fixtures or the grasped part.
            if not self.state_valid.wait_for_service(timeout_sec=3):
                raise RuntimeError('MoveIt state validity service unavailable')
            for index in sorted(set(round(k*(len(plan['q'])-1)/20) for k in range(21))):
                req=GetStateValidity.Request()
                req.group_name='ur_onrobot_manipulator'
                req.robot_state=RobotState()
                req.robot_state.joint_state.name=JOINTS
                req.robot_state.joint_state.position=plan['q'][index]
                valid=self.request(self.state_valid,req,timeout=5)
                if not valid.valid:
                    raise RuntimeError(f'{label}: MoveIt state invalid at CSV row {index}')
            traj=JointTrajectory();traj.joint_names=JOINTS
            for t,q in zip(plan['t'],plan['q']):
                p=JointTrajectoryPoint();p.positions=q
                p.time_from_start=Duration(seconds=t).to_msg();traj.points.append(p)
            self.stage=label
            end=self.action(self.arm,traj,label,plan['t'][-1]*5+30,expected_start=plan['q'][0])
            if err(end,plan['q'][-1])>END_TOL:
                raise RuntimeError(f'{label}: endpoint mismatch '
                                   f'({err(end,plan["q"][-1]):.6f} rad)')
            print(f'{label}: PASS | final joint error {err(end,plan["q"][-1]):.6f} rad')

        def rg2(self,label,width_m,duration_s):
            self.fresh()
            if (self.width is None or not math.isfinite(self.width)
                    or time.monotonic()-self.width_at>1.0):
                raise RuntimeError('Missing fresh RG2 finger_width state')
            if not 0<=self.width<=0.12:
                raise RuntimeError('Unexpected RG2 width units')
            traj=JointTrajectory();traj.joint_names=['finger_width']
            point=JointTrajectoryPoint();point.positions=[width_m]
            point.time_from_start=Duration(seconds=duration_s).to_msg()
            traj.points=[point]
            self.action(self.grip,traj,label,30)
            # GRIP can stop on contact instead of reaching exact requested 8.2 mm.
            # This verifies motion only; it is NOT a force/part-presence check.
            after=time.monotonic()
            deadline=after+3.0
            while time.monotonic()<deadline and self.width_at<after:
                rclpy.spin_once(self,timeout_sec=.1)
            measured=self.width
            tol=.005 if label=='GRIP' else .003
            if self.width_at<after or measured is None or abs(measured-width_m)>tol:
                raise RuntimeError(f'{label}: RG2 width {measured} m differs '
                                   f'from {width_m} m or state is stale; inspect gripper physically')
            print(f'{label}: PASS | commanded {width_m*1000:.1f} mm '
                  f'over {duration_s:.1f}s; measured {measured*1000:.2f} mm '
                  '(contact and grasp security NOT proven by width)')

        def home_state_validity(self,start,goal):
            # Interpolation validates nominal robot self/scene collision states.
            # It cannot verify real fixture/part geometry missing from MoveIt.
            if not self.state_valid.wait_for_service(timeout_sec=5):
                raise RuntimeError('MoveIt /check_state_validity unavailable; '
                                   'HOME must not be sent blind')
            n=61
            for k in range(n):
                alpha=k/(n-1)
                vals=[a+(b-a)*alpha for a,b in zip(start,goal)]
                request=GetStateValidity.Request();request.group_name='ur_onrobot_manipulator'
                request.robot_state=RobotState()
                request.robot_state.joint_state.name=JOINTS
                request.robot_state.joint_state.position=vals
                valid=self.request(self.state_valid,request,timeout=5)
                if not valid.valid:
                    raise RuntimeError(f'HOME collision/state check FAILED at {k}/{n-1}')
            print('HOME: 61 MoveIt robot-state samples valid; missing scene geometry '
                  'still requires operator inspection')

        def home(self,goal,plan_only=False):
            start=self.fresh()
            # Require the specific retreated V1 endpoint corridor, rather
            # than allowing arbitrary far-away poses as HOME starts.
            expected=[-1.236968,-1.640721,-1.571950,-1.492302,
                      1.545205,-3.969157]
            if err(start,expected)>.035:
                raise RuntimeError('HOME blocked: robot not at verified '
                                   f'V1 retreat endpoint, error={err(start,expected):.4f}')
            if (self.width is None or self.width<.016 or
                    time.monotonic()-self.width_at > 2.0):
                raise RuntimeError('HOME blocked: fresh RG2 open state not verified')
            self.home_state_validity(start,goal)
            print('HOME: long joint-space path from retreated V1; fixture must be clear.')
            if plan_only:
                print('HOME-ONLY PREFLIGHT: PASS. NO physical motion commanded.')
                return
            approve('FINAL_HOME', 'Confirm entire joint-space route is clear of trolley, fixtures and part.')
            latest=self.fresh()
            if err(latest,start)>.002:
                raise RuntimeError('Robot moved during HOME review; stop and replan')
            traj=JointTrajectory();traj.joint_names=JOINTS
            point=JointTrajectoryPoint();point.positions=goal
            point.time_from_start=Duration(seconds=30).to_msg()
            traj.points=[point]
            end=self.action(self.arm,traj,'HOME',200,expected_start=start)
            if err(end,goal)>.015:
                raise RuntimeError(f'HOME endpoint error {err(end,goal):.6f}')
            print(f'HOME: PASS | final joint error {err(end,goal):.6f} rad')

    def approve(label,detail):
        if auto and label != 'V1_START':
            if node.stop_requested:
                raise RuntimeError(f'{label}: STOP already requested; no next stage')
            print(f'\nAUTO {label}: {detail}')
            print('Automatic supervised transition; existing live-state/controller/scene checks remain active.')
            return
        print('\n--- PHYSICAL STAGE REVIEW:',label,'---')
        print(detail)
        print('Confirm real fixture/workpiece placement, motion clearance, '
              'one command source, stop button accessible.')
        print('Do not confirm if person/obstacle is in robot workspace.')
        if auto and label == 'V1_START':
            print('SINGLE APPROVAL: all 12 stages will continue without further typing.')
            print('Supervise the robot throughout; any safety/trajectory/state failure must stop the sequence.')
            print('ONLY use after separate physical verification of RG2 OPEN/RELEASE 40 mm and GRIP 8.2 mm/2 s.')
            phrase = 'RUN_SUPERVISED_V1_AUTO'
        else:
            phrase = f'RUN_{label}'
        if node.stop_requested or input(f'Type {phrase} to continue: ').strip()!=phrase:
            raise RuntimeError(f'{label}: operator cancelled; NO further stages sent')

    logdir=(ROOT/'results/physical'/('v1_home_only' if home_only else ('v1_pickup_alignment' if pickup_only else 'v1_integrated'))/
            datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    logdir.mkdir(parents=True,exist_ok=False)
    import rclpy
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Robot()
    active_child=[None]
    def handle_sigint(_signum,_frame):
        child=active_child[0]
        if child is not None and child.poll() is None:
            print('\nCtrl+C: forwarding SIGINT to RETREAT child; '
                  'verify controller cancellation and physical stop.',
                  file=sys.stderr,flush=True)
            try:
                os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:
                pass
            node.stop_requested=True
        elif node.goal_inflight:
            node.stop_requested=True
            print('\nCtrl+C: cancellation requested; waiting for controller result. '
                  'Use pendant stop if motion persists.',file=sys.stderr,flush=True)
        else:
            raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,handle_sigint)
    status={'status':'IN_PROGRESS','stages':[], 'log_folder':str(logdir),
            'rg2_targets':{'open_mm':40.0,'release_mm':40.0,'grip_mm':8.2,
                           'open_duration_s':3.0,'grip_duration_s':2.0},
            'trial_type':('PICKUP_ALIGNMENT_DIAGNOSTIC' if pickup_only else 'PILOT_NOT_FORMAL_L0'), 'auto_mode': bool(auto), 'started_at':datetime.now().isoformat(),
            'note':'wall_elapsed_s includes operator confirmations; not pure robot cycle time'}
    cycle_start=time.monotonic()
    all_records=[]
    try:
        node.hardware()
        if home_only:
            print('HOME ONLY: skipping all completed V1 stages; verify fixture clearance.')
            node.home(home,plan_only=home_only_plan)
            status['stages'].append({'name':'FINAL_HOME',
                                      'status':'PLAN_ONLY_PASS' if home_only_plan else 'PASS'})
            status['status']='PLAN_ONLY_PASS' if home_only_plan else 'PASS'
            return
        start=node.fresh()
        if err(start,home)>.015:
            raise RuntimeError(f'Physical cycle must start at saved HOME; error={err(start,home):.5f}')
        print('\n=== OPERATOR CHECK: physical V1 PICKUP-ONLY test ===' if pickup_only else '\n=== OPERATOR CHECK: physical V1 automatic sequence ===')
        print('All stage trajectories MUST be verified against individually tested physical files.')
        print('CSV timing is multiplied by 2; confirm this was appropriate for every stage.')
        print('NEW GRIPPER SETTINGS: OPEN/RELEASE 40 mm, GRIP 8.2 mm / 2 s.')
        print('Verify 40 mm RELEASE cannot collide with or disturb the assembled component.')
        print('Robot must be at saved HOME. V1 component seated at correct pickup.')
        if pickup_only:
            print('PICKUP ONLY: OPEN -> REACH -> ALIGN_PICKUP, then STOP. NO GRIP/TRANSPORT/INSERT/HOME.')
            print('Use only AFTER qualified E-stop recovery and safe, verified return to HOME.')
        print('Fixture/trolley/component may NOT be represented accurately in MoveIt.')
        print('MODE: single-approval SUPERVISED automatic pilot.' if auto else
              'MODE: stage-by-stage confirmations. No unattended run.')
        print('Verify physical fixture clearance for EVERY arm movement; no complete collision-scene validation.')
        approve('PICKUP_START' if pickup_only else 'V1_START',
                'Move ONLY to V1 pickup alignment from verified HOME; no grip or insertion.'
                if pickup_only else 'Start guarded V1 sequence, from HOME.')
        stages = [
            ('OPEN',lambda:node.rg2('OPEN',RG2_OPEN_M,RG2_OPEN_DURATION_S),'Check RG2 open to 40mm'),
            ('REACH',lambda:node.arm_stage('REACH',plans['REACH']),'Move toward V1 pickup area'),
            ('ALIGN_PICKUP',lambda:node.arm_stage('ALIGN_PICKUP',plans['ALIGN_PICKUP']),'Fine alignment to V1 grip'),
            ('GRIP',lambda:node.rg2('GRIP',RG2_GRIP_M,RG2_GRIP_DURATION_S),'Grip V1 at 8.2mm over 2s; verify workpiece is present'),
            ('TRANSPORT',lambda:node.arm_stage('TRANSPORT',plans['TRANSPORT']),'Carry V1 to assembly fixture'),
            ('ALIGN_INSERT',lambda:node.arm_stage('ALIGN_INSERT',plans['ALIGN_INSERT']),'Align above insertion'),
            ('INSERT',lambda:node.arm_stage('INSERT',plans['INSERT']),'Insert with physically validated trajectory'),
            ('ROTATE',lambda:node.arm_stage('ROTATE',plans['ROTATE']),'Rotate using physically validated 25-degree motion'),
            ('RELEASE',lambda:node.rg2('RELEASE',RG2_OPEN_M,RG2_OPEN_DURATION_S),'Open RG2 to 40mm and release assembled V1; verify part/fixture clearance'),
        ]
        if pickup_only:
            stages=stages[:3]  # OPEN, REACH, ALIGN_PICKUP only
        for name,action,detail in stages:
            approve(name,detail)
            node.hardware()
            begin=time.monotonic()
            action()
            status['stages'].append({'name':name,'status':'PASS',
                                     'elapsed_s':round(time.monotonic()-begin,3)})
            (logdir/'status.json').write_text(json.dumps(status,indent=2))
            if node.stop_requested:
                raise RuntimeError('Interrupted; refusing subsequent stages')
        if pickup_only:
            status['status']='PICKUP_POSITION_REACHED'
            status['wall_elapsed_s']=round(time.monotonic()-cycle_start,3)
            print('\nPICKUP ALIGNMENT POSITION REACHED. No GRIP, TRANSPORT, INSERT, ROTATE, RETREAT or HOME issued.')
            print('Robot is intentionally left stationary at V1 pickup alignment.')
            print('Inspect jaw-to-part alignment and physical clearance without entering unsafe workspace.')
            print('A separately validated plan is required before leaving this position.')
            return
        print('\nRETREAT uses the successful independent guarded script, '
              'including new Cartesian planning, branch correction, and its own operator review.')
        approve('RETREAT','Confirm released workpiece; +30mm base-Z withdrawal is clear.')
        all_records.extend(node.records)
        node.destroy_node()
        rclpy.shutdown()
        # Separate already-validated retreat implementation; preserve its
        # independent hardware, pose, collision/path and execution guards.
        # Retreat has its own independent controller/pose/collision guards.
        # Isolate the child's process group so Ctrl+C is deliberately relayed.
        retreat_begin=time.monotonic()
        retreat_args=[sys.executable,str(RETREAT),'--execute']
        retreat_env=os.environ.copy()
        if auto:
            # Delegates ONLY the earlier explicit whole-cycle consent.
            # Child still independently verifies current pose, Cartesian path,
            # hardware, active controller, joint branch, final joint/TCP position.
            retreat_args.append('--approved-by-parent')
            retreat_env['V1_AUTO_PARENT_PID']=str(os.getpid())
            retreat_env['V1_AUTO_SESSION']=secrets.token_hex(24)
        child=subprocess.Popen(retreat_args,env=retreat_env,
                               start_new_session=True)
        active_child[0]=child
        try:
            code=child.wait()
        finally:
            active_child[0]=None
        if node.stop_requested or code != 0:
            raise RuntimeError('RETREAT interrupted/failed; HOME NOT SENT')
        status['stages'].append({'name':'RETREAT','status':'PASS',
                                 'elapsed_s_including_retreat_review':round(time.monotonic()-retreat_begin,3)})
        (logdir/'status.json').write_text(json.dumps(status,indent=2))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Robot()
        node.hardware()
        home_begin=time.monotonic()
        node.home(home)
        status['stages'].append({'name':'HOME','status':'PASS',
                                 'elapsed_s_including_home_review':round(time.monotonic()-home_begin,3)})
        status['status']='PASS'
        status['wall_elapsed_s']=round(time.monotonic()-cycle_start,3)
        print('\nV1 COMPLETE: all 12 stages completed. Independently inspect assembly result.')
    except BaseException as e:
        status['status']='STOPPED'
        status['error']=str(e)
        status['wall_elapsed_s']=round(time.monotonic()-cycle_start,3)
        print('V1 STOPPED:',e,file=sys.stderr)
        print('Do not blindly restart after a partial assembly; identify robot state.',file=sys.stderr)
        raise
    finally:
        (logdir/'status.json').write_text(json.dumps(status,indent=2))
        with (logdir/'joint_states.csv').open('w',newline='') as f:
            writer=csv.writer(f)
            writer.writerow(['timestamp','stage']+JOINTS+['finger_width_m'])
            writer.writerows(all_records + node.records)
        try:node.destroy_node()
        except Exception:pass
        try:
            if rclpy.ok():rclpy.shutdown()
        except Exception:pass
        print('Sequence logs:',logdir)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',
                    help='Physical robot: guarded staged run with confirmations')
    ap.add_argument('--auto',action='store_true',
                    help='One whole-cycle approval for SUPERVISED physical pilot (requires --execute)')
    ap.add_argument('--reach-timing-preview',type=float,default=2.0,metavar='FACTOR',
                    help='Offline-only: compare timing factor (e.g. 1.5) with proven x2; never alters physical commands')
    ap.add_argument('--pickup-only',action='store_true',
                    help='OPEN40 -> REACH -> ALIGN_PICKUP from verified HOME; stop BEFORE grip; separate staged physical execution only')
    ap.add_argument('--home-only',action='store_true',
                    help='Resume FINAL HOME only from verified V1 retreat endpoint; never replay assembly')
    args=ap.parse_args()
    if not 1.0 <= args.reach_timing_preview <= 3.0 or not math.isfinite(args.reach_timing_preview):
        ap.error('--reach-timing-preview must be finite and within 1.0..3.0')
    if args.execute and args.reach_timing_preview != 2.0:
        ap.error('Physical execution only uses proven x2 timing; timing-preview is OFFLINE ONLY')
    if args.execute and not args.pickup_only:
        ap.error('This diagnostic copy permits physical execution ONLY with --pickup-only')
    if args.pickup_only and (args.auto or args.home_only):
        ap.error('--pickup-only cannot be combined with --auto or --home-only')
    if args.auto and (not args.execute or args.home_only):
        ap.error('--auto requires --execute and cannot be combined with --home-only')
    if args.home_only:
        home=load_home()
        if args.execute:
            print('HOME-ONLY EXECUTION REQUESTED. Requires explicit FINAL_HOME confirmation.')
        else:
            print('HOME-ONLY PLAN-ONLY PREFLIGHT. No physical movement will be sent.')
        run_actual(home,{},home_only=True,home_only_plan=not args.execute)
        return
    home,plans=preflight()
    if args.reach_timing_preview != 2.0:
        reach=plans['REACH']
        duration=reach['t'][-1]*args.reach_timing_preview/2.0
        max_speed=reach['max_speed']*2.0/args.reach_timing_preview
        print(f'REACH OFFLINE PREVIEW: factor x{args.reach_timing_preview:.2f}, duration {duration:.2f} s, '
              f'estimated max joint speed {max_speed:.4f} rad/s. NO ROBOT MOVEMENT; NOT PHYSICALLY VALIDATED.')
    if not args.execute:
        if args.pickup_only:
            print('PICKUP-ONLY PREFLIGHT: PASS. No robot movement. Real recovery/clearance not yet verified.')
        return
    if args.pickup_only:
        print('PICKUP-ONLY EXECUTION REQUESTED. No automatic cycle, no GRIP, no HOME.')
        run_actual(home,plans,pickup_only=True)
        return
    if args.auto:
        confirm_updated_gripper_pilot()
    print('EXECUTE requested. Real robot will move if all guards and confirmations pass.')
    run_actual(home,plans,auto=args.auto)


if __name__=='__main__':
    try:main()
    except (RuntimeError,ValueError,OSError) as e:
        sys.exit('BLOCKED: '+str(e))
