#!/usr/bin/env python3
import csv, math, os, time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '73':
    raise SystemExit('BLOCKED: Require ROS_DOMAIN_ID=73')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

base = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
folder = base / 'demonstrations/processed/phase4_assembly'
original = folder / 'v1_insert_joint_sim_candidate.csv'
retimed = folder / 'v1_insert_joint_sim_13s_CANDIDATE.csv'
log = base / 'results/simulation/v1_insert_sim_actual_001.csv'

joints = [
    s + '_joint' for s in
    ('shoulder_pan', 'shoulder_lift', 'elbow',
     'wrist_1', 'wrist_2', 'wrist_3')
]

def read(path):
    with path.open(newline='') as f:
        d = csv.DictReader(f)
        if d.fieldnames != ['time'] + joints:
            raise RuntimeError('Unexpected columns: ' + str(path))
        rows = list(d)

    return (
        [float(r['time']) for r in rows],
        [[float(r[j]) for j in joints] for r in rows]
    )

class Executor(Node):
    def __init__(self):
        super().__init__('mock_only_v1_insert')

        self.q = None
        self.record = False
        self.samples = []
        self.t0 = 0

        self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint,
            qos_profile_sensor_data
        )

        self.hardware = self.create_client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components'
        )

        self.controllers = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        self.action = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def on_joint(self, msg):
        if not all(j in msg.name for j in joints):
            return

        self.q = [
            float(msg.position[msg.name.index(j)])
            for j in joints
        ]

        if self.record:
            self.samples.append(
                [time.monotonic() - self.t0] + self.q
            )

    def call(self, client, service):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Required ROS service unavailable')

        fut = client.call_async(service.Request())

        rclpy.spin_until_future_complete(
            self, fut, timeout_sec=5
        )

        if not fut.done() or fut.result() is None:
            raise RuntimeError('ROS service request failed')

        return fut.result()

    def check_fake(self):
        hardware = self.call(
            self.hardware, ListHardwareComponents
        ).component

        if not hardware:
            raise RuntimeError('No hardware components')

        expected_arm_interfaces = {
            j + '/position' for j in joints
        }

        fake_arms = []
        fake_grippers = []

        for h in hardware:
            print('Hardware:', h.name, h.class_type)

            interfaces = {
                i.name for i in h.command_interfaces
            }

            if h.class_type == 'mock_components/GenericSystem':
                if not expected_arm_interfaces.issubset(interfaces):
                    raise RuntimeError(
                        'BLOCKED: Fake arm joint interfaces not verified'
                    )
                fake_arms.append(h)

            elif (
                h.name == 'OnRobotGripper'
                and h.class_type ==
                'test_hardware_components/TestSingleJointActuator'
            ):
                fake_grippers.append(h)

            else:
                raise RuntimeError(
                    'BLOCKED: Unexpected hardware component: '
                    + h.name + ' (' + h.class_type + ')'
                )

        if (
            len(hardware) != 2
            or len(fake_arms) != 1
            or len(fake_grippers) != 1
        ):
            raise RuntimeError(
                'BLOCKED: Expected exactly one fake UR3e arm '
                'and one test OnRobot gripper'
            )

        print('FAKE ARM AND TEST GRIPPER: VERIFIED')

        controllers = self.call(
            self.controllers, ListControllers
        ).controller

        states = {
            c.name: c.state.lower()
            for c in controllers
        }

        if states.get('joint_trajectory_controller') != 'active':
            raise RuntimeError(
                'Fake arm controller is not active'
            )

        if states.get('scaled_joint_trajectory_controller') == 'active':
            raise RuntimeError(
                'BLOCKED: Scaled controller is active'
            )

        if states.get('finger_width_trajectory_controller') != 'active':
            raise RuntimeError(
                'Gripper controller is not active'
            )

def main():
    if log.exists():
        raise RuntimeError('Existing log: ' + str(log))

    ts, qs = read(retimed)
    _, old = read(original)

    if len(qs) != 9 or len(old) != 9:
        raise RuntimeError('Unexpected joint point count')

    if not all(math.isfinite(v) for r in qs for v in r):
        raise RuntimeError('Invalid joint values')

    if any(
        abs(a - b) > 1e-9
        for r, s in zip(qs, old)
        for a, b in zip(r, s)
    ):
        raise RuntimeError(
            'Retiming changed joint positions'
        )

    if (
        not all(math.isfinite(t) for t in ts)
        or abs(ts[0]) > 1e-6
        or abs(ts[-1] - 12.999) > 0.01
        or any(b <= a for a, b in zip(ts, ts[1:]))
    ):
        raise RuntimeError('Invalid trajectory timing')

    if qs[0][-1] > -3:
        raise RuntimeError('Wrist 3 branch mismatch')

    step = max(
        abs(b[k] - a[k])
        for a, b in zip(qs, qs[1:])
        for k in range(6)
    )

    speed = max(
        abs(qs[i+1][k] - qs[i][k]) / (ts[i+1] - ts[i])
        for i in range(8)
        for k in range(6)
    )

    if step > 0.05 or speed > 0.10:
        raise RuntimeError(
            'Joint step or segment velocity limit exceeded'
        )

    rclpy.init()
    node = Executor()

    try:
        node.check_fake()

        deadline = time.monotonic() + 5

        while node.q is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        if node.q is None:
            raise RuntimeError('No live joint states')

        start_error = max(
            abs(a - b)
            for a, b in zip(node.q, qs[0])
        )

        print(f'Live start error: {start_error:.6f} rad')

        if start_error > 0.002:
            raise RuntimeError(
                'Robot is not at INSERT start'
            )

        if not node.action.wait_for_server(timeout_sec=5):
            raise RuntimeError(
                'Joint trajectory action unavailable'
            )

        print('\n========== V1 INSERT PRECHECK ==========')
        print('Mock hardware: VERIFIED')
        print('Fake joint controller: ACTIVE')
        print(f'Joint points: {len(qs)}')
        print(f'Duration: {ts[-1]:.3f} s')
        print(f'Maximum joint step: {step:.6f} rad')
        print(f'Max segment speed: {speed:.5f} rad/s')
        print('No automatic home or repositioning.')
        print('Confirm fake gripper remains closed to 9 mm.')

        if input('\nType INSERT to simulate: ').strip() != 'INSERT':
            print('Cancelled. No movement.')
            return

        # Recheck hardware immediately before sending.
        node.check_fake()
        rclpy.spin_once(node, timeout_sec=0.2)

        if node.q is None or max(
            abs(a-b) for a,b in zip(node.q, qs[0])
        ) > 0.002:
            raise RuntimeError(
                'Starting joint state changed'
            )

        traj = JointTrajectory()
        traj.joint_names = joints

        for t, q in zip(ts, qs):
            p = JointTrajectoryPoint()
            p.positions = q
            p.time_from_start = Duration(
                seconds=t
            ).to_msg()
            traj.points.append(p)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(
            seconds=2
        ).to_msg()

        node.t0 = time.monotonic()
        node.record = True

        sent = node.action.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            node, sent, timeout_sec=5
        )

        if (
            not sent.done()
            or sent.result() is None
            or not sent.result().accepted
        ):
            raise RuntimeError('Trajectory goal rejected')

        result = sent.result().get_result_async()

        rclpy.spin_until_future_complete(
            node, result, timeout_sec=22
        )

        node.record = False

        if not result.done() or result.result() is None:
            raise RuntimeError(
                'Execution timed out; inspect controller before retrying'
            )

        reply = result.result()

        if (
            reply.status != GoalStatus.STATUS_SUCCEEDED
            or reply.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                'Controller reported execution failure'
            )

        error = max(
            abs(a-b)
            for a,b in zip(node.q, qs[-1])
        )

        if error > 0.002:
            raise RuntimeError(
                f'Endpoint joint error: {error:.6f} rad'
            )

        log.parent.mkdir(
            parents=True, exist_ok=True
        )

        with log.open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['elapsed_s'] + joints)
            writer.writerows(node.samples)

        print('\n========== V1 INSERT SIMULATION ==========')
        print('Result: PASS')
        print(f'Final joint error: {error:.6f} rad')
        print(f'Joint-state samples: {len(node.samples)}')
        print(f'Saved: {log}')
        print('Fake robot is now at INSERT endpoint.')
        print('Do not send home before ROTATE.')

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError) as exc:
        raise SystemExit('BLOCKED / FAILED: ' + str(exc))
