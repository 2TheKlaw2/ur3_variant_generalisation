#!/usr/bin/env python3
"""Supervised reverse of the saved V1 INSERT joint path; no automatic retry."""
import csv, math, time
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from action_msgs.msg import GoalStatus

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
FILE = ROOT / 'demonstrations/processed/phase4_assembly/v1_insert_joint_sim_13s_CANDIDATE.csv'
JOINTS = [x + '_joint' for x in ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')]
QUIET_FORCE = [-1.356, 2.560, -5.993]  # from immediately before the failed insertion
SCALE = 2.0

with FILE.open(newline='') as f:
    data = list(csv.DictReader(f))
assert len(data) == 9, 'Unexpected insertion path length'
T = [float(row['time']) for row in data]
Q = [[float(row[j]) for j in JOINTS] for row in data]
assert all(math.isfinite(x) for q in Q for x in q)
assert all(b>a for a,b in zip(T,T[1:]))

rclpy.init()
node = rclpy.create_node('v1_insert_retract_physical')
state = {'joint':None,'force':None,'when':0.0}
records = []
recording = {'active':False,'start':0.0}

def on_joint(msg):
    j = dict(zip(msg.name,msg.position))
    state['joint'] = j
    if recording['active'] and all(name in j for name in JOINTS):
        w = state['force'] or [float('nan')]*6
        records.append([time.monotonic()-recording['start'],j.get('finger_width',float('nan'))] +
                       [j[name] for name in JOINTS] + list(w))

def on_wrench(msg):
    w=msg.wrench
    state['force']=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
    state['when']=time.monotonic()

node.create_subscription(JointState,'/joint_states',on_joint,qos_profile_sensor_data)
node.create_subscription(WrenchStamped,'/force_torque_sensor_broadcaster/wrench',on_wrench,qos_profile_sensor_data)

def refresh():
    state['joint']=None
    state['force']=None
    limit=time.monotonic()+7
    while (state['joint'] is None or state['force'] is None) and time.monotonic()<limit:
        rclpy.spin_once(node,timeout_sec=0.15)
    if state['joint'] is None or state['force'] is None:
        raise RuntimeError('Live joint or F/T feedback missing')
    return state['joint'],list(state['force'])

service=node.create_client(ListControllers,'/controller_manager/list_controllers')
assert service.wait_for_service(timeout_sec=6),'Controller manager unavailable'
fut=service.call_async(ListControllers.Request())
rclpy.spin_until_future_complete(node,fut,timeout_sec=8)
assert fut.result() is not None,'No controller status'
active={c.name for c in fut.result().controller if c.state=='active'}
assert {'joint_state_broadcaster','scaled_joint_trajectory_controller',
        'finger_width_trajectory_controller'} <= active,'Required physical controllers inactive'

j,w=refresh()
assert all(name in j for name in JOINTS),'Missing arm positions'
x=[j[name] for name in JOINTS]
width=j.get('finger_width',float('nan'))
assert math.isfinite(width) and 0.008 <= width <= 0.016,'Check gripper holding V1'
assert max(abs(w[k]-QUIET_FORCE[k]) for k in range(3))<5.0, \
    'F/T differs significantly from pre-insertion resting load: inspect jam/contact'

# Find the nearest point on the SAVED insertion trajectory, including between waypoints.
best=None
for k in range(len(Q)-1):
    a,b=Q[k],Q[k+1]
    d=[b[n]-a[n] for n in range(6)]
    denom=sum(z*z for z in d)
    fraction=max(0.0,min(1.0,sum((x[n]-a[n])*d[n] for n in range(6))/denom))
    candidate=[a[n]+fraction*d[n] for n in range(6)]
    gap=max(abs(x[n]-candidate[n]) for n in range(6))
    if best is None or gap<best[0]:
        best=(gap,k,fraction)

gap,k,fraction=best
elapsed=T[k]+fraction*(T[k+1]-T[k])
print('\n===== V1 INSERT RETRACT PREFLIGHT =====')
print(f'Current gripper: {width*1000:.2f} mm')
print('Current F/T:',[round(z,3) for z in w])
print(f'Saved insertion-path progress: {elapsed:.3f} / {T[-1]:.3f} s')
print(f'Off-path joint error: {gap:.6f} rad')
assert gap<0.002,'Current robot pose differs from saved INSERT path: do not move'
assert 0.5 <= elapsed <= T[-1]+0.1,'Unexpected insertion stage progress'

# Start from the ACTUAL joint positions; go backward through the prior saved waypoints.
points=[(0.30,x)]
for idx in range(k,-1,-1):
    points.append((0.30 + (elapsed-T[idx])*SCALE,Q[idx]))
assert all(b[0]>a[0] for a,b in zip(points,points[1:]))
step=max(max(abs(u-v) for u,v in zip(a[1],b[1])) for a,b in zip(points,points[1:]))
speed=max(max(abs(u-v) for u,v in zip(a[1],b[1]))/(b[0]-a[0]) for a,b in zip(points,points[1:]))
print(f'Retraction: {len(points)} points, {points[-1][0]:.2f} commanded seconds')
print(f'Max joint step {step:.5f} rad; max average joint speed {speed:.5f} rad/s')
assert step<0.01 and speed<0.015,'Retraction trajectory needs review'

traj=JointTrajectory(joint_names=JOINTS)
for secs,q in points:
    p=JointTrajectoryPoint()
    p.positions=q
    p.velocities=[0.0]*6
    ns=round(secs*1e9)
    s,n=divmod(ns,1_000_000_000)
    p.time_from_start=Duration(sec=s,nanosec=n)
    traj.points.append(p)

client=ActionClient(node,FollowJointTrajectory,
    '/scaled_joint_trajectory_controller/follow_joint_trajectory')
assert client.wait_for_server(timeout_sec=8),'Physical arm controller action unavailable'
print('\nTHIS WILL PHYSICALLY RETRACT V1 FROM PARTIAL INSERTION TO INSERT ALIGN.')
print('No environmental collision checking. No subsequent INSERT, ROTATE or RELEASE.')
print('Verify obstruction removed, V1 not mechanically jammed, reverse path clear.')
print('Operator must supervise, outside robot envelope, with pendant STOP available.')
if input('Type RETRACT to execute (anything else cancels): ').strip()!='RETRACT':
    raise SystemExit('CANCELLED; no robot movement')

j,w=refresh()
assert max(abs(j[name]-x[n]) for n,name in enumerate(JOINTS))<0.0015, \
    'Joint positions changed since preflight'
assert abs(j['finger_width']-width)<0.002,'Gripper opening changed'
assert max(abs(w[n]-QUIET_FORCE[n]) for n in range(3))<5.0, \
    'Unexpected F/T contact load; do not retract'
base=list(w)

goal=FollowJointTrajectory.Goal()
goal.trajectory=traj
goal.goal_time_tolerance=Duration(sec=40)
sent=client.send_goal_async(goal)
rclpy.spin_until_future_complete(node,sent,timeout_sec=10)
assert sent.result() is not None and sent.result().accepted,'Goal rejected'
handle=sent.result()
reply=handle.get_result_async()
recording['start']=time.monotonic()
recording['active']=True
aborted=None
try:
    while not reply.done():
        rclpy.spin_once(node,timeout_sec=0.10)
        wr=state['force']
        if wr is None or time.monotonic()-state['when']>1.0:
            aborted='Lost force/torque feedback'
        elif max(abs(wr[i]-base[i]) for i in range(3))>15.0:
            aborted='Unexpected >15 N force change during retraction'
        elif max(abs(wr[i]-base[i]) for i in range(3,6))>1.5:
            aborted='Unexpected >1.5 Nm torque change during retraction'
        jnow=state['joint'] or {}
        nowwidth=jnow.get('finger_width',float('nan'))
        if not math.isfinite(nowwidth) or abs(nowwidth-width)>0.003:
            aborted='Gripper opening changed unexpectedly'
        if aborted:
            print('ABORT:',aborted,'— use pendant Stop if movement continues')
            cancel=handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
            break
except KeyboardInterrupt:
    aborted='Operator cancelled'
    print('Use pendant Stop if robot is still moving.')
    cancel=handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
finally:
    recording['active']=False
    folder=ROOT/'results/physical/v1_remaining'
    folder.mkdir(parents=True,exist_ok=True)
    logfile=folder/f'insert_retract_{datetime.now():%Y%m%d_%H%M%S}.csv'
    with logfile.open('w',newline='') as f:
        wr=csv.writer(f)
        wr.writerow(['elapsed_s','finger_width_m']+JOINTS+
                    ['Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm'])
        wr.writerows(records)
    print('Retraction log:',logfile)

if aborted:
    raise SystemExit('STOP: '+aborted+'. Do NOT retry INSERT.')
assert reply.done() and reply.result() is not None,'Goal outcome unknown'
res=reply.result()
print('Action status:',res.status,'error code:',res.result.error_code)
if res.status!=GoalStatus.STATUS_SUCCEEDED or res.result.error_code!=0:
    raise SystemExit('RETRACT FAILED; do not INSERT')
j,_=refresh()
final=max(abs(j[name]-Q[0][i]) for i,name in enumerate(JOINTS))
print(f'Insert ALIGN endpoint error: {final:.6f} rad')
if final>0.005:
    raise SystemExit('RETRACT ENDPOINT NOT VERIFIED; do not INSERT')
print('RETRACT TO INSERT ALIGN: PASS')
print('Inspect fixture and confirm the cause of the previous force spike before retrying.')
node.destroy_node()
rclpy.shutdown()
