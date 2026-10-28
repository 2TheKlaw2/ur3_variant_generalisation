#!/usr/bin/env python3
"""Offline, read-only diagnostics for the V1 transport joint candidate.

Checks planned path start/end against the real transport demo, grip handoff,
branch continuity, time ordering and conservative fake-replay dynamic limits.
This does NOT approve execution or establish collision clearances.
"""
import argparse
import csv
import math
from pathlib import Path
import yaml

JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
SHORT = [j.removesuffix('_joint') for j in JOINTS]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path,
                   default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    p.add_argument('--slowdown', type=float, default=2)
    a = p.parse_args()
    if not 1 <= a.slowdown <= 5:
        p.error('slowdown must be between 1 and 5')
    root = a.project.expanduser().resolve()
    joint_file = root/'demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv'
    demo_file = root/'demonstrations/processed/v1_transport_001_motion.csv'
    grip_file = root/'config/saved_poses/v1_grip_pose.yaml'
    if any(not path.is_file() for path in (joint_file, demo_file, grip_file)):
        raise SystemExit('Missing candidate, cleaned demo or grip pose')

    with joint_file.open(newline='') as f:
        r = csv.DictReader(f)
        if r.fieldnames != ['time'] + JOINTS:
            raise SystemExit('FAIL: Candidate CSV headers do not match expected six-joint plan')
        rows = list(r)
    with demo_file.open(newline='') as f:
        demo = list(csv.DictReader(f))
    saved = yaml.safe_load(grip_file.read_text())
    if not isinstance(saved, dict) or 'arm' not in saved:
        raise SystemExit('FAIL: Unrecognised grip pose format')
    if saved['arm'].get('joint_names') != JOINTS:
        raise SystemExit('FAIL: Saved grip pose joint order mismatch')
    if len(rows) < 200 or len(demo) < 100:
        raise SystemExit('FAIL: Suspiciously short trajectory or demo')

    times = [float(r['time']) for r in rows]
    qs = [[float(r[j]) for j in JOINTS] for r in rows]
    if not all(math.isfinite(x) for x in times + [v for row in qs for v in row]):
        raise SystemExit('FAIL: Non-finite trajectory data')
    dt = [(t_next-t_prev)*a.slowdown for t_prev, t_next in zip(times, times[1:])]
    if times[0] < 0 or any(t <= 0.0001 for t in dt):
        raise SystemExit('FAIL: Non-increasing or too-short trajectory time steps')
    if not 1 < times[-1]-times[0] < 180:
        raise SystemExit('FAIL: Unexpected trajectory duration')
    if any(abs(x) > 2*math.pi + 0.1 for row in qs for x in row):
        raise SystemExit('FAIL: Joint outside review angle limits')

    demo_start = [float(demo[0][j]) for j in SHORT]
    demo_end = [float(demo[-1][j]) for j in SHORT]
    grip = [float(x) for x in saved['arm']['positions_rad']]

    def err(u, v):
        return max(abs(x-y) for x, y in zip(u, v))

    demo_start_err = err(qs[0], demo_start)
    grip_err = err(qs[0], grip)
    end_err = err(qs[-1], demo_end)
    max_step = max(abs(b[i]-a[i]) for a, b in zip(qs, qs[1:]) for i in range(6))

    print('\nV1 GUARDED TRANSPORT — OFFLINE JOINT-PLAN VALIDATION')
    print(f'Waypoints: {len(qs)}')
    print(f'Original MoveIt plan duration: {times[-1]-times[0]:.3f} s')
    print(f'Duration at {a.slowdown:g}x slowdown: {(times[-1]-times[0])*a.slowdown:.3f} s')
    print(f'Max planned-start vs cleaned-demo-start error: {demo_start_err:.6f} rad')
    print(f'Max planned-start vs saved-grip error: {grip_err:.6f} rad')
    print(f'Max planned-end vs cleaned-demo-end error: {end_err:.6f} rad')
    print(f'Max consecutive joint step: {max_step:.6f} rad')
    print(f'Wrist 3 start/end: {qs[0][5]:.6f} / {qs[-1][5]:.6f} rad')

    violations = []
    if demo_start_err > 0.02:
        violations.append('Plan start differs from recorded transport start by >0.02 rad')
    if grip_err > 0.015:
        violations.append('Plan start differs from saved grip pose by >0.015 rad')
    if end_err > 0.08:
        violations.append('Plan end differs from recorded transport end by >0.08 rad')
    if max_step > 0.05:
        violations.append('Joint step >0.05 rad')

    for i, joint in enumerate(JOINTS):
        v = [(b[i]-a[i])/d for a, b, d in zip(qs, qs[1:], dt)]
        acc = [(v[k+1]-v[k])/(0.5*(dt[k]+dt[k+1])) for k in range(len(v)-1)]
        mv = max(map(abs, v))
        ma = max(map(abs, acc)) if acc else 0.0
        print(f'{joint:22s} max |v|={mv:.5f} rad/s; max |a|={ma:.5f} rad/s^2')
        if mv > 0.22 or ma > 0.35:
            violations.append(f'{joint} exceeds conservative fake replay speed/accel thresholds')

    if violations:
        print('\nOFFLINE VALIDATION: FAIL')
        for problem in violations:
            print(' -', problem)
        raise SystemExit(1)
    print('\nOFFLINE VALIDATION: PASS — NOT EXECUTION APPROVAL')
    print('No ROS connection, robot action, or hardware movement.')
    print('Full trajectory clearance and physical fixture collisions remain unverified.')


if __name__ == '__main__':
    main()
