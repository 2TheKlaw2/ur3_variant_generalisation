#!/usr/bin/env python3
"""Offline six-DOF V1 CDMP goal and temporal adaptation. No ROS nodes or actions.

Trains once on the real cleaned V1 demonstration, then reproduces with a new
Cartesian goal from v1_grip_pose.yaml, at several temporal scales. Uses the
existing Victor Hernandez-Moreno CartesianDMPs, not CSV translation/warping.
"""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

# CSV uses XYZW; CartesianDMPs expects WXYZ.
def xyzw_to_wxyz(q):
    q = np.asarray(q, dtype=float)
    return np.r_[q[3], q[:3]]


def wxyz_to_xyzw(q):
    q = np.asarray(q, dtype=float)
    return np.r_[q[1:4], q[0]]


def quat_angle_deg(a_xyzw, b_xyzw):
    return float(np.degrees((Rotation.from_quat(a_xyzw).inv() *
                             Rotation.from_quat(b_xyzw)).magnitude()))


def load_clean_demo(path, hz=50.0):
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        needed = ['time', 'tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw']
        if not reader.fieldnames or any(k not in reader.fieldnames for k in needed):
            raise RuntimeError('Clean demonstration missing TCP/time fields')
        records = list(reader)
    if len(records) < 10:
        raise RuntimeError('Not enough samples in cleaned demonstration')
    a = np.asarray([[float(r[k]) for k in needed] for r in records], dtype=float)
    if not np.isfinite(a).all() or np.any(np.diff(a[:, 0]) <= 0):
        raise RuntimeError('Demo timestamps or values invalid')
    t = a[:, 0] - a[0, 0]
    p = a[:, 1:4]
    q = a[:, 4:8]
    norms = np.linalg.norm(q, axis=1)
    if np.any(abs(norms-1) > 0.01):
        raise RuntimeError('Invalid demonstration quaternion')
    q = q / norms[:, None]
    for i in range(1, len(q)):
        if np.dot(q[i], q[i-1]) < 0:
            q[i] *= -1
    count = max(3, round(t[-1] * hz) + 1)
    uniform_t = np.linspace(0, t[-1], count)
    uniform_p = np.column_stack([np.interp(uniform_t, t, p[:, i]) for i in range(3)])
    uniform_q = Slerp(t, Rotation.from_quat(q))(uniform_t).as_quat()
    for i in range(1, len(uniform_q)):
        if np.dot(uniform_q[i], uniform_q[i-1]) < 0:
            uniform_q[i] *= -1
    uniform_q_wxyz = np.column_stack([uniform_q[:, 3], uniform_q[:, 0], uniform_q[:, 1], uniform_q[:, 2]])
    return uniform_t, uniform_p, uniform_q_wxyz


def read_grip(path):
    with path.open() as f:
        data = yaml.safe_load(f)
    if data.get('base_frame') != 'base' or data.get('tool_frame') != 'gripper_tcp':
        raise RuntimeError('Saved grip pose must be base -> gripper_tcp; do not adapt mixed frames')
    p = np.asarray(data['tcp']['position_m'], dtype=float)
    q = np.asarray(data['tcp']['quaternion_xyzw'], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise RuntimeError('Invalid saved grip pose')
    n = np.linalg.norm(q)
    if abs(n-1.0) > 0.01:
        raise RuntimeError('Non-unit saved grip orientation')
    return p, q/n


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_reproduction(cdmp, original_start, original_start_q, original_goal,
                       original_goal_q, target_p, target_q_xyzw, tau):
    # Goal delta is expressed in the same BASE frame as the recorded TCP.
    # Quaternion multiplication: q_relative = q_target * inverse(q_demo_goal).
    source_q = Rotation.from_quat(wxyz_to_xyzw(original_goal_q))
    target_q = Rotation.from_quat(target_q_xyzw)
    q_relative = (target_q * source_q.inv()).as_quat()
    goal_delta = target_p - original_goal
    cdmp.init_reproduction(
        tau_scaler=tau,
        rep_abspose_start=np.r_[original_start, original_start_q],
        rep_relpose_goal=np.r_[goal_delta, xyzw_to_wxyz(q_relative)],
    )
    for s in cdmp.rep_cs.s_track:
        cdmp.rollout_step(curr_s_step=float(s))
    times = np.asarray(cdmp.rep_time).reshape(-1)
    positions = np.asarray(cdmp.rep_pos)
    quats = np.asarray(cdmp.rep_quat)  # WXYZ
    if len(times) != len(positions) or len(times) != len(quats):
        raise RuntimeError('CDMP rollout size mismatch')
    out_q = np.column_stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]])
    out_q /= np.linalg.norm(out_q, axis=1)[:, None]
    for i in range(1, len(out_q)):
        if np.dot(out_q[i], out_q[i-1]) < 0:
            out_q[i] *= -1
    if not np.isfinite(positions).all() or np.any(np.diff(times) <= 0):
        raise RuntimeError('Nonfinite or nonmonotonic CDMP rollout')
    err_mm = float(np.linalg.norm(positions[-1]-target_p)*1000)
    angle_deg = quat_angle_deg(out_q[-1], target_q_xyzw)
    # Offline DMP integration tolerance, not robot positional accuracy.
    if err_mm > 2.0 or angle_deg > 0.5:
        raise RuntimeError(f'Goal not converged: {err_mm:.3f} mm / {angle_deg:.3f} deg')
    return times, positions, out_q, err_mm, angle_deg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project', type=Path,
                    default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    ap.add_argument('--library', type=Path,
                    default=Path.home()/'ros2_ws/src/rs2_dmp_demo')
    ap.add_argument('--output-dir', type=Path)
    args = ap.parse_args()
    root = args.project.expanduser().resolve()
    demo = root/'demonstrations/processed/v1_reach_001_motion.csv'
    grip = root/'config/saved_poses/v1_grip_pose.yaml'
    library = args.library.expanduser().resolve()
    for f in (demo, grip, library/'cdmp/src/cdmps.py'):
        if not f.is_file():
            raise SystemExit(f'Missing input: {f}')
    # Import from the user's existing implementation. Requires its dependencies
    # (including pyquaternion), as used in the successful earlier CDMP generation.
    sys.path.insert(0, str(library))
    from cdmp.src.cdmps import CartesianDMPs
    output = args.output_dir or root/'demonstrations/processed/phase2_reach'
    output = output.expanduser().resolve()
    trials = [
        ('baseline_tau2', 2.0, False),
        ('adapted_tau2', 2.0, True),
        ('adapted_tau1', 1.0, True),
        ('adapted_tau3', 3.0, True),
    ]
    planned_files = [output/f'v1_reach_{name}_moveit.csv' for name, _, _ in trials]
    manifest = output/'phase2_manifest.json'
    existing = [x for x in planned_files+[manifest] if x.exists()]
    if existing:
        raise SystemExit('Refusing to overwrite: ' + ', '.join(map(str, existing)))
    t, p, q = load_clean_demo(demo)
    grip_p, grip_q = read_grip(grip)
    original_goal = p[-1].copy()
    original_goal_q = q[-1].copy()
    offset = grip_p-original_goal
    print('\nPHASE 2 GOAL ADAPTATION — OFFLINE ONLY')
    print('Demo samples:', len(t), 'duration:', round(t[-1],3), 's')
    print('Demo TCP endpoint (m):', original_goal.tolist())
    print('Saved grip TCP goal (m):', grip_p.tolist())
    print('Translation offset (mm):', np.round(offset*1000, 4).tolist())
    print('Goal separation (mm):', round(float(np.linalg.norm(offset)*1000), 4))
    print('Goal orientation offset (deg):', round(quat_angle_deg(wxyz_to_xyzw(original_goal_q), grip_q),4))
    print('No robot or ROS interaction.\n')
    # Train ONCE. Different goals and tau scales reuse the SAME learned weights.
    cdmp = CartesianDMPs()
    cdmp.load_demo(filename=demo.name, dem_time=t, dem_pos=p, dem_quat=q)
    cdmp.learn_cdmp(alpha_s=7.0, k_gain=100.0, rbfs_pSec=4.0)
    results = []
    all_outputs = []
    for name, tau, adapted in trials:
        goal_p = grip_p if adapted else original_goal
        goal_q = grip_q if adapted else wxyz_to_xyzw(original_goal_q)
        rt, rp, rq, err_mm, err_deg = build_reproduction(
            cdmp, p[0], q[0], original_goal, original_goal_q,
            goal_p, goal_q, tau)
        path = output/f'v1_reach_{name}_moveit.csv'
        all_outputs.append((path, rt, rp, rq))
        results.append({
            'trial': name, 'adapted': adapted, 'tau_scale': tau,
            'samples': len(rt), 'duration_s': float(rt[-1]),
            'start_m': rp[0].tolist(), 'goal_requested_m': goal_p.tolist(),
            'goal_reached_m': rp[-1].tolist(),
            'goal_position_error_mm': err_mm, 'goal_orientation_error_deg': err_deg,
            'output_csv': str(path),
        })
        print(f'{name:17s}: {len(rt):4d} points | {rt[-1]:6.2f} s | '
              f'position error {err_mm:.4f} mm | orientation error {err_deg:.4f} deg')
    meta = {
        'experiment': 'V1 single-demonstration CDMP Cartesian goal and temporal adaptation',
        'source_demo': str(demo), 'source_demo_sha256': digest(demo),
        'saved_grip_pose': str(grip), 'saved_grip_pose_sha256': digest(grip),
        'base_frame': 'base', 'tool_frame': 'gripper_tcp',
        'translation_goal_offset_mm': (offset*1000).tolist(),
        'orientation_goal_offset_deg': quat_angle_deg(wxyz_to_xyzw(original_goal_q), grip_q),
        'learned_weights_reused_across_all_trials': True,
        'rollouts': results, 'physical_execution': False,
    }
    output.mkdir(parents=True, exist_ok=True)
    for dest, rt, rp, rq in all_outputs:
        with dest.open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time','x','y','z','qx','qy','qz','qw'])
            for ts, pt, quat in zip(rt, rp, rq):
                writer.writerow([f'{ts:.9f}'] + [f'{x:.12f}' for x in pt] +
                                [f'{x:.12f}' for x in quat])
    with manifest.open('x') as f:
        json.dump(meta, f, indent=2)
    print('\nSaved:', output)
    print('TRAINED ONCE; ADAPTED GOALS; NO ROBOT MOVEMENT.')


if __name__ == '__main__':
    main()
