#!/usr/bin/env python3
"""Offline train ONE 6D Cartesian DMP on the trimmed real V1 transport.

This reproduces the ORIGINAL transport at tau=2, without extending the DMP
past its nominal duration and without artificially forcing its endpoint.
It does NOT publish to ROS, plan, or move any robot.
"""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_csv(path):
    with path.open(newline='') as f:
        r = csv.DictReader(f)
        required = ['time', 'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1',
                    'wrist_2', 'wrist_3', 'tcp_x', 'tcp_y', 'tcp_z',
                    'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw', 'gripper_width']
        if r.fieldnames != required:
            raise ValueError(f'Unexpected recording columns in {path}: {r.fieldnames}')
        rows = list(r)
    if len(rows) < 100:
        raise ValueError(f'Too few recorded points in {path}')
    values = np.array([[float(row[k]) for k in required] for row in rows])
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f'Invalid times or nonfinite data in {path}')
    if np.max(np.abs(np.linalg.norm(values[:, 10:14], axis=1)-1)) > 0.01:
        raise ValueError(f'Bad quaternions in {path}')
    return values


def check_trim_from_raw(clean, raw):
    """Fail closed if the existing clean file is not a contiguous raw subset."""
    if clean.shape[0] > raw.shape[0]:
        raise ValueError('Cleaned recording cannot be longer than raw')
    # Columns 1..14 (6 joints, TCP position, orientation, grip) are unchanged.
    first = clean[0, 1:]
    candidate_indices = np.flatnonzero(np.max(np.abs(raw[:, 1:] - first), axis=1) < 1e-8)
    for start in candidate_indices:
        end = start + len(clean)
        if end > len(raw):
            continue
        subset = raw[start:end]
        if (np.max(np.abs(subset[:, 1:] - clean[:, 1:])) < 1e-8 and
                np.max(np.abs((subset[:, 0] - subset[0, 0]) - clean[:, 0])) < 1e-6):
            return int(start), int(end-1)
    raise ValueError('Trimmed demonstration does not match a contiguous subset of raw; inspect before training')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project', type=Path, default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    ap.add_argument('--library', type=Path, default=Path.home()/'ros2_ws/src/rs2_dmp_demo')
    ap.add_argument('--tau', type=float, default=2.0)
    a = ap.parse_args()
    if not (0.5 <= a.tau <= 4):
        ap.error('tau must be in [0.5,4]')
    root = a.project.expanduser().resolve()
    lib = a.library.expanduser().resolve()
    raw_file = root/'demonstrations/raw/v1_transport_001.csv'
    demo_file = root/'demonstrations/processed/v1_transport_001_motion.csv'
    library_file = lib/'cdmp/src/cdmps.py'
    for required in [raw_file, demo_file, library_file]:
        if not required.exists():
            raise SystemExit(f'Missing prerequisite: {required}')
    out_dir = root/'demonstrations/processed/phase3_transport'
    target = out_dir/'v1_transport_cdmp_baseline_moveit.csv'
    manifest = out_dir/'phase3_transport_manifest.json'
    if target.exists() or manifest.exists():
        raise SystemExit('Existing transport CDMP output will not be overwritten')

    raw = load_csv(raw_file)
    clean = load_csv(demo_file)
    first_idx, last_idx = check_trim_from_raw(clean, raw)

    # This helper is the same proven input resampler as Phase 1/2, but we do
    # NOT use the Phase 2 settling extension that distorted the reach endpoint.
    phase2 = root/'scripts/phase2_reach'
    sys.path.insert(0, str(phase2))
    from generate_phase2_cdmp import load_clean_demo, wxyz_to_xyzw, quat_angle_deg
    sys.path.insert(0, str(lib))
    from cdmp.src.cdmps import CartesianDMPs

    t, p, q = load_clean_demo(demo_file)
    cdmp = CartesianDMPs()
    cdmp.load_demo(filename=demo_file.name, dem_time=t, dem_pos=p, dem_quat=q)
    cdmp.learn_cdmp(alpha_s=7.0, k_gain=100.0, rbfs_pSec=4.0)
    cdmp.init_reproduction(
        tau_scaler=a.tau,
        rep_abspose_start=np.r_[p[0], q[0]],
        rep_relpose_goal=np.array([0., 0., 0., 1., 0., 0., 0.]),
    )
    for s in cdmp.rep_cs.s_track:
        cdmp.rollout_step(curr_s_step=float(s))

    rt = np.asarray(cdmp.rep_time, dtype=float).reshape(-1)
    rp = np.asarray(cdmp.rep_pos, dtype=float)
    rq = np.asarray(cdmp.rep_quat, dtype=float)  # CDMP output WXYZ
    if not (len(rt) == len(rp) == len(rq)) or len(rt) < 100:
        raise RuntimeError('Invalid CDMP rollout dimensions')
    if np.any(np.diff(rt) <= 0) or not np.isfinite(rp).all() or not np.isfinite(rq).all():
        raise RuntimeError('Bad CDMP rollout values')
    rq_xyzw = np.column_stack([rq[:, 1], rq[:, 2], rq[:, 3], rq[:, 0]])
    norms = np.linalg.norm(rq_xyzw, axis=1)
    if np.any(norms < 0.9) or np.any(norms > 1.1):
        raise RuntimeError('Invalid CDMP quaternion norm')
    rq_xyzw /= norms[:, None]
    for i in range(1, len(rq_xyzw)):
        if np.dot(rq_xyzw[i], rq_xyzw[i-1]) < 0:
            rq_xyzw[i] *= -1

    position_mm = float(np.linalg.norm(rp[-1]-p[-1])*1000)
    rotation_deg = quat_angle_deg(rq_xyzw[-1], wxyz_to_xyzw(q[-1]))
    start_mm = float(np.linalg.norm(rp[0]-p[0])*1000)
    recorded_min_z = float(p[:, 2].min())
    learned_min_z = float(rp[:, 2].min())
    learned_max_z = float(rp[:, 2].max())
    recorded_max_z = float(p[:, 2].max())
    if start_mm > 2.0:
        raise RuntimeError(f'CDMP begins {start_mm:.3f} mm away from demonstration start')
    # A learned transport is allowed to have some endpoint residual because a
    # separate final ALIGN will be planned; leave residual visible and measured.
    # Large errors are diagnostics, NOT permission to execute.
    report = {
        'skill': 'V1 transport baseline, trained from one physical demonstration',
        'raw_demo': str(raw_file), 'raw_sha256': digest(raw_file),
        'source_demo': str(demo_file), 'clean_sha256': digest(demo_file),
        'clean_is_contiguous_raw_subset': True,
        'raw_first_index': first_idx, 'raw_last_index': last_idx,
        'training_samples': int(len(t)),
        'tau': a.tau, 'samples': int(len(rt)), 'duration_s': float(rt[-1]-rt[0]),
        'cdmp_start_error_mm': start_mm,
        'cdmp_position_goal_error_mm': position_mm,
        'cdmp_orientation_goal_error_deg': rotation_deg,
        'demonstrated_min_z_m': recorded_min_z,
        'demonstrated_max_z_m': recorded_max_z,
        'cdmp_min_z_m': learned_min_z,
        'cdmp_max_z_m': learned_max_z,
        'nominal_duration_only': True,
        'no_extra_settling': True,
        'robot_execution': False,
        'not_yet_approved_for_planning_or_execution': True,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    with target.open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for ti, pi, qi in zip(rt, rp, rq_xyzw):
            writer.writerow([f'{ti:.9f}'] + [f'{v:.12f}' for v in pi] + [f'{v:.12f}' for v in qi])
    with manifest.open('x') as f:
        json.dump(report, f, indent=2)
    print('\n=== V1 LEARNED TRANSPORT: OFFLINE CDMP ===')
    print(f'Cleaned demo: {len(t)} samples, duration {t[-1]:.3f} s')
    print(f'Raw provenance verified: rows {first_idx}–{last_idx}')
    print(f'CDMP output: {len(rt)} points, duration {rt[-1]-rt[0]:.3f} s, tau {a.tau:g}')
    print(f'Goal residual: {position_mm:.3f} mm / {rotation_deg:.3f} deg')
    print(f'Original demo Z range: {recorded_min_z*1000:.2f}–{recorded_max_z*1000:.2f} mm')
    print(f'Learned CDMP Z range: {learned_min_z*1000:.2f}–{learned_max_z*1000:.2f} mm')
    print(f'Saved: {target}')
    print('TRAINED ONCE. NOMINAL DMP ONLY. NO ROS INTERACTION OR ROBOT MOVEMENT.')
    print('NOT CLEARED FOR EXECUTION; review full path, start handoff, fixtures and IK first.')

if __name__ == '__main__':
    main()
