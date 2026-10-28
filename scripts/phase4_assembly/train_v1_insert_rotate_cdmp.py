#!/usr/bin/env python3
"""Offline CDMP baseline training of V1 INSERT and ROTATE from ONE physical recording.

Uses the already installed Victor Hernandez-Moreno CartesianDMPs implementation.
No ROS imports, no planning, no command to the robot. Generated trajectories are
CANDIDATES, not approval for physical or fake-hardware execution.
"""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    with path.open(newline='') as f:
        rd = csv.DictReader(f)
        fields = rd.fieldnames
        required = ['time', 'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1',
                    'wrist_2', 'wrist_3', 'tcp_x', 'tcp_y', 'tcp_z',
                    'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw', 'gripper_width']
        if fields != required:
            raise RuntimeError(f'Unexpected columns in {path}: {fields}')
        rows = list(rd)
    if len(rows) < 50:
        raise RuntimeError(f'Too few samples in {path}')
    vals = np.asarray([[float(r[k]) for k in required] for r in rows], dtype=float)
    if not np.isfinite(vals).all() or not np.all(np.diff(vals[:, 0]) > 0):
        raise RuntimeError(f'Invalid data or time order in {path}')
    qnorm = np.linalg.norm(vals[:, 10:14], axis=1)
    if np.any(abs(qnorm - 1) > 0.01):
        raise RuntimeError(f'Invalid quaternion magnitude in {path}')
    return vals


def verify_subset(clean, raw, name):
    # All non-time raw columns must be preserved exactly, and clean time reset.
    if len(clean) > len(raw):
        raise RuntimeError(f'{name}: cleaned segment exceeds raw length')
    match = np.flatnonzero(np.max(np.abs(raw[:, 1:] - clean[0, 1:]), axis=1) < 1e-8)
    for i in match:
        part = raw[i:i+len(clean)]
        if (len(part) == len(clean) and
            np.max(np.abs(part[:, 1:] - clean[:, 1:])) < 1e-8 and
            np.max(np.abs((part[:, 0] - part[0, 0]) - clean[:, 0])) < 1e-6):
            return int(i), int(i+len(clean)-1)
    raise RuntimeError(f'{name}: segment is not an unchanged contiguous subset of raw')


def angular_degrees(x, y):
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    d = abs(float(np.dot(x, y)))
    return math.degrees(2 * math.acos(min(1.0, max(0.0, d))))


def learn(path, tau, load_clean_demo, CartesianDMPs):
    t, pos, quat_wxyz = load_clean_demo(path)
    model = CartesianDMPs()
    model.load_demo(filename=path.name, dem_time=t, dem_pos=pos, dem_quat=quat_wxyz)
    model.learn_cdmp(alpha_s=7.0, k_gain=100.0, rbfs_pSec=4.0)
    model.init_reproduction(
        tau_scaler=tau,
        rep_abspose_start=np.r_[pos[0], quat_wxyz[0]],
        rep_relpose_goal=np.array([0., 0., 0., 1., 0., 0., 0.]),
    )
    for phase in model.rep_cs.s_track:
        model.rollout_step(curr_s_step=float(phase))

    tt = np.asarray(model.rep_time, dtype=float).reshape(-1)
    pp = np.asarray(model.rep_pos, dtype=float)
    qq = np.asarray(model.rep_quat, dtype=float)  # WXYZ
    if (len(tt) != len(pp) or len(pp) != len(qq) or len(tt) < 100 or
            np.any(np.diff(tt) <= 0) or
            not np.isfinite(pp).all() or not np.isfinite(qq).all()):
        raise RuntimeError(f'{path.name}: invalid CDMP rollout')
    xyzw = qq[:, [1, 2, 3, 0]].copy()
    norms = np.linalg.norm(xyzw, axis=1)
    if np.any(norms < 0.9) or np.any(norms > 1.1):
        raise RuntimeError(f'{path.name}: invalid CDMP quaternion norm')
    xyzw /= norms[:, None]
    for i in range(1, len(xyzw)):
        if np.dot(xyzw[i], xyzw[i-1]) < 0:
            xyzw[i] = -xyzw[i]
    demo_xyzw = quat_wxyz[:, [1, 2, 3, 0]]

    # Fidelity measured against demonstration at equivalent NORMALISED phase.
    # Interpolation is for comparison, not altering the learned rollout.
    phase_demo = t / t[-1]
    phase_rep = (tt - tt[0]) / (tt[-1] - tt[0])
    ref_p = np.column_stack([np.interp(phase_rep, phase_demo, pos[:, i]) for i in range(3)])
    deviation_mm = np.linalg.norm(pp - ref_p, axis=1)*1000
    from scipy.spatial.transform import Rotation, Slerp
    ref_q = Slerp(phase_demo, Rotation.from_quat(demo_xyzw))(phase_rep).as_quat()
    quat_dot = np.abs(np.sum(ref_q*xyzw, axis=1)).clip(0, 1)
    deviation_deg = np.degrees(2*np.arccos(quat_dot))

    report = {
        'input': str(path), 'input_sha256': sha256(path),
        'demo_samples': int(len(t)), 'demo_duration_s': float(t[-1]),
        'tau': tau, 'cdmp_points': int(len(tt)), 'cdmp_duration_s': float(tt[-1]-tt[0]),
        'start_error_mm': float(np.linalg.norm(pp[0]-pos[0])*1000),
        'endpoint_position_error_mm': float(np.linalg.norm(pp[-1]-pos[-1])*1000),
        'endpoint_orientation_error_deg': angular_degrees(xyzw[-1], demo_xyzw[-1]),
        'phase_matched_RMS_position_mm': float(np.sqrt(np.mean(deviation_mm**2))),
        'phase_matched_max_position_mm': float(deviation_mm.max()),
        'phase_matched_max_orientation_deg': float(deviation_deg.max()),
        'demo_min_z_mm': float(pos[:, 2].min()*1000),
        'cdmp_min_z_mm': float(pp[:, 2].min()*1000),
        'cdmp_end_z_mm': float(pp[-1, 2]*1000),
        'demo_end_z_mm': float(pos[-1, 2]*1000),
        'cdmp_max_xy_drift_from_start_mm': float(np.max(np.linalg.norm(pp[:, :2]-pp[0, :2], axis=1))*1000),
        'cdmp_max_rotation_from_start_deg': float(max(angular_degrees(xyzw[0], q) for q in xyzw)),
        'nominal_rollout_only': True,
        'reproduced_demonstration_without_goal_adaptation': True,
        'not_validated_for_motion_execution': True,
    }
    return tt, pp, xyzw, report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project', type=Path, default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    ap.add_argument('--library', type=Path, default=Path.home()/'ros2_ws/src/rs2_dmp_demo')
    ap.add_argument('--tau', type=float, default=2.0)
    a = ap.parse_args()
    if not (1 <= a.tau <= 3):
        ap.error('tau must be between 1 and 3')
    root = a.project.expanduser().resolve()
    lib = a.library.expanduser().resolve()
    raw_path = root/'demonstrations/raw/v1_assembly_full_001.csv'
    processed = root/'demonstrations/processed'
    sources = {
        'insert': processed/'v1_insert_001_motion.csv',
        'rotate': processed/'v1_rotate_001_motion.csv',
    }
    combined_path = processed/'v1_assembly_001_motion.csv'
    for path in [raw_path, combined_path, *sources.values(), lib/'cdmp/src/cdmps.py',
                 root/'scripts/phase2_reach/generate_phase2_cdmp.py']:
        if not path.is_file():
            raise SystemExit(f'Missing required input: {path}')

    raw = read_rows(raw_path)
    insert = read_rows(sources['insert'])
    rotate = read_rows(sources['rotate'])
    combined = read_rows(combined_path)
    insert_range = verify_subset(insert, raw, 'insert')
    rotate_range = verify_subset(rotate, raw, 'rotate')
    combined_range = verify_subset(combined, raw, 'combined')
    if (insert_range[1] != rotate_range[0] or
        combined_range != (insert_range[0], rotate_range[1]) or
        np.max(abs(insert[-1, 1:] - rotate[0, 1:])) > 1e-8):
        raise SystemExit('BLOCKED: INSERT / ROTATE do not share the same raw boundary sample')

    sys.path.insert(0, str(root/'scripts/phase2_reach'))
    from generate_phase2_cdmp import load_clean_demo
    sys.path.insert(0, str(lib))
    from cdmp.src.cdmps import CartesianDMPs

    output = processed/'phase4_assembly'
    outputs = {key: output/f'v1_{key}_cdmp_baseline_CANDIDATE.csv' for key in sources}
    manifest = output/'v1_insert_rotate_cdmp_report.json'
    existing = [p for p in [*outputs.values(), manifest] if p.exists()]
    if existing:
        raise SystemExit('Refusing to overwrite existing outputs: ' + ', '.join(map(str, existing)))

    results = {}
    for skill in ('insert', 'rotate'):
        print(f'\n========== TRAINING {skill.upper()} ==========', flush=True)
        rt, rp, rq, report = learn(sources[skill], a.tau, load_clean_demo, CartesianDMPs)
        results[skill] = (rt, rp, rq, report)
        print(f'{skill.upper():6s}: {len(rt)} pts / {rt[-1]:.3f} s; '
              f'goal error {report["endpoint_position_error_mm"]:.3f} mm / '
              f'{report["endpoint_orientation_error_deg"]:.3f} deg')
        print(f'         min Z demo/CDMP {report["demo_min_z_mm"]:.3f} / '
              f'{report["cdmp_min_z_mm"]:.3f} mm; '
              f'max XY drift {report["cdmp_max_xy_drift_from_start_mm"]:.3f} mm')
        print(f'         RMS/maximum shape difference '
              f'{report["phase_matched_RMS_position_mm"]:.3f} / '
              f'{report["phase_matched_max_position_mm"]:.3f} mm')

    # Check the *generated* INSERT -> ROTATE handoff; matching source samples
    # does NOT guarantee matching independent CDMP reproductions.
    insert_pp, insert_qq = results['insert'][1:3]
    rotate_pp, rotate_qq = results['rotate'][1:3]
    boundary_mm = float(np.linalg.norm(insert_pp[-1]-rotate_pp[0])*1000)
    boundary_deg = angular_degrees(insert_qq[-1], rotate_qq[0])
    print(f'\nGENERATED INSERT -> ROTATE handoff: {boundary_mm:.3f} mm / {boundary_deg:.3f} deg')
    print('Reminder: source-demo boundary is identical; learned rollout boundary may not be.')

    meta = {
        'experiment': 'V1 insertion and rotation learned separately from one physical assembly demonstration',
        'raw_demo': str(raw_path), 'raw_sha256': sha256(raw_path),
        'source_raw_row_ranges_inclusive': {'insert': insert_range, 'rotate': rotate_range},
        'split_boundary_identical_in_source': True,
        'base_frame': 'base', 'tool_frame': 'gripper_tcp',
        'generated_skill_handoff_position_mm': boundary_mm,
        'generated_skill_handoff_orientation_deg': boundary_deg,
        'rollouts': {skill: data[3] for skill, data in results.items()},
        'no_robot_movement': True, 'not_execution_approval': True,
    }
    output.mkdir(parents=True, exist_ok=True)
    for skill, (rt, rp, rq, _) in results.items():
        with outputs[skill].open('x', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for tt, pp, qq in zip(rt, rp, rq):
                w.writerow([f'{tt:.9f}'] + [f'{x:.12f}' for x in pp] + [f'{x:.12f}' for x in qq])
    with manifest.open('x') as f:
        json.dump(meta, f, indent=2)
    print('\nSaved offline candidate files and report to:', output)
    print('NO ROS INTERACTION OR ROBOT MOVEMENT.')
    print('NOT CLEARED FOR MOVEIT EXECUTION: inspect overshoot, phase fidelity, handoff, and workcell clearance.')


if __name__ == '__main__':
    main()
