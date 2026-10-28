#!/usr/bin/env python3
"""Offline V1 INSERT refinement: replace only the early human positioning wobble.

The raw demonstration and previous INSERT/ROTATE segments stay untouched.
The first TCP pose is preserved, and a smooth, small TCP transition rejoins
an unmodified sample at ~1.50 s.  The original INSERT endpoint (ROTATE start)
is also preserved.  Only TCP/time data are written to the refined training CSV;
we deliberately do not fabricate corresponding robot joint demonstrations.

The existing 6-DOF Cartesian CDMP implementation trains the new candidate.
NO ROS imports, action clients, or robot movement.
"""
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quat_angle_deg(a, b):
    dot = abs(float(np.dot(a, b))) / (float(np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(2.0 * math.acos(max(0.0, min(1.0, dot))))


def xy_drift_mm(p):
    return float(np.max(np.linalg.norm(p[:, :2] - p[0, :2], axis=1)) * 1000.0)


def main():
    root = Path(__file__).resolve().parents[2]
    original_path = root / 'demonstrations/processed/v1_insert_001_motion.csv'
    rotate_path = root / 'demonstrations/processed/v1_rotate_001_motion.csv'
    raw_path = root / 'demonstrations/raw/v1_assembly_full_001.csv'
    saved_path = root / 'config/saved_poses/v1_insertion_ready.yaml'
    output_dir = root / 'demonstrations/processed/phase4_assembly'
    clean_path = output_dir / 'v1_insert_refined_tcp_training.csv'
    rollout_path = output_dir / 'v1_insert_cdmp_refined_CANDIDATE.csv'
    report_path = output_dir / 'v1_insert_refinement_report.json'

    for p in (original_path, rotate_path, raw_path, saved_path):
        if not p.is_file():
            raise SystemExit(f'BLOCKED: Required input missing: {p}')
    for p in (clean_path, rollout_path, report_path):
        if p.exists():
            raise SystemExit(f'BLOCKED: Existing output would be overwritten: {p}')

    sys.path.insert(0, str(root / 'scripts/phase4_assembly'))
    from train_v1_insert_rotate_cdmp import (read_rows, verify_subset,
                                              learn, angular_degrees)
    sys.path.insert(0, str(root / 'scripts/phase2_reach'))
    from generate_phase2_cdmp import load_clean_demo
    sys.path.insert(0, str(Path.home() / 'ros2_ws/src/rs2_dmp_demo'))
    from cdmp.src.cdmps import CartesianDMPs
    from scipy.spatial.transform import Rotation, Slerp
    import yaml

    original = read_rows(original_path)
    raw = read_rows(raw_path)
    rotate = read_rows(rotate_path)
    assert original.shape[1] == 15
    source_range = verify_subset(original, raw, 'insert')
    rotate_range = verify_subset(rotate, raw, 'rotate')
    if source_range[1] != rotate_range[0] or not np.allclose(original[-1, 1:], rotate[0, 1:], atol=1e-9, rtol=0):
        raise SystemExit('BLOCKED: INSERT and ROTATE raw endpoints do not share same sample')

    t = original[:, 0] - original[0, 0]
    p0 = original[:, 7:10].copy()
    q0 = original[:, 10:14].copy()  # XYZW
    join_i = int(np.argmin(abs(t-1.50)))
    join_t = float(t[join_i])
    if not (1.45 < join_t < 1.55):
        raise SystemExit(f'BLOCKED: Unexpected join time: {join_t}')
    if np.any(np.diff(t) <= 0):
        raise SystemExit('BLOCKED: Non-monotonic timestamps')

    with saved_path.open() as f:
        saved = yaml.safe_load(f)
    if saved.get('base_frame') != 'base' or saved.get('tool_frame') != 'gripper_tcp':
        raise SystemExit('BLOCKED: Unexpected saved-pose frames')
    saved_p = np.asarray(saved['tcp']['position_m'], dtype=float)
    saved_q = np.asarray(saved['tcp']['quaternion_xyzw'], dtype=float)
    start_mm = float(np.linalg.norm(p0[0] - saved_p) * 1000)
    start_deg = quat_angle_deg(q0[0], saved_q)
    if start_mm > 0.3 or start_deg > 0.2:
        raise SystemExit(f'BLOCKED: Original start not aligned to insertion-ready: {start_mm:.3f} mm / {start_deg:.3f} deg')

    p = p0.copy()
    q = q0.copy()
    u = np.clip(t[:join_i + 1] / join_t, 0, 1)
    smooth = 3 * u**2 - 2 * u**3
    p[:join_i + 1] = p0[0] + smooth[:, None] * (p0[join_i] - p0[0])
    q[:join_i + 1] = Slerp(
        [0., join_t], Rotation.from_quat([q0[0], q0[join_i]])
    )(join_t * smooth).as_quat()
    # Everything from the exact rejoin sample onward is untouched.
    if (not np.allclose(p[join_i:], p0[join_i:], atol=1e-12, rtol=0) or
            quat_angle_deg(q[-1], q0[-1]) > 1e-6):
        raise SystemExit('BLOCKED: Rejoin or endpoint modified unexpectedly')

    step_mm = np.linalg.norm(np.diff(p, axis=0), axis=1)*1000
    original_step_mm = np.linalg.norm(np.diff(p0, axis=0), axis=1)*1000
    join_step_mm = float(np.max(step_mm[max(0, join_i-2):join_i+2]))
    if join_step_mm > 1.0:
        raise SystemExit(f'BLOCKED: Excessive transition step at join: {join_step_mm:.3f} mm / sample')

    output_dir.mkdir(parents=True, exist_ok=True)
    with clean_path.open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'tcp_x', 'tcp_y', 'tcp_z',
                         'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw'])
        for ti, pi, qi in zip(t, p, q):
            writer.writerow([f'{ti:.9f}'] + [f'{x:.12f}' for x in pi] +
                            [f'{x:.12f}' for x in qi])

    print('\n=== REFINED V1 INSERT TRAINING — OFFLINE ===')
    print(f'Raw provenance: INSERT rows {source_range[0]}–{source_range[1]}, ROTATE starts at {rotate_range[0]}')
    print(f'Start vs insertion-ready: {start_mm:.3f} mm / {start_deg:.3f} deg (unchanged)')
    print(f'Wobble replacement: 0.00–{join_t:.3f} s; rejoin exact original sample {join_i}')
    print(f'Original max XY drift: {xy_drift_mm(p0):.3f} mm')
    print(f'Refined-demo max XY drift: {xy_drift_mm(p):.3f} mm')
    print(f'Largest refined pose step: {step_mm.max():.3f} mm/sample (original {original_step_mm.max():.3f})')
    print(f'Largest local rejoin step: {join_step_mm:.3f} mm/sample')
    print('Training new CDMP once on refined TCP/time data...\n')

    tt, pp, qq, metrics = learn(clean_path, 2.0, load_clean_demo, CartesianDMPs)
    with rotate_path.open(newline='') as f:
        rotate_first = next(csv.DictReader(f))
    # Compare generated INSERT end against original ROTATE start, which is an
    # exact raw recorded boundary; the ROTATE CDMP also starts from this pose.
    boundary_p = np.array([float(rotate_first['tcp_' + k]) for k in 'xyz'])
    boundary_q = np.array([float(rotate_first['tcp_q' + k]) for k in 'xyzw'])
    handoff_mm = float(np.linalg.norm(pp[-1] - boundary_p) * 1000)
    handoff_deg = angular_degrees(qq[-1], boundary_q)
    dz_overshoot = max(0.0, float((p[:, 2].min() - pp[:, 2].min()) * 1000))

    with rollout_path.open('x', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for ti, pi, qi in zip(tt, pp, qq):
            w.writerow([f'{ti:.9f}'] + [f'{x:.12f}' for x in pi] +
                       [f'{x:.12f}' for x in qi])

    report = {
        'purpose': 'Candidate modified initial TCP motion; not raw recorded joints',
        'original_demo_sha256': digest(original_path),
        'raw_demo_sha256': digest(raw_path),
        'original_insert_raw_range': source_range,
        'rotate_raw_range': rotate_range,
        'join_original_s': join_t,
        'modified_time_range_s': [0.0, join_t],
        'preserves_start_pose': True,
        'preserves_exact_original_demo_at_and_after_join': True,
        'preserves_insert_rotate_boundary': True,
        'max_XY_drift_original_demo_mm': xy_drift_mm(p0),
        'max_XY_drift_refined_demo_mm': xy_drift_mm(p),
        'max_XY_drift_refined_CDMP_mm': xy_drift_mm(pp),
        'learned_CDMP_min_Z_mm': float(pp[:, 2].min() * 1000),
        'refined_demo_min_Z_mm': float(p[:, 2].min() * 1000),
        'extra_learned_depth_mm': dz_overshoot,
        'generated_to_rotate_start_handoff_mm': handoff_mm,
        'generated_to_rotate_start_handoff_deg': handoff_deg,
        'cdmp_fidelity': metrics,
        'not_execution_approval': True,
    }
    with report_path.open('x') as f:
        json.dump(report, f, indent=2)

    print('\n=== REFINED CDMP CANDIDATE ===')
    print(f'Points: {len(tt)}, 2x duration: {tt[-1]:.3f} s')
    print(f'Endpoint error: {metrics["endpoint_position_error_mm"]:.3f} mm / {metrics["endpoint_orientation_error_deg"]:.3f} deg')
    print(f'Max XY drift: {xy_drift_mm(pp):.3f} mm')
    print(f'RMS/max path difference: {metrics["phase_matched_RMS_position_mm"]:.3f} / {metrics["phase_matched_max_position_mm"]:.3f} mm')
    print(f'Min Z demo/CDMP: {p[:, 2].min()*1000:.3f} / {pp[:, 2].min()*1000:.3f} mm')
    print(f'Additional downward depth: {dz_overshoot:.3f} mm')
    print(f'End vs ROTATE start: {handoff_mm:.3f} mm / {handoff_deg:.3f} deg')
    print(f'Saved cleaned TCP training data: {clean_path}')
    print(f'Saved learned CDMP candidate: {rollout_path}')
    print(f'Saved report: {report_path}')
    print('NO ROS ACTION, NO ROBOT MOVEMENT. NOT CLEARED FOR EXECUTION.')
    if dz_overshoot > 0.10:
        print('REVIEW REQUIRED: Insertion depth overshoot exceeds 0.10 mm.')


if __name__ == '__main__':
    main()
