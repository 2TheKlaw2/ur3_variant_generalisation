#!/usr/bin/env python3
"""Offline depth-floor guard for the refined V1 Cartesian INSERT CDMP.

Preserves the original refined CDMP input and the INSERT start/end poses;
modifies ONLY TCP z near or below the minimum demonstrated insertion height.
The modification is a smooth C2 remapping of z, not additional DMP learning.
NO ROS imports and NO robot commands. Not physical clearance approval.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def load_rows(path: Path, expected: list[str]):
    with path.open(newline='') as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames != expected:
            raise SystemExit(f'BLOCKED: unexpected columns in {path}: {reader.fieldnames}')
        rows = list(reader)
    if len(rows) < 10:
        raise SystemExit(f'BLOCKED: too few samples: {path}')
    return rows


def smooth_guard(z: float, floor: float, margin: float) -> float:
    """C2 map: z<=floor -> floor; z>=floor+margin -> unchanged."""
    d = z - floor
    if d <= 0.0:
        return floor
    if d >= margin:
        return z
    u = d / margin
    s = u*u*u * (10.0 + u*(-15.0 + 6.0*u))  # smoothstep5
    return floor + d*s


def main():
    root = Path(__file__).resolve().parents[2]
    outdir = root / 'demonstrations/processed/phase4_assembly'
    source = outdir / 'v1_insert_cdmp_refined_CANDIDATE.csv'
    reference = outdir / 'v1_insert_refined_tcp_training.csv'
    rotate = root / 'demonstrations/processed/v1_rotate_001_motion.csv'
    target = outdir / 'v1_insert_cdmp_refined_depth_guard_CANDIDATE.csv'
    report = outdir / 'v1_insert_depth_guard_report.json'
    for p in (source, reference, rotate):
        if not p.is_file():
            raise SystemExit(f'BLOCKED: missing source {p}')
    for p in (target, report):
        if p.exists():
            raise SystemExit(f'BLOCKED: output exists; refusing to overwrite: {p}')

    keys = ['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    rows = load_rows(source, keys)
    ref = load_rows(reference, ['time', 'tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw'])
    rotation_rows = load_rows(rotate, ['time', 'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3', 'tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw', 'gripper_width'])
    z = [float(row['z']) for row in rows]
    t = [float(row['time']) for row in rows]
    if not all(math.isfinite(v) for v in t+z) or not all(b>a for a,b in zip(t,t[1:])):
        raise SystemExit('BLOCKED: invalid times or heights')

    floor = min(float(r['tcp_z']) for r in ref)
    margin = 0.010e-3  # 0.010 mm smooth transition above the demonstrated minimum
    if min(z) >= floor:
        raise SystemExit('NO CHANGE: learned INSERT does not go below demo floor')
    if not (z[0] > floor + margin and z[-1] > floor + margin):
        raise SystemExit('BLOCKED: start/end pose too near height floor to preserve exactly')

    new_z = [smooth_guard(v, floor, margin) for v in z]
    guarded = []
    changes = []
    for row, old, new in zip(rows, z, new_z):
        r = row.copy()
        if new != old:
            r['z'] = f'{new:.12f}'
        guarded.append(r)
        changes.append((new-old)*1000)

    if guarded[0] != rows[0] or guarded[-1] != rows[-1]:
        raise SystemExit('BLOCKED: trajectory start/end changed')
    if any(a[k] != b[k] for a,b in zip(rows,guarded) for k in keys if k != 'z'):
        raise SystemExit('BLOCKED: unexpectedly modified non-Z field')
    if min(float(row['z']) for row in guarded) < floor - 1e-9:
        raise SystemExit('BLOCKED: depth constraint violation')
    if max(changes) > 0.5:  # mm; protects against modifying a grossly wrong input
        raise SystemExit('BLOCKED: unexpected >0.5 mm correction required')

    def xyz(row, prefix=''):
        return [float(row[prefix+k]) for k in ('x','y','z')]
    end = xyz(guarded[-1]); rotate_first = xyz(rotation_rows[0], 'tcp_')
    handoff_mm = math.dist(end, rotate_first)*1000
    if handoff_mm > 0.1:
        raise SystemExit(f'BLOCKED: insertion/rotation translation handoff {handoff_mm:.4f} mm')

    def max_dz(series):
        return max(abs(b-a) for a,b in zip(series, series[1:]))*1000
    def max_z_accel(series):
        vel = [(b-a)/(t2-t1) for a,b,t1,t2 in zip(series,series[1:],t,t[1:])]
        return max(abs(b-a)/(0.5*(t[i+2]-t[i])) for i,(a,b) in enumerate(zip(vel,vel[1:])))

    with target.open('x', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        writer.writerows(guarded)

    summary = {
        'purpose': 'Constrained insertion depth; preserves refined learned CDMP except Z near floor',
        'method': 'C2 quintic smooth height floor; not a learned modification',
        'source': str(source), 'source_sha256': sha256(source),
        'reference': str(reference), 'reference_sha256': sha256(reference),
        'num_points': len(rows),
        'minimum_reference_Z_mm': floor*1000,
        'original_min_Z_mm': min(z)*1000,
        'guarded_min_Z_mm': min(float(r['z']) for r in guarded)*1000,
        'transition_margin_mm': margin*1000,
        'max_Z_correction_mm': max(changes),
        'changed_points': sum(c > 1e-9 for c in changes),
        'max_original_Z_step_mm': max_dz(z),
        'max_guarded_Z_step_mm': max_dz([float(r['z']) for r in guarded]),
        'max_original_Z_acceleration_m_s2': max_z_accel(z),
        'max_guarded_Z_acceleration_m_s2': max_z_accel([float(r['z']) for r in guarded]),
        'start_end_pose_unchanged': True,
        'XY_and_quaternion_unchanged': True,
        'insert_rotate_handoff_mm': handoff_mm,
        'is_collision_validated': False,
        'is_execution_approved': False,
    }
    with report.open('x') as fh:
        json.dump(summary, fh, indent=2)

    print('\nV1 INSERT DEPTH GUARD — OFFLINE ONLY')
    print(f'Points: {len(rows)}')
    print(f'Lowest demonstrated height: {floor*1000:.3f} mm')
    print(f'Original learned minimum Z: {min(z)*1000:.3f} mm')
    print(f'Depth-guarded minimum Z: {summary["guarded_min_Z_mm"]:.3f} mm')
    print(f'Maximum Z correction: {max(changes):.3f} mm over {summary["changed_points"]} points')
    print(f'Max consecutive Z step original/guarded: {summary["max_original_Z_step_mm"]:.3f}/{summary["max_guarded_Z_step_mm"]:.3f} mm')
    print(f'Max Z acceleration original/guarded: {summary["max_original_Z_acceleration_m_s2"]:.4f}/{summary["max_guarded_Z_acceleration_m_s2"]:.4f} m/s^2')
    print(f'INSERT -> ROTATE handoff: {handoff_mm:.4f} mm')
    print('Full start/end pose and all XY/quaternions preserved.')
    print(f'Candidate: {target}')
    print(f'Report: {report}')
    print('NO ROS OR ROBOT MOVEMENT. NOT APPROVED FOR EXECUTION.')


if __name__ == '__main__':
    main()
