#!/usr/bin/env python3
"""Offline constrained smoothing of the V1 INSERT depth-guard candidate.

Keeps the refined learned Cartesian CDMP unchanged, creates a NEW joint-planning
candidate with the smallest useful Z-only nonnegative smooth correction.
Solves a bounded quadratic smoothness objective using scipy L-BFGS-B.
No ROS imports, no robot actions, no execution approval.
"""
from __future__ import annotations
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

FIELDS = ['time','x','y','z','qx','qy','qz','qw']
REF_FIELDS = ['time','tcp_x','tcp_y','tcp_z','tcp_qx','tcp_qy','tcp_qz','tcp_qw']

def read_csv(p, fields):
    with p.open(newline='') as f:
        rdr=csv.DictReader(f)
        if rdr.fieldnames != fields:
            raise SystemExit(f'BLOCKED: Wrong CSV headings in {p}: {rdr.fieldnames}')
        rows=list(rdr)
    if len(rows)<20:
        raise SystemExit(f'BLOCKED: Too few rows: {p}')
    return rows

def derivative_metrics(z_m,t):
    v=np.diff(z_m)/np.diff(t)
    a=np.diff(v)/(0.5*(t[2:]-t[:-2]))
    jerk=np.diff(a)/(0.5*(t[3:]-t[1:-2])) if len(t)>3 else np.array([0.0])
    return {'max_z_step_mm':float(np.abs(np.diff(z_m)).max()*1000),
            'max_abs_v_m_s':float(np.abs(v).max()),
            'max_abs_a_m_s2':float(np.abs(a).max()),
            'max_abs_jerk_m_s3':float(np.abs(jerk).max())}

def optimize_correction(z_mm, floor_mm, lam):
    n=len(z_mm)
    lower=np.maximum(0.0, floor_mm-z_mm)
    if lower[0]>1e-8 or lower[-1]>1e-8:
        raise SystemExit('BLOCKED: start/end below depth floor')
    lower[0]=lower[-1]=0.0
    upper=np.full(n,0.50)
    upper[0]=upper[-1]=0.0
    # y=z+c; penalize both magnitude and curvature of the correction.
    # No time shifts or edits to XY/quaternion. Corrections must be nonnegative.
    def objective(c):
        d2=np.diff(c,n=2)
        value=0.5*np.dot(c,c)+0.5*lam*np.dot(d2,d2)
        gradient=c.copy()
        d=lam*d2
        gradient[:-2]+=d
        gradient[1:-1]-=2*d
        gradient[2:]+=d
        return value,gradient
    bounds=list(zip(lower,upper))
    solution=minimize(objective,np.maximum(lower,0),method='L-BFGS-B',jac=True,
                      bounds=bounds,
                      options={'maxiter':5500,'ftol':1e-12,'gtol':1e-8,'maxls':40})
    c=solution.x
    violation=max(float((lower-c).max()),float((c-upper).max()))
    return c,solution,violation

def main():
    root=Path(__file__).resolve().parents[2]
    folder=root/'demonstrations/processed/phase4_assembly'
    src=folder/'v1_insert_cdmp_refined_CANDIDATE.csv'
    ref=folder/'v1_insert_refined_tcp_training.csv'
    existing=folder/'v1_insert_cdmp_refined_depth_guard_CANDIDATE.csv'
    out=folder/'v1_insert_cdmp_refined_depth_guard_SMOOTH_CANDIDATE.csv'
    report=folder/'v1_insert_smooth_depth_guard_report.json'
    for p in (src,ref,existing):
        if not p.exists(): raise SystemExit(f'BLOCKED: Missing required file: {p}')
    for p in (out,report):
        if p.exists(): raise SystemExit(f'BLOCKED: Output already exists, refusing overwrite: {p}')
    rows=read_csv(src,FIELDS)
    reference=read_csv(ref,REF_FIELDS)
    old_guard=read_csv(existing,FIELDS)
    if len(old_guard)!=len(rows): raise SystemExit('BLOCKED: Existing guard has wrong length')
    z=np.array([float(r['z']) for r in rows]); t=np.array([float(r['time']) for r in rows]);
    floor=min(float(r['tcp_z']) for r in reference)
    if not np.isfinite(z).all() or not np.isfinite(t).all() or not (np.diff(t)>0).all():
        raise SystemExit('BLOCKED: Nonfinite or non-increasing trajectory data')
    if not (0.20e-3 < floor-z.min() < 0.40e-3):
        raise SystemExit(f'BLOCKED: Unexpected depth violation {(floor-z.min())*1000:.4f} mm')
    if abs(t[-1]-12.998915308) > 0.05 or len(rows)!=651:
        raise SystemExit('BLOCKED: Unexpected INSERT rollout duration / sample count')
    before=derivative_metrics(z,t)
    limit=min(0.03,1.5*before['max_abs_a_m_s2'])
    zmm=z*1000; floormm=floor*1000
    print('\nV1 SMOOTH INSERT DEPTH GUARD — OFFLINE ONLY')
    print(f'Raw refined CDMP min Z: {zmm.min():.4f} mm; recorded floor: {floormm:.4f} mm')
    print(f'Original max |Z acceleration|: {before["max_abs_a_m_s2"]:.4f} m/s^2')
    print(f'Acceptance limit for trial candidate: {limit:.4f} m/s^2')
    picked=None
    trials=[]
    for lam in (2e4,2e5,2e6,2e7):
        c,sol,violation=optimize_correction(zmm,floormm,lam)
        modified=z+c/1000
        metrics=derivative_metrics(modified,t)
        maxcorr=float(c.max())
        minmargin=float((modified-floor).min()*1000)
        record={'curvature_weight':lam,'optimizer_success':bool(sol.success),
                'optimizer_message':str(sol.message), 'violation_mm':violation,
                'max_correction_mm':maxcorr,'minimum_margin_mm':minmargin,
                **metrics}
        trials.append(record)
        print(f'  lambda={lam:.0e}: acceleration={metrics["max_abs_a_m_s2"]:.4f} m/s^2; '
              f'max correction={maxcorr:.4f} mm; depth margin={minmargin:.6f} mm; '
              f'optimizer={"OK" if sol.success else "NOT CONVERGED"}')
        if (violation<2e-6 and minmargin>=-2e-6 and sol.success and
            metrics['max_abs_a_m_s2']<=limit and
            metrics['max_abs_v_m_s']<=1.5*before['max_abs_v_m_s'] and maxcorr<=0.50):
            picked=(modified,record)
            break
    if picked is None:
        print('\nNOT ACCEPTED: Could not achieve a smooth feasible depth guard.')
        print('No trajectory written. Keep original and guarded candidates unchanged.')
        print('Please share the diagnostic table; do not plan/execute insertion.')
        raise SystemExit(2)
    modified,metrics=picked
    if abs(modified[0]-z[0])>1e-10 or abs(modified[-1]-z[-1])>1e-10:
        raise SystemExit('BLOCKED: start or end pose changed')
    if (modified<floor-1e-9).any():
        raise SystemExit('BLOCKED: depth floor violated')
    if max(abs(modified-z))>0.0005+1e-9:
        raise SystemExit('BLOCKED: correction >0.5mm')
    with out.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS); w.writeheader()
        for row,zz in zip(rows,modified):
            r=row.copy(); r['z']=f'{zz:.12f}'; w.writerow(r)
    data={'method':'Time-domain bounded curvature-minimizing positive vertical correction; NOT DMP learning',
          'original_source':str(src),'original_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
          'floor_source':str(ref),'floor_m':float(floor),
          'before':before,'accepted':metrics,'trials':trials,
          'start_end_preserved':True,'XY_and_orientation_preserved':True,
          'collision_validated':False,'execution_approved':False}
    with report.open('x') as f: json.dump(data,f,indent=2)
    print('\nOFFLINE DEPTH GUARD: CANDIDATE GENERATED (NOT EXECUTION APPROVAL)')
    print(f'Minimum Z: {min(modified)*1000:.4f} mm')
    print(f'Max correction: {metrics["max_correction_mm"]:.4f} mm')
    print(f'Old/New acceleration: {before["max_abs_a_m_s2"]:.4f} / {metrics["max_abs_a_m_s2"]:.4f} m/s^2')
    print(f'Saved: {out}')
    print(f'Report: {report}')
    print('No ROS connection or robot movement. Physical clearance still unverified.')

if __name__=='__main__':main()
