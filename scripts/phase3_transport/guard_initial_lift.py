#!/usr/bin/env python3
"""OFFLINE ONLY: create a candidate V1 transport CDMP with a monotonic initial Z lift.

Replaces only the initial TCP Z samples up to a matching ascending +50mm lift
with a C1-continuous monotone cubic Hermite profile. Other fields, sample times,
and the remainder of the learned Cartesian CDMP are unchanged. This is an
engineered geometry constraint, not a learned-CDMP improvement or collision proof.
"""
from pathlib import Path
import csv
import json
import math

ROOT = Path.home() / 'ros2_ws/src/ur3_variant_generalisation'
DIR = ROOT / 'demonstrations/processed/phase3_transport'
SRC = DIR / 'v1_transport_cdmp_baseline_moveit.csv'
DST = DIR / 'v1_transport_cdmp_initial_lift_guard_CANDIDATE.csv'
META = DIR / 'v1_transport_cdmp_initial_lift_guard_CANDIDATE.json'
COLUMNS = ['time','x','y','z','qx','qy','qz','qw']

if DST.exists() or META.exists():
    raise SystemExit('Candidate already exists. Existing data preserved; inspect before overwriting.')
with SRC.open(newline='') as f:
    reader = csv.DictReader(f)
    if reader.fieldnames != COLUMNS:
        raise SystemExit(f'Unexpected CSV header: {reader.fieldnames!r}')
    raw = list(reader)
if len(raw) < 100:
    raise SystemExit('Insufficient trajectory samples')
vals = [[float(r[k]) for k in COLUMNS] for r in raw]
if not all(math.isfinite(v) for row in vals for v in row):
    raise SystemExit('Nonfinite trajectory value')
times = [r[0] for r in vals]
if any(b <= a for a,b in zip(times,times[1:])):
    raise SystemExit('Invalid trajectory timestamps')
zs = [r[3] for r in vals]
z0 = zs[0]
low_i = min(range(len(zs)), key=zs.__getitem__)
if not (zs[low_i] < z0 - .0005 and times[low_i]-times[0] < 3.0):
    raise SystemExit('Expected initial CDMP downward dip not found; inspect source manually.')

# Select an early rising rejoin point. For a cubic Hermite prefix with
# initial derivative zero and final derivative matching the CDMP, the
# monotonicity condition 0 <= m_end <= 3 * average_slope must hold.
join = None
for i in range(max(low_i + 2,2), len(vals)-1):
    if times[i]-times[0] > 6.0:
        break
    if zs[i]-z0 < .050:  # preserve CDMP after first +50mm lift
        continue
    d_end = (zs[i+1]-zs[i-1]) / (times[i+1]-times[i-1])
    slope = (zs[i]-z0) / (times[i]-times[0])
    if 0 <= d_end <= 3*slope:
        join, m_end = i, d_end
        break
if join is None:
    raise SystemExit('Could not find a safe monotone C1 splice within first 6 s; no output written.')
T = times[join]-times[0]
Z = zs[join]
new_z = zs.copy()
for i in range(join):
    u = (times[i]-times[0])/T
    # cubic Hermite: start z0, slope 0; end Z, slope m_end
    new_z[i] = (2*u**3-3*u**2+1)*z0 + (-2*u**3+3*u**2)*Z + (u**3-u**2)*T*m_end

if any(new_z[i+1] < new_z[i]-1e-10 for i in range(join)):
    raise SystemExit('New initial lift is not monotone; no output written')
if min(new_z) < z0 - .0001:
    raise SystemExit('Corrected trajectory still drops below starting height; no output written')
if any(abs(a-b)>1e-12 for a,b in zip(new_z[join:],zs[join:])):
    raise SystemExit('Rejoin segment unexpectedly modified')

# Check clearance proxy at key horizontal displacement thresholds.
xy0 = vals[0][1:3]
clearance = {}
for mm in [5,10,20,30,50]:
    index = next((i for i,r in enumerate(vals)
                  if math.hypot(r[1]-xy0[0],r[2]-xy0[1])*1000 >= mm), None)
    if index is not None:
        clearance[str(mm)] = round((new_z[index]-z0)*1000,3)

change_mm = max(abs(a-b) for a,b in zip(zs,new_z))*1000
max_z_step_mm = max(abs(b-a) for a,b in zip(new_z,new_z[1:]))*1000
step_before_mm = max(abs(b-a) for a,b in zip(zs,zs[1:]))*1000
meta = {
    'source': str(SRC), 'candidate': str(DST),
    'type': 'C1 monotone geometry-constrained initial lift + original learned CDMP suffix',
    'training': 'unchanged', 'robot_execution_authorised': False,
    'original_min_z_mm': round(min(zs)*1000,3),
    'candidate_min_z_mm': round(min(new_z)*1000,3),
    'start_z_mm': round(z0*1000,3),
    'join_index': join, 'join_time_s': round(times[join]-times[0],3),
    'join_lift_mm': round((Z-z0)*1000,3),
    'max_z_correction_mm': round(change_mm,3),
    'original_max_z_step_mm': round(step_before_mm,3),
    'candidate_max_z_step_mm': round(max_z_step_mm,3),
    'vertical_lift_mm_at_xy_displacement_mm': clearance,
    'limitations': [
        'TCP clearance is NOT component/fixture clearance',
        'Only Cartesian Z was corrected; IK, joint continuity, dynamics and collision model not yet validated',
        'Trajectory is a hybrid: learned CDMP with a geometry-constrained initial lift',
        'A separate small start-pose mismatch to actual saved grip remains and needs handling',
    ],
}
# Preserve exact original CSV numeric strings in every column except Z of prefix.
with DST.open('x',newline='') as f:
    wr = csv.DictWriter(f, fieldnames=COLUMNS)
    wr.writeheader()
    for i,r in enumerate(raw):
        out = r.copy()
        if i < join:
            out['z'] = f'{new_z[i]:.12f}'
        wr.writerow(out)
with META.open('x') as f:
    json.dump(meta,f,indent=2)
print('\nV1 TRANSPORT INITIAL LIFT — OFFLINE CANDIDATE')
print(f'Original early dip: {meta["original_min_z_mm"]:.3f} mm')
print(f'Candidate minimum: {meta["candidate_min_z_mm"]:.3f} mm (start {meta["start_z_mm"]:.3f} mm)')
print(f'Rejoin original learned CDMP at {meta["join_time_s"]:.3f} s, +{meta["join_lift_mm"]:.3f} mm rise')
print(f'Maximum Z modification: {meta["max_z_correction_mm"]:.3f} mm')
print(f'Max Z step original/candidate: {step_before_mm:.3f}/{max_z_step_mm:.3f} mm per sample')
for xy,rise in clearance.items():
    print(f'XY displacement {xy:>2s} mm => candidate TCP rise {rise:7.3f} mm')
print('Start and end pose preserved. All other Cartesian columns preserved.')
print(f'Candidate saved: {DST}')
print(f'Report: {META}')
print('NO ROBOT COMMANDS. NOT APPROVED FOR EXECUTION.')
