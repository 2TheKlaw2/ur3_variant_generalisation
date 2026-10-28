#!/usr/bin/env python3
"""Pure offline Phase 2 evaluation: goal errors, normalized shapes and timing."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

TRIALS = ['baseline_tau2','adapted_tau2','adapted_tau1','adapted_tau3']


def read(path):
    with path.open(newline='') as f:
        rd = csv.DictReader(f)
        if rd.fieldnames != ['time','x','y','z','qx','qy','qz','qw']:
            raise RuntimeError('Unexpected Cartesian CSV columns: '+str(path))
        rows = np.array([[float(row[k]) for k in rd.fieldnames] for row in rd], dtype=float)
    if len(rows) < 10 or not np.isfinite(rows).all() or np.any(np.diff(rows[:,0]) <= 0):
        raise RuntimeError('Invalid path: '+str(path))
    quat_len = np.linalg.norm(rows[:, 4:8],axis=1)
    if np.max(abs(quat_len-1)) > 0.01:
        raise RuntimeError('Nonunit orientations: '+str(path))
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path,
                   default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    a=p.parse_args()
    root=a.project.expanduser().resolve()
    output=root/'demonstrations/processed/phase2_reach'
    manifest_file=output/'phase2_manifest.json'
    with manifest_file.open() as f:
        manifest=json.load(f)
    data={name: read(output/f'v1_reach_{name}_moveit.csv') for name in TRIALS}
    goal_by_name={trial['trial']: trial for trial in manifest['rollouts']}
    baseline=data['baseline_tau2']
    adapted=data['adapted_tau2']
    u=np.linspace(0, 1, 251)
    def interp_xyz(arr):
        s=(arr[:,0]-arr[0,0])/(arr[-1,0]-arr[0,0])
        return np.column_stack([np.interp(u,s,arr[:,axis]) for axis in (1,2,3)])
    baseline_shape=interp_xyz(baseline)
    adapted_shape=interp_xyz(adapted)
    diff_mm=np.linalg.norm(adapted_shape-baseline_shape,axis=1)*1000
    speed_shapes=[]
    for name in ('adapted_tau1','adapted_tau2','adapted_tau3'):
        speed_shapes.append(interp_xyz(data[name]))
    duration={name:float(arr[-1,0]-arr[0,0]) for name,arr in data.items()}
    time_shape_delta_mm=max(float(np.max(np.linalg.norm(s-speed_shapes[1],axis=1)*1000)) for s in speed_shapes)

    # Numerical summary (no claims about physical execution).
    result={
        'goal_offset_mm':manifest['translation_goal_offset_mm'],
        'goal_orientation_offset_deg':manifest['orientation_goal_offset_deg'],
        'durations_s':duration,
        'baseline_to_adapted_phase_matched_path_distance_mm': {
            'start':float(diff_mm[0]), 'middle':float(diff_mm[len(diff_mm)//2]),
            'end':float(diff_mm[-1]), 'maximum':float(np.max(diff_mm)),
        },
        'max_phase_matched_position_difference_across_tau_settings_mm':time_shape_delta_mm,
        'trial_goal_errors': {name:{
            'position_mm':goal_by_name[name]['goal_position_error_mm'],
            'orientation_deg':goal_by_name[name]['goal_orientation_error_deg'],
        } for name in TRIALS},
        'disclaimer':'This is offline learned-path comparison, not robot tracking/collision validation.',
    }
    resfile=output/'phase2_comparison.json'
    figure=output/'phase2_path_comparison.png'
    figure2=output/'phase2_temporal_comparison.png'
    if any(x.exists() for x in (resfile,figure,figure2)):
        raise SystemExit('Comparison output exists; refusing overwrite')
    with resfile.open('x') as f:
        json.dump(result,f,indent=2)

    fig=plt.figure(figsize=(8.5,5.5))
    ax=fig.add_subplot(111,projection='3d')
    ax.plot(*(baseline_shape[:,i] for i in range(3)),label='Original CDMP goal, tau=2',lw=2)
    ax.plot(*(adapted_shape[:,i] for i in range(3)),label='Adapted grip goal, tau=2',lw=2,linestyle='--')
    ax.scatter(*baseline_shape[-1],label='Original endpoint',s=45)
    ax.scatter(*adapted_shape[-1],label='Grip target endpoint',s=45)
    ax.set(xlabel='base X (m)',ylabel='base Y (m)',zlabel='base Z (m)',title='V1 CDMP reach: taught goal versus adapted goal')
    ax.legend(loc='best',fontsize=8)
    fig.tight_layout()
    fig.savefig(figure,dpi=180)
    plt.close(fig)

    fig,axs=plt.subplots(2,1,figsize=(8.5,7),constrained_layout=True)
    for name in ('adapted_tau1','adapted_tau2','adapted_tau3'):
        arr=data[name]
        axs[0].plot(arr[:,0],arr[:,3],label=f'{name} ({duration[name]:.1f}s)')
    axs[0].set(xlabel='CDMP time (s)',ylabel='TCP Z in base (m)',title='CDMP temporal adaptation: absolute time')
    axs[0].legend()
    for name,shape in zip(('tau1','tau2','tau3'),speed_shapes):
        axs[1].plot(u,shape[:,2],label=name)
    axs[1].set(xlabel='Normalized phase (0–1)',ylabel='TCP Z in base (m)',title='Same motion compared by normalized phase')
    axs[1].legend()
    fig.savefig(figure2,dpi=180)
    plt.close(fig)
    print('\nPHASE 2 OFFLINE COMPARISON')
    print('Goal translation offset (mm):',np.round(manifest['translation_goal_offset_mm'],3))
    print('Goal orientation offset (deg):',round(manifest['orientation_goal_offset_deg'],3))
    print('CDMP durations:', {name:round(v,3) for name,v in duration.items()})
    print('Distance original vs adapted endpoint (mm):',round(float(diff_mm[-1]),4))
    print('Max Cartesian shape change across time scales (mm):',round(time_shape_delta_mm,4))
    print('Saved analysis and figures to:',output)
    print('No robot interaction.')


if __name__=='__main__':
    main()
