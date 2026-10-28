#!/usr/bin/env python3
"""Offline fidelity analysis of V1 transport demonstration vs CDMP rollout.

Errors are measured by normalized motion phase, not robot tracking error.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp


def read(path,mode):
    cols=(['time','tcp_x','tcp_y','tcp_z','tcp_qx','tcp_qy','tcp_qz','tcp_qw']
          if mode=='demo' else ['time','x','y','z','qx','qy','qz','qw'])
    with path.open(newline='') as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or any(k not in reader.fieldnames for k in cols):
            raise RuntimeError('Unexpected columns: '+str(path))
        a=np.array([[float(r[k]) for k in cols] for r in reader],dtype=float)
    if len(a)<20 or not np.isfinite(a).all() or np.min(np.diff(a[:,0]))<=0:
        raise RuntimeError('Invalid path: '+str(path))
    if np.max(np.abs(np.linalg.norm(a[:,4:8],axis=1)-1))>.01:
        raise RuntimeError('Bad quaternion: '+str(path))
    return a


def resample(a,phase):
    t=(a[:,0]-a[0,0])/(a[-1,0]-a[0,0])
    pp=np.column_stack([np.interp(phase,t,a[:,j]) for j in [1,2,3]])
    qq=Slerp(t,Rotation.from_quat(a[:,4:8]))(phase)
    return pp,qq


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project',type=Path,default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    args=ap.parse_args()
    root=args.project.expanduser().resolve()
    demo=root/'demonstrations/processed/v1_transport_001_motion.csv'
    out=root/'demonstrations/processed/phase3_transport'
    path=out/'v1_transport_cdmp_baseline_moveit.csv'
    metrics=out/'phase3_transport_comparison.json'
    plot1=out/'phase3_transport_xyz_comparison.png'
    plot2=out/'phase3_transport_position_error.png'
    if any(p.exists() for p in (metrics,plot1,plot2)):
        raise SystemExit('Comparison outputs exist; refusing to overwrite')
    a=read(demo,'demo'); b=read(path,'path')
    phase=np.linspace(0,1,301)
    ap,aq=resample(a,phase); bp,bq=resample(b,phase)
    errors=np.linalg.norm(ap-bp,axis=1)*1000
    rot_err=np.degrees((aq.inv()*bq).magnitude())
    result={
        'metric_type':'phase_normalized_offline_DMP_fit_NOT_same_time_tracking',
        'demo_duration_s':float(a[-1,0]-a[0,0]),
        'cdmp_duration_s':float(b[-1,0]-b[0,0]),
        'rms_position_path_difference_mm':float(np.sqrt(np.mean(errors*errors))),
        'max_position_path_difference_mm':float(np.max(errors)),
        'rms_orientation_path_difference_deg':float(np.sqrt(np.mean(rot_err*rot_err))),
        'max_orientation_path_difference_deg':float(np.max(rot_err)),
        'endpoint_position_difference_mm':float(errors[-1]),
        'endpoint_orientation_difference_deg':float(rot_err[-1]),
        'note':'Time scale differs by design; controller execution timing must be measured separately.',
    }
    out.mkdir(parents=True,exist_ok=True)
    with metrics.open('x') as f:json.dump(result,f,indent=2)
    fig,axs=plt.subplots(3,1,figsize=(8,8),sharex=True,constrained_layout=True)
    for k,dim in enumerate('XYZ'):
        axs[k].plot(phase,ap[:,k]*1000,label='Human transport demo')
        axs[k].plot(phase,bp[:,k]*1000,linestyle='--',label='Learned CDMP')
        axs[k].set_ylabel(dim+' (mm, base)')
    axs[-1].set_xlabel('Normalized motion phase')
    axs[0].legend()
    fig.suptitle('V1 Transport: human demonstration vs learned CDMP')
    fig.savefig(plot1,dpi=160)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4.5))
    ax.plot(phase,errors)
    ax.set(xlabel='Normalized motion phase',ylabel='Cartesian position difference (mm)',
           title='V1 Transport: phase-matched offline path difference')
    fig.tight_layout(); fig.savefig(plot2,dpi=160); plt.close(fig)
    print('\nPHASE 3 TRANSPORT CDMP FIDELITY (OFFLINE, NOT ROBOT TRACKING)')
    print('Phase-matched path RMS position difference:',round(result['rms_position_path_difference_mm'],3),'mm')
    print('Max phase-matched position difference:',round(result['max_position_path_difference_mm'],3),'mm')
    print('Goal difference:',round(result['endpoint_position_difference_mm'],4),'mm /',
          round(result['endpoint_orientation_difference_deg'],4),'deg')
    print('Saved report and charts under',out)

if __name__=='__main__':main()
