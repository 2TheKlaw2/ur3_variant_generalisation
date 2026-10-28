#!/usr/bin/env python3
"""Create fake-hardware PLAN-ONLY MoveIt YAMLs for Phase 2 CDMP rollouts."""
import argparse
import copy
import json
from pathlib import Path

import yaml

TRIALS=['baseline_tau2','adapted_tau2','adapted_tau1','adapted_tau3']


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project',type=Path,
                    default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    ap.add_argument('--template',type=Path,default=Path('/tmp/v1_phase1_FULL_REACH_PLAN_ONLY.yaml'))
    ap.add_argument('--out-dir',type=Path,default=Path('/tmp/v1_phase2_sim_plans'))
    a=ap.parse_args()
    root=a.project.expanduser().resolve()
    out=a.out_dir.expanduser().resolve()
    manifest=root/'demonstrations/processed/phase2_reach/phase2_manifest.json'
    if not manifest.exists():
        raise SystemExit('Run generate_phase2_cdmp.py first; manifest is absent')
    with manifest.open() as f:
        meta=json.load(f)
    if len(meta.get('rollouts',[]))!=4:
        raise SystemExit('Expected four generated CDMP trials')
    if not a.template.is_file():
        raise SystemExit('Run Phase 1 prepare_full_reach_plan.py first: '+str(a.template))
    with a.template.open() as f:
        template=yaml.safe_load(f)
    key='/execute_cartesian_path_moveit'
    try:
        params=template[key]['ros__parameters']
    except Exception as e:
        raise SystemExit('Bad Phase 1 parameter template') from e
    if params.get('moveit.execute') is not False:
        raise SystemExit('Unsafe template: MoveIt execution not explicitly false')
    if ('mock_components/GenericSystem' not in params.get('robot_description','') or
            params.get('moveit.group_name')!='ur_onrobot_manipulator'):
        raise SystemExit('Unsafe template: this is not the Phase 1 fake-hardware URDF and group')
    if params.get('moveit.source_demo') != str(root/'demonstrations/processed/v1_reach_001_motion.csv'):
        raise SystemExit('Phase 1 template source demonstration does not match expected V1 input')
    output_files=[out/f'{name}_PLAN_ONLY.yaml' for name in TRIALS]
    if any(x.exists() for x in output_files):
        raise SystemExit('Refusing to overwrite existing Phase 2 plan configurations')
    for name,path in zip(TRIALS,output_files):
        motion=root/f'demonstrations/processed/phase2_reach/v1_reach_{name}_moveit.csv'
        if not motion.is_file():
            raise SystemExit('Missing rollout '+str(motion))
    out.mkdir(parents=True,exist_ok=True)
    for name,path in zip(TRIALS,output_files):
        settings=copy.deepcopy(template)
        p=settings[key]['ros__parameters']
        p['moveit.path_file']=str(root/f'demonstrations/processed/phase2_reach/v1_reach_{name}_moveit.csv')
        p['moveit.execute']=False
        p['moveit.avoid_collisions']=True
        p['moveit.min_fraction']=1.0
        with path.open('x') as f:
            yaml.safe_dump(settings,f,sort_keys=False)
        print('PLAN ONLY:',path)
    print('Simulation domain: set ROS_DOMAIN_ID=73 in every ROS terminal.')
    print('Planning time is set by MoveIt; raw CDMP tau is not necessarily preserved.')
    print('NO ROBOT MOVEMENT.')


if __name__=='__main__':
    main()
