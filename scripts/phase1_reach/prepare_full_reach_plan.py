#!/usr/bin/env python3
"""Produce a full-V1-reach PLAN-ONLY parameter file from fake-hardware URDF.

Works on the Ubuntu ROS 2 Humble workstation; no ROS nodes are created and
no trajectory is published or executed by this script.
"""
import argparse
import copy
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path,
                   default=Path.home()/'ros2_ws/src/ur3_variant_generalisation')
    p.add_argument('--output', type=Path,
                   default=Path('/tmp/v1_phase1_FULL_REACH_PLAN_ONLY.yaml'))
    a = p.parse_args()

    # Deliberately import ROS packages only when run on the ROS workstation.
    import yaml
    import xacro
    from ament_index_python.packages import get_package_share_directory

    root = a.project.expanduser().resolve()
    motion = root/'demonstrations/processed/v1_reach_cdmp_moveit_001.csv'
    demo = root/'demonstrations/processed/v1_reach_001_motion.csv'
    calibration = root/'config/ur3e_lab_calibration.yaml'
    for source in (motion, demo, calibration):
        if not source.is_file():
            raise SystemExit(f'Required file missing: {source}')

    desc_share = Path(get_package_share_directory('ur_onrobot_description'))
    moveit_share = Path(get_package_share_directory('ur_onrobot_moveit_config'))
    urdf = xacro.process_file(str(desc_share/'urdf/ur_onrobot.urdf.xacro'), mappings={
        'robot_ip': '127.0.0.1',
        'ur_type': 'ur3e',
        'onrobot_type': 'rg2',
        'name': 'ur_onrobot',
        'use_fake_hardware': 'true',
        'kinematics_parameters_file': str(calibration),
    }).toxml()
    srdf = xacro.process_file(str(moveit_share/'srdf/ur_onrobot.srdf.xacro'),
                              mappings={'name': 'ur_onrobot'}).toxml()
    kinematics = yaml.safe_load((moveit_share/'config/kinematics.yaml').read_text())
    if 'ur_onrobot_manipulator' not in kinematics:
        raise SystemExit('The expected manipulator kinematics were not found')

    params = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': copy.deepcopy(kinematics),
        'moveit.path_file': str(motion),
        'moveit.source_demo': str(demo),
        'moveit.group_name': 'ur_onrobot_manipulator',
        'moveit.reference_frame': 'base',
        'moveit.end_effector_link': 'gripper_tcp',
        'moveit.eef_step': 0.005,
        'moveit.waypoint_stride': 10,
        'moveit.min_fraction': 1.0,
        'moveit.velocity_scale': 0.05,
        'moveit.acceleration_scale': 0.05,
        'moveit.avoid_collisions': True,
        'moveit.collision_diagnostic': False,
        'moveit.execute': False,
        'use_sim_time': False,
    }
    if a.output.exists():
        raise SystemExit(f'Output already exists, refusing to overwrite: {a.output}')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x') as handle:
        yaml.safe_dump({'/execute_cartesian_path_moveit': {
            'ros__parameters': params}}, handle, sort_keys=False)
    print(f'PLAN-ONLY parameters created: {a.output}')
    print('MoveIt execution disabled; no robot action sent.')
    print('Note: planning scene still needs trolley and plates as collision objects.')


if __name__ == '__main__':
    main()
