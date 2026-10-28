#!/usr/bin/env python3
"""Offline math-only unit tests; no robot and no CartesianDMPs dependency."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

SOURCE=Path(__file__).with_name('generate_phase2_cdmp.py')
spec=importlib.util.spec_from_file_location('phase2_math',SOURCE)
impl=importlib.util.module_from_spec(spec)
spec.loader.exec_module(impl)


class GoalMathTests(unittest.TestCase):
    def test_order_conversion(self):
        q=np.array([0.1,0.2,0.3,0.9],dtype=float)
        np.testing.assert_allclose(impl.wxyz_to_xyzw(impl.xyzw_to_wxyz(q)),q)

    def test_quaternion_sign_symmetry(self):
        q=Rotation.from_euler('xyz',[25,-10,14],degrees=True).as_quat()
        self.assertLess(impl.quat_angle_deg(q,-q),1e-5)

    def test_relative_target_rotation_order(self):
        source=Rotation.from_euler('xyz',[30,-40,12],degrees=True)
        target=Rotation.from_euler('xyz',[-15,20,55],degrees=True)
        relative=target*source.inv()
        reconstructed=relative*source
        self.assertAlmostEqual((reconstructed.inv()*target).magnitude(),0,places=10)

    def test_saved_pose_fields_and_frames(self):
        import yaml
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'grip.yaml'
            d={'base_frame':'base','tool_frame':'gripper_tcp',
               'tcp':{'position_m':[.2,-.3,.04],'quaternion_xyzw':[0,0,0,1]}}
            path.write_text(yaml.safe_dump(d))
            p,q=impl.read_grip(path)
            self.assertEqual(len(p),3)
            self.assertEqual(len(q),4)
            d['tool_frame']='tool0'
            path.write_text(yaml.safe_dump(d))
            with self.assertRaisesRegex(RuntimeError,'same|gripper_tcp|Saved grip pose'):
                impl.read_grip(path)


if __name__=='__main__':
    unittest.main(verbosity=2)
