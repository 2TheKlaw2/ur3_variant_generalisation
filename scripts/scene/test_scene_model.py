#!/usr/bin/env python3
"""Check scene_model.py against the expanded scene URDF and the CAD meshes.

  python3 scripts/scene/test_scene_model.py [--urdf expanded_scene.urdf]
Offline; needs numpy, pyyaml (and the xacro python module if --urdf is omitted).
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import scene_model as sm  # noqa: E402


def urdf_poses(xml):
    root = ET.fromstring(xml)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        r = [float(v) for v in o.get("rpy", "0 0 0").split()]
        M = sm.translate(xyz); M[:3, :3] = sm.rpy_matrix(*r)
        joints[j.find("child").get("link")] = (j.find("parent").get("link"), M)
    memo = {}

    def pose(l):
        if l not in joints: return np.eye(4)
        if l not in memo:
            p, M = joints[l]; memo[l] = pose(p) @ M
        return memo[l]
    return {l: pose(l) for l in joints}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--urdf"); a = ap.parse_args()
    if a.urdf:
        xml = open(a.urdf).read()
    else:
        import xacro
        xml = xacro.process_file(os.path.join(sm.PKG, "urdf", "variant_scene.urdf.xacro")).toprettyxml()
    ref = urdf_poses(xml)
    objs = sm.scene_objects()
    fails = 0
    for name, o in objs.items():
        err_t = np.linalg.norm(o["T"][:3, 3] - ref[name][:3, 3]) * 1000
        err_r = np.degrees(np.arccos(np.clip((np.trace(o["T"][:3, :3].T @ ref[name][:3, :3]) - 1) / 2, -1, 1)))
        v, f = sm.read_stl(o["mesh"])
        ok = err_t < 1e-6 and err_r < 1e-6 and len(f) > 0
        fails += not ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:16s} pose vs URDF {err_t:.2e} mm / {err_r:.2e} deg; "
              f"mesh {len(f)} tris, extents {np.round((v.max(0)-v.min(0))*1000, 2).tolist()} mm")
    q = sm.matrix_to_quat(objs["pickup_V3"]["T"][:3, :3])
    ok = np.allclose(sm.quat_to_matrix(q), objs["pickup_V3"]["T"][:3, :3], atol=1e-12)
    fails += not ok
    print(f"[{'PASS' if ok else 'FAIL'}] quaternion round trip")
    print("ALL PASS" if not fails else f"{fails} FAIL")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
