"""Shared scene geometry: component poses in `base` and minimal mesh I/O.

Single source of truth = cad/manifest.yaml (poses in plate frames, mm)
+ config/scene/scene_registration.yaml (plate -> base, PROVISIONAL).
Matches urdf/variant_scene.urdf.xacro exactly (checked by test_scene_model.py).
No ROS imports here, so it can be used offline and inside ROS nodes.
"""
import math
import os
import struct

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _load(rel):
    with open(os.path.join(PKG, rel)) as f:
        return yaml.safe_load(f)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def translate(xyz):
    M = np.eye(4); M[:3, 3] = xyz; return M


def matrix_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_to_matrix(xyz, quat_xyzw):
    M = np.eye(4); M[:3, :3] = quat_to_matrix(quat_xyzw); M[:3, 3] = xyz; return M


def plate_frames():
    reg = _load("config/scene/scene_registration.yaml")
    out = {}
    for key in ("pickup_plate", "placement_plate"):
        r = reg[key]
        out[key] = translate(r["xyz_m"]) @ rot_z(r["yaw_rad"])
    return out


def scene_objects():
    """Return {object_id: {"T": 4x4 base pose [m], "mesh": abs path, "plate": key}}.

    Object ids match the URDF link names: pickup_plate, placement_plate,
    pickup_V1..V5, fitting_V1..V5, and the environment boxes fixture_plate, trolley_top.
    """
    man = _load("cad/manifest.yaml")["meshes"]
    frames = plate_frames()
    objs = {}
    for key in ("pickup_plate", "placement_plate"):
        objs[key] = {"T": frames[key], "mesh": os.path.join(PKG, "cad", man[key]["file"]), "plate": key}
    for v in range(1, 6):
        for kind, mkey, pkey, plate in (("pickup", f"V{v}_moving", "pose_in_pickup_plate", "pickup_plate"),
                                         ("fitting", f"V{v}_fitting", "pose_in_placement_plate", "placement_plate")):
            p = man[mkey][pkey]
            local = translate(np.array(p["xyz_mm"]) / 1000.0)
            local[:3, :3] = rpy_matrix(*[math.radians(a) for a in p["rpy_deg"]])
            objs[f"{kind}_V{v}"] = {"T": frames[plate] @ local,
                                    "mesh": os.path.join(PKG, "cad", man[mkey]["file"]), "plate": plate}
    for key, e in _load("config/scene/scene_registration.yaml").get("environment", {}).items():
        objs[key] = {"T": translate(e["xyz_m"]) @ rot_z(e["yaw_rad"]),
                     "mesh": os.path.join(PKG, "cad", "environment", f"{key}.stl"), "plate": "environment"}
    return objs


def read_stl(path, scale=0.001):
    """Read a binary (or ASCII) STL -> (vertices Nx3 float64 [m], faces Mx3 int)."""
    with open(path, "rb") as f:
        data = f.read()
    tris = None
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
            tris = rec["v"].astype(np.float64)
    if tris is None:  # ASCII fallback
        vals = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
                if ln.strip().startswith("vertex")]
        tris = np.array(vals, float).reshape(-1, 3, 3)
    flat = tris.reshape(-1, 3) * scale
    verts, inv = np.unique(np.round(flat, 9), axis=0, return_inverse=True)
    return verts, inv.reshape(-1, 3).astype(np.int64)


# Physical/simulation settings shared by pickup scripts
RG2_OPEN_M = 0.040     # physical V1 pilot setting (run_v1_physical_sequence.py)
RG2_GRIP_M = 0.0082
RG2_TOUCH_LINKS = ["gripper_tcp", "left_inner_finger", "right_inner_finger",
                   "left_inner_knuckle", "right_inner_knuckle",
                   "left_outer_knuckle", "right_outer_knuckle"]
POCKET_DEPTH_M = 0.002  # pickup pocket: 3 mm plate, 1 mm floor
