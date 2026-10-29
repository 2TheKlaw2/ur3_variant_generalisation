"""UR3e + RG2 forward kinematics from the lab calibration, and a local IK.

numpy only (no ROS). FK reproduces all recorded `gripper_tcp` poses in
config/saved_poses within 0.03 mm (see results/scene_check/scene_check.txt).
Chain: base -> base_link_inertia -> calibrated joints -> flange -> tool0
       -> onrobot_base_link (yaw -pi/2) -> gripper_tcp (+0.218 m z).
"""
import math

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T(xyz=(0, 0, 0), r=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = _rpy(*r); M[:3, 3] = xyz; return M


def axis_angle(k, a):
    k = np.asarray(k, float); k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    M = np.eye(4); M[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; return M


def ur_fk(q, calib):
    """Return (T_base_rg2base, T_base_gripper_tcp) for joint vector q (6) and calibration dict."""
    k = calib["kinematics"]
    M = T(r=(0, 0, math.pi))
    for name, qi in zip(["shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"], q):
        e = k[name]
        M = M @ T((e["x"], e["y"], e["z"]), (e["roll"], e["pitch"], e["yaw"])) @ axis_angle([0, 0, 1.0], qi)
    M = M @ T(r=(0, -math.pi / 2, -math.pi / 2)) @ T(r=(math.pi / 2, 0, math.pi / 2))
    tool0 = np.linalg.inv(T(r=(0, 0, math.pi))) @ M
    rg2_base = tool0 @ T(r=(0, 0, -math.pi / 2))
    return rg2_base, rg2_base @ T((0, 0, 0.218))


def tcp(q, calib):
    return ur_fk(q, calib)[1]


def _err(target, Tc):
    e_p = target[:3, 3] - Tc[:3, 3]
    Re = target[:3, :3] @ Tc[:3, :3].T
    e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return np.r_[e_p, e_r]


def ik_local(q_seed, target, calib, iters=60, tol=1e-9):
    """Damped-least-squares IK started at q_seed (stays on the seed's branch for small steps)."""
    q = np.array(q_seed, float)
    for _ in range(iters):
        Tc = tcp(q, calib); e = _err(target, Tc)
        if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < tol:
            return q
        J = np.zeros((6, 6)); h = 1e-7
        for j in range(6):
            dq = q.copy(); dq[j] += h
            J[:, j] = (_err(target, Tc) - _err(target, tcp(dq, calib))) / h
        q += J.T @ np.linalg.solve(J @ J.T + 1e-10 * np.eye(6), e)
    e = _err(target, tcp(q, calib))
    if np.linalg.norm(e[:3]) > 1e-6 or np.linalg.norm(e[3:]) > 1e-6:
        raise RuntimeError(f"IK did not converge (pos err {np.linalg.norm(e[:3])*1000:.4f} mm)")
    return q
