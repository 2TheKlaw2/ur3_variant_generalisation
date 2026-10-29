"""Stage 7 geometry/timing (pure numpy): raised BRIDGE and learned TRANSPORT tail.

Bridge geometry follows scripts/phase5_integration/run_v1_lift_bridge_transport_staged.py
(physically tested): from the lift end, move horizontally at lift height to above the
join row's xy, then straight to the join row pose of the learned transport. Join row =
first row (from 8, step 2) whose TCP is >= 31 mm above the grip TCP (physically: row 24).
TRANSPORT tail = learned rows from the join row onwards, original 0.1 s spacing x1.5.
Bridge timing in simulation is NEW: minimum-jerk per segment, 6.0 s total (the physical
bridge used MoveIt timing with a 6 s minimum).
"""
import csv
import hashlib
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase6_pickup_sim")); sys.path.insert(0, os.path.join(HERE, "..", "scene"))
import lift_profile as lp  # noqa: E402
import scene_model as sm  # noqa: E402
import ur_kinematics as uk  # noqa: E402

TRANSPORT_CSV = "demonstrations/processed/phase3_transport/v1_transport_joint_sim_candidate.csv"
TRANSPORT_SHA_PREFIX = "f4335b3ef675"     # as physical script
TRANSPORT_ROWS = 246
EXPECTED_JOIN_ROW = 24                    # as physical script
JOIN_TOL_RAD = 0.0055                     # as physical script
JOIN_MIN_RISE_M = lp.LIFT_M + 0.001       # >= 31 mm above grip TCP
BRIDGE_MIN_HEIGHT_M = lp.LIFT_M - 0.002   # bridge must not dip below 28 mm (physical check)
TRANSPORT_TIME_SCALE = 1.5                # physically tested
BRIDGE_SIM_DURATION_S = 6.0               # NEW sim timing (physical: MoveIt-timed, >= 6 s)
ROT_WEIGHT_M_PER_RAD = 0.05               # splits bridge time between segments (1 rad ~ 50 mm)
ALIGN_INSERT_CSV = "config/trajectories/v1/v1_align_to_insertion_joint_plan_001.csv"
ALIGN_INSERT_MULTIPLIER = 2.0             # as run_v1_integrated_fake.py
TAIL_SPEED_LIMIT = 0.14                   # physical verify_path guard (rad/s)


def load_transport(root):
    path = os.path.join(root, TRANSPORT_CSV)
    sha = hashlib.sha256(open(path, "rb").read()).hexdigest()
    if not sha.startswith(TRANSPORT_SHA_PREFIX):
        raise RuntimeError("TRANSPORT file hash changed; refusing to plan from an unknown path")
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        if r.fieldnames != ["time"] + lp.JOINTS:
            raise RuntimeError("Transport CSV header changed")
        rows = list(r)
    if len(rows) != TRANSPORT_ROWS:
        raise RuntimeError("Transport CSV sample count changed")
    t = np.array([float(x["time"]) for x in rows])
    q = np.array([[float(x[j]) for j in lp.JOINTS] for x in rows])
    return sha, t, q


def find_join_row(q_transport, z_grip, calib):
    rises = [(i, uk.tcp(q_transport[i], calib)[2, 3] - z_grip) for i in range(8, len(q_transport), 2)]
    for i, dz in rises:
        if dz >= JOIN_MIN_RISE_M:
            return i, dz, max(d for _, d in rises)
    raise RuntimeError("No learned transport row >= 31 mm above pickup")


def slerp(q0, q1, s):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0: q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + s * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)


def interp_pose(A, B, s):
    T = np.eye(4)
    T[:3, 3] = A[:3, 3] + s * (B[:3, 3] - A[:3, 3])
    T[:3, :3] = sm.quat_to_matrix(slerp(sm.matrix_to_quat(A[:3, :3]), sm.matrix_to_quat(B[:3, :3]), s))
    return T


def rot_angle(A, B):
    return math.acos(np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1))


def bridge_targets(T_lift_end, T_row):
    """Waypoints: lift end -> H (row xy, lift height, lift orientation) -> row pose."""
    H = T_lift_end.copy(); H[:2, 3] = T_row[:2, 3]
    return [T_lift_end, H, T_row]


def plan_bridge(q_start, T_lift_end, T_row, calib, duration=BRIDGE_SIM_DURATION_S, dt=lp.SAMPLE_DT_S):
    """Per-sample IK along the 2-segment bridge with min-jerk timing per segment."""
    W = bridge_targets(T_lift_end, T_row)
    lens = [np.linalg.norm(b[:3, 3] - a[:3, 3]) + ROT_WEIGHT_M_PER_RAD * rot_angle(a, b) for a, b in zip(W, W[1:])]
    durs = [max(1.0, duration * L / sum(lens)) for L in lens]
    durs = [d * duration / sum(durs) for d in durs]
    t_all, T_all = [0.0], [W[0]]
    t_off = 0.0
    for (a, b), d in zip(zip(W, W[1:]), durs):
        tg, frac = lp.min_jerk_grid(d, dt)
        for tk, sk in zip(tg[1:], frac[1:]):
            t_all.append(t_off + tk); T_all.append(interp_pose(a, b, sk))
        t_off += d
    q, seed = [list(q_start)], list(q_start)
    for k in range(1, len(T_all)):
        qn = uk.ik_local(seed, T_all[k], calib).tolist()
        if lp.maxdiff(qn, seed) > 0.05:
            raise RuntimeError(f"BRIDGE IK branch jump {lp.maxdiff(qn, seed):.4f} rad at sample {k}")
        seed = qn; q.append(seed)
    q, wraps = lp.correct_wraps(q, q_start, "BRIDGE")
    return np.array(t_all), np.array(q), T_all, durs, wraps


def transport_tail(t_raw, q_raw, join_row, scale=TRANSPORT_TIME_SCALE):
    t = (t_raw[join_row:] - t_raw[join_row]) * scale
    return t, q_raw[join_row:]


def alignment_in_fitting(T_v1, T_fit):
    """V1 pose relative to its fitting: lateral offset, height above base, tab/slot yaw, tilt."""
    L = np.linalg.inv(T_fit) @ T_v1
    yaw = math.degrees(math.atan2(L[1, 0], L[0, 0]))
    yaw_mod = (yaw + 90) % 180 - 90          # two tabs at 180 deg -> symmetric
    tilt = math.degrees(math.acos(np.clip(L[2, 2], -1, 1)))
    return {"lateral_offset_mm": (L[:2, 3] * 1000).round(4).tolist(),
            "lateral_offset_abs_mm": round(float(np.linalg.norm(L[:2, 3]) * 1000), 4),
            "v1_bottom_above_fitting_base_mm": round(float(L[2, 3] * 1000), 4),
            "tab_to_slot_yaw_error_deg": round(yaw_mod, 4),
            "tilt_deg": round(tilt, 4)}
