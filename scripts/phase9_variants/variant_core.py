"""Stage 9 core (no ROS): transfer the single V1 INSERT/ROTATE demonstration to V1-V5.

Kinematic simulation only. Every condition starts from the same state as Stage 8 B2 after
its CORRECTION: the part upright and centred on its fitting axis, tabs on the entry-slot
axis, bottom START_ABOVE_TOP_MM above the fitting top. Correction (B2) is therefore common
to all conditions; the conditions differ only in how the V1 CDMP is adapted.

Conditions (demo = V1 INSERT CDMP depth/yaw profile and ROTATE CDMP yaw/height profile,
original timestamps, applied along/about the fitting axis as in B2):
  L0  no variant adaptation: V1 depth (5.42 mm) and V1 rotation (-24.39 deg) as demonstrated
  L1  CAD goal scaling: depth goal = CAD seat (floor) depth, rotation goal = CAD seat yaw;
      demo profiles scaled to the new goals (DMP spatial-scaling property), phases kept
      sequential as demonstrated (insert straight, then rotate)
  L2  CAD geometry-conditioned: depth as L1, yaw during INSERT follows the CAD free-yaw
      corridor as a function of depth (cad_corridor.py), then ROTATE from the corridor yaw at
      the seat to the CAD seat yaw with the demo's yaw progress profile

Common rules (as Stage 8): per-sample IK seeded by the previous sample, floor hold with the
unused descent recorded, V_i <-> fitting_V_i contact classified with the Stage 8 classifier
(exact 0.02 mm touching test, floor tolerance 0.05 mm), stop at the first unacceptable
contact. ROTATE runs when INSERT completed without unacceptable contact, seated or not (the
B2R protocol); seating is reported.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("../phase8_insert_sim", "../scene", "../phase6_pickup_sim", "../phase7_transport_sim", "."):
    sys.path.insert(0, os.path.join(HERE, d))
import cad_corridor as cc  # noqa: E402
import scene_model as sm  # noqa: E402
import stage8_core as s8  # noqa: E402
import ur_kinematics as uk  # noqa: E402

np = sm.np
LEVELS = ("L0", "L1", "L2", "CAD")   # CAD = CAD-only baseline: L2 path, min-jerk timing, no demonstration data
CORRIDOR_MARGIN_DEG = 1.0        # L2 keeps at least this far from a corridor wall (or the centre if narrower)
YAW_TARGET_TOL_DEG = 1.5         # final yaw accepted within this of the CAD seat yaw (V1 Stage 8: 23.5-26.5)
RETAIN_LIFT_MM = None            # "retained": a straight pull-out along the fitting axis collides (see VariantGeometry.retained)
# IK continuity: a real branch jump is ~pi; 0.1 rad per 20 ms sample allows the faster motions that
# scaled depth goals produce at the original timestamps. Joint speeds are then checked separately
# against the UR3e limits (base/shoulder/elbow 180 deg/s, wrists 360 deg/s).
IK_MAX_STEP_RAD = 0.1
UR3E_SPEED_LIMIT = np.radians([180, 180, 180, 360, 360, 360])


# ------------------------------------------------------------------ geometry
class VariantGeometry(s8.Geometry):
    """Stage 8 classifier for moving part i in fitting i (same tolerances and tests).

    Region labels: floor, rim, bore_wall, feature (slot/channel/helix surfaces). Only the floor
    is an intended contact; every other region is judged against CONTACT_TOL."""

    def __init__(self, i, objs, T_fit):
        import fcl
        import trimesh
        from scipy.spatial import cKDTree
        self.fcl, self.KD = fcl, cKDTree
        v, f = sm.read_stl(objs[f"pickup_V{i}"]["mesh"])
        self.v1 = trimesh.Trimesh(v, f, process=False)
        fv, ff = sm.read_stl(objs[f"fitting_V{i}"]["mesh"])
        self.fit = trimesh.Trimesh(fv, ff, process=False)
        self.T_fit = T_fit
        self.v1_obj = self._bvh(v, f); self.fit_obj = self._bvh(fv, ff)
        self.fit_obj.setTransform(fcl.Transform(T_fit[:3, :3], T_fit[:3, 3]))
        self.v1_verts = v
        self.top_mm = float(fv[:, 2].max() * 1000)
        rng = np.random.default_rng(0)
        self.v1_pts = np.vstack([v, self._sample(self.v1, rng)])
        self.fit_pts = np.vstack([fv, self._sample(self.fit, rng)])
        self.fit_tree = cKDTree(self.fit_pts)

    def _region(self, p, stage, yaw_deg, below_floor):
        r = math.hypot(p[0], p[1]) * 1000; z = p[2] * 1000
        if below_floor and r <= 16.6:
            return "floor"
        if z >= self.top_mm - s8.CONTACT_TOL_MM:
            return "rim"
        if r < s8.BORE_R_MM + 0.05:
            return "bore_wall"
        return "feature"

    def retained(self, T_v1, step_mm=0.25):
        """True if the part cannot be pulled straight out: lifting it along the fitting axis
        until its bottom clears the fitting top collides somewhere on the way. (Was a fixed
        1 mm lift; V5 locks by its reverse helix, which only blocks after a few mm.)"""
        clear_mm = self.top_mm - self.lowest_z_mm(T_v1) + 0.5
        for k in range(1, int(clear_mm / step_mm) + 2):
            T = T_v1.copy(); T[:3, 3] = T_v1[:3, 3] + self.T_fit[:3, 2] * (k * step_mm) / 1000
            self.v1_obj.setTransform(self.fcl.Transform(T[:3, :3], T[:3, 3]))
            r = self.fcl.CollisionResult(); self.fcl.collide(self.v1_obj, self.fit_obj, self.fcl.CollisionRequest(), r)
            if r.is_collision:
                return True
        return False


# ------------------------------------------------------------------ CAD features
def load_corridor(path):
    rows = {}
    with open(path) as f:
        next(f)
        for line in f:
            z, k, lo, hi = line.strip().split(",")
            rows.setdefault(float(z), [])
            if int(k) >= 0:
                rows[float(z)].append((float(lo), float(hi)))
    zs = sorted(rows, reverse=True)
    return [(z, rows[z]) for z in zs]


def corridor_at(rows, z_mm):
    """Free intervals at the grid level at or just below z (conservative for descent)."""
    zs = np.array([z for z, _ in rows])
    k = int(np.argmin(np.where(zs <= z_mm + 1e-6, z_mm - zs, np.inf))) if (zs <= z_mm + 1e-6).any() else len(zs) - 1
    return rows[k][1]


MAX_YAW_STEP_DEG = 1.0           # per 0.1 mm grid level (10 deg/mm; the steepest CAD helix is ~4 deg/mm)


def corridor_yaw_path(rows, margin=CORRIDOR_MARGIN_DEG, max_step=MAX_YAW_STEP_DEG):
    """CAD yaw to hold at each grid height when descending from yaw 0 (top -> seat).

    Keeps the current yaw while it is at least `margin` inside the free interval that contains
    it; otherwise moves the minimum amount (or to the interval centre if it is narrower than
    2*margin). The path must stay connected: if the corridor would need a jump larger than
    max_step between adjacent levels (or closes), the yaw is held and the remaining levels are
    marked unreachable (lo = hi = None), so the motion meets the obstruction instead of
    jumping through it. Returns [(z_mm, yaw_deg, lo, hi)] top -> bottom."""
    out, y, blocked = [], 0.0, False
    for z, ints in rows:
        if blocked or not ints:
            blocked = True; out.append((z, y, None, None)); continue
        iv = cc.interval_containing(ints, y) or min(ints, key=lambda t: min(abs(t[0] - y), abs(t[1] - y)))
        lo, hi = iv
        m = min(margin, 0.5 * (hi - lo))
        y_new = min(max(y, lo + m), hi - m)
        if abs(y_new - y) > max_step:
            blocked = True; out.append((z, y, None, None)); continue
        y = y_new
        out.append((z, y, lo, hi))
    return out


def seat_target(rows, demo_rotate_deg, v1_lock_deg):
    """CAD seat yaw: the seat-corridor end in the rotation direction, less the margin the V1
    demonstration kept from V1's lock end, but not past the seat-corridor centre."""
    path = corridor_yaw_path(rows)
    z_seat, y_seat, lo, hi = path[-1]
    if lo is None:                                 # corridor closes before the seat
        z_close = next(z for z, _, l, _ in path if l is None)     # first level the connected corridor cannot reach
        return {"seat_corridor_deg": None, "lock_end_deg": None, "centre_deg": None, "demo_margin_deg": None,
                "target_deg": 0.0, "yaw_at_seat_on_path_deg": None, "corridor_closed_at_bottom_z_mm": z_close,
                "note": "CAD corridor closes before the seat; seat yaw undefined, 0 deg (slot axis) used as goal"}
    lock = lo if abs(lo) >= abs(hi) else hi
    margin = abs(v1_lock_deg) - abs(demo_rotate_deg)
    centre = 0.5 * (lo + hi)
    tgt = lock - math.copysign(margin, lock)
    if (lock - centre) * (tgt - centre) < 0:      # would pass the centre
        tgt = centre
    return {"seat_corridor_deg": [lo, hi], "lock_end_deg": lock, "centre_deg": centre,
            "demo_margin_deg": margin, "target_deg": tgt, "yaw_at_seat_on_path_deg": y_seat}


# ------------------------------------------------------------------ demo profiles
def demo_profiles(root):
    t_i, T_i = s8.load_cdmp(root, s8.INSERT_CSV)
    t_r, T_r = s8.load_cdmp(root, s8.ROTATE_CSV)

    def prof(T):
        z0, R0 = T[0][2, 3], T[0][:3, :3]
        dz = np.array([(M[2, 3] - z0) * 1000 for M in T])
        yaw = np.array([math.degrees(s8.yaw_about_z(M[:3, :3] @ R0.T)) for M in T])
        return dz, yaw
    dz_i, y_i = prof(T_i); dz_r, y_r = prof(T_r)
    return {"t_ins": t_i, "dz_ins": dz_i, "yaw_ins": y_i, "t_rot": t_r, "dz_rot": dz_r, "yaw_rot": y_r,
            "D_demo": -dz_i[-1], "Y_demo": y_r[-1]}


def part_pose(T_fit, z_mm, yaw_deg):
    L = np.eye(4); L[:3, :3] = s8.rot_z(math.radians(yaw_deg))[:3, :3]; L[2, 3] = z_mm / 1000
    return T_fit @ L


def min_jerk(t):
    tau = np.clip(np.asarray(t) / t[-1], 0, 1)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def condition_targets(level, prof, z_start, feat, rows):
    """Per-sample (bottom z mm, yaw deg) of the part in the fitting frame, INSERT and ROTATE."""
    D_cad = z_start - s8.FLOOR_Z_MM
    if level == "CAD":
        # CAD-only baseline: the same CAD path as L2 (seat depth, corridor yaw, seat yaw) with
        # generic minimum-jerk timing over the same INSERT/ROTATE durations; no demo profile,
        # no demo pause or end lift. Isolates what the demonstration contributes.
        path = corridor_yaw_path(rows)
        pz = np.array([p[0] for p in path][::-1]); py = np.array([p[1] for p in path][::-1])
        z_ins = z_start - D_cad * min_jerk(prof["t_ins"])
        yaw_ins = np.interp(np.maximum(z_ins, s8.FLOOR_Z_MM), pz, py)
        y_goal = feat["target_deg"]; y0 = yaw_ins[-1]
        yaw_rot = y0 + (y_goal - y0) * min_jerk(prof["t_rot"])
        return z_ins, yaw_ins, np.zeros_like(prof["t_rot"]), yaw_rot, y_goal, D_cad, 1.0
    sc = 1.0 if level == "L0" else D_cad / prof["D_demo"]
    z_ins = z_start + prof["dz_ins"] * sc
    if level == "L2":
        path = corridor_yaw_path(rows)
        pz = np.array([p[0] for p in path][::-1]); py = np.array([p[1] for p in path][::-1])
        yaw_ins = np.interp(np.maximum(z_ins, s8.FLOOR_Z_MM), pz, py)
    else:
        yaw_ins = prof["yaw_ins"].copy()
    if level == "L0":
        y_goal = prof["Y_demo"]
    else:
        y_goal = feat["target_deg"]
    y0 = yaw_ins[-1]
    if level == "L2":
        yaw_rot = y0 + (y_goal - y0) * prof["yaw_rot"] / prof["Y_demo"]
    else:
        yaw_rot = y0 + prof["yaw_rot"] * (1.0 if level == "L0" else y_goal / prof["Y_demo"])
    return z_ins, yaw_ins, prof["dz_rot"], yaw_rot, y_goal, D_cad, sc


# ------------------------------------------------------------------ IK helpers
def ik_continuation(q_seed, T_from, T_to, calib, steps=60):
    """Walk the TCP from T_from to T_to in small steps (same IK branch)."""
    import transport_profile as tp
    q = list(q_seed)
    for k in range(1, steps + 1):
        q = uk.ik_local(q, tp.interp_pose(T_from, T_to, k / steps), calib).tolist()
    err = s8.sim_err(uk.tcp(q, calib), T_to)
    if err > 1e-3:
        raise RuntimeError(f"IK continuation did not converge ({err:.4f} mm)")
    return q


# ------------------------------------------------------------------ run one variant x level
def run_condition(i, level, ctx, geom, rows, feat, prof):
    calib, T_fit, G = ctx["calib"], ctx["T_fit"], ctx["G"]
    z_start = ctx["z_start"]
    z_ins, yaw_ins, dz_rot, yaw_rot, y_goal, D_cad, sc = condition_targets(level, prof, z_start, feat, rows)
    Ginv = np.linalg.inv(G)
    T_cmd_ins = [part_pose(T_fit, z, ctx["yaw0"] + y) @ Ginv for z, y in zip(z_ins, yaw_ins)]
    q_cur = list(ctx["q_start"])
    out_rows, stages = [], {}

    def add(stage, t, sol):
        nonlocal q_cur
        for k, (ti, s) in enumerate(zip(t, sol)):
            T_v = uk.tcp(s["q"], calib) @ G
            lat, zb, yaw, tilt = s8.v1_in_fitting(T_v, T_fit)
            ev = geom.evaluate(T_v, stage, yaw)
            row = {"stage": stage, "sample": k, "t_s": float(ti), "q": s["q"],
                   "cmd_xyz_mm": (s["T_cmd"][:3, 3] * 1000).tolist(), "exec_xyz_mm": (s["T_exec"][:3, 3] * 1000).tolist(),
                   "clamp_mm": s["clamp_mm"], "lat_x_mm": lat[0], "lat_y_mm": lat[1], "lat_mm": float(np.hypot(*lat)),
                   "bottom_z_mm": zb, "lowest_z_mm": geom.lowest_z_mm(T_v), "yaw_deg": yaw, "tilt_deg": tilt,
                   "min_dist_mm": ev["min_dist_mm"], "pen_mm": ev["pen_mm"], "contact": ev["contact"],
                   "status": ev["status"], "regions": ev["regions"], "feasible": True}
            if ev["status"] in ("INTERFERENCE", "EXCESSIVE_PENETRATION"):
                row["feasible"] = False; out_rows.append(row); return row
            out_rows.append(row); q_cur = s["q"]
        return None

    try:
        sol = s8.solve_path(T_cmd_ins, q_cur, calib, G, geom, max_step_rad=IK_MAX_STEP_RAD)
        stop = add("INSERT", prof["t_ins"], sol)
        ik_err = None
    except RuntimeError as e:
        stop, ik_err = {"stage": "INSERT", "status": "IK_FAIL", "contact": str(e)}, str(e)
    ins = [r for r in out_rows if r["stage"] == "INSERT" and r["feasible"]]
    insert_complete = stop is None and len(ins) == len(prof["t_ins"])
    seated = bool(insert_complete and abs(ins[-1]["lowest_z_mm"] - s8.FLOOR_Z_MM) <= s8.FLOOR_TOL_MM
                  and ins[-1]["lat_mm"] <= s8.RADIAL_CLEARANCE_MM)
    stages["INSERT"] = {"samples_commanded": len(prof["t_ins"]), "samples_feasible": len(ins),
                        "commanded_depth_mm": float(z_ins[0] - z_ins[-1]), "cad_seat_depth_mm": D_cad,
                        "depth_scale": sc, "max_unused_commanded_descent_mm": max([r["clamp_mm"] for r in ins] or [0.0]),
                        "seated": seated, "end_seating_gap_mm": ins[-1]["lowest_z_mm"] - s8.FLOOR_Z_MM if ins else None}
    if insert_complete:
        T_start = uk.tcp(q_cur, calib); T_v0 = T_start @ G
        _, zb0, y0_act, _ = s8.v1_in_fitting(T_v0, T_fit)
        zb_r = zb0 + dz_rot
        T_cmd_rot = [part_pose(T_fit, z, ctx["yaw0"] + y) @ Ginv for z, y in zip(zb_r, yaw_rot)]
        try:
            sol = s8.solve_path(T_cmd_rot, q_cur, calib, G, geom, max_step_rad=IK_MAX_STEP_RAD)
            stop = add("ROTATE", prof["t_rot"], sol)
        except RuntimeError as e:
            stop, ik_err = {"stage": "ROTATE", "status": "IK_FAIL", "contact": str(e)}, str(e)
        rot = [r for r in out_rows if r["stage"] == "ROTATE" and r["feasible"]]
        stages["ROTATE"] = {"samples_commanded": len(prof["t_rot"]), "samples_feasible": len(rot),
                            "commanded_rotation_deg": float(yaw_rot[-1] - yaw_rot[0]), "goal_yaw_deg": float(y_goal),
                            "started_seated": seated}
    else:
        stages["ROTATE"] = {"status": "NOT_EXECUTED (INSERT stopped)"}
    feas = [r for r in out_rows if r["feasible"]]
    fin = feas[-1] if feas else None
    rotated = stages["ROTATE"].get("samples_feasible") == len(prof["t_rot"])
    T_fin = uk.tcp(fin["q"], calib) @ G if fin else None
    retained = bool(geom.retained(T_fin)) if fin else False
    yaw_ok = bool(fin is not None and abs(fin["yaw_deg"] - feat["target_deg"]) <= YAW_TARGET_TOL_DEG)
    fs = None if stop is None else {k: stop.get(k) for k in ("stage", "sample", "t_s", "status", "contact", "pen_mm",
                                                              "regions", "lat_mm", "bottom_z_mm", "yaw_deg")}
    res = {"variant": f"V{i}", "level": level, "rows": out_rows, "stages": stages, "first_unacceptable": fs,
           "ik_error": ik_err,
           "final": None if fin is None else {"stage": fin["stage"], "yaw_deg": fin["yaw_deg"], "lowest_z_mm": fin["lowest_z_mm"],
                                              "seating_gap_mm": fin["lowest_z_mm"] - s8.FLOOR_Z_MM,
                                              "centre_offset_mm": fin["lat_mm"], "tilt_deg": fin["tilt_deg"]},
           "max_insertion_depth_mm": max([z_start - r["lowest_z_mm"] for r in feas] or [0.0]),
           "cad_seat_depth_mm": D_cad,
           "min_clearance_mm": min([r["min_dist_mm"] for r in out_rows] or [float("nan")]),
           "max_penetration_mm": max([r["pen_mm"] for r in out_rows] or [0.0]),
           "touching_samples": sum(r["status"] == "touching" for r in out_rows),
           "cad_seat_yaw_deg": feat["target_deg"]}
    qs = np.array([r["q"] for r in feas]) if len(feas) > 1 else np.zeros((2, 6))
    ts = np.array([r["t_s"] + (prof["t_ins"][-1] + 0.02 if r["stage"] == "ROTATE" else 0.0) for r in feas]) if len(feas) > 1 else np.array([0, 1.0])
    dq = np.abs(np.diff(qs, axis=0)) / np.maximum(np.diff(ts), 1e-6)[:, None]
    peak = dq.max(axis=0) if len(dq) else np.zeros(6)
    res["peak_joint_speed_deg_s"] = np.degrees(peak).tolist()
    res["peak_speed_fraction_of_limit"] = float((peak / UR3E_SPEED_LIMIT).max())
    crit = {"within_ur3e_joint_speed_limits": bool((peak <= UR3E_SPEED_LIMIT).all()),
            "no_unacceptable_contact": stop is None, "insert_completed": insert_complete, "rotate_completed": bool(rotated),
            "final_yaw_within_tol_of_cad_seat": yaw_ok, "retained_by_fitting": retained, "seated_at_insert_end": seated}
    res["criteria"] = crit
    res["functional_outcome"] = "PASS" if all(crit[k] for k in ("within_ur3e_joint_speed_limits", "no_unacceptable_contact", "insert_completed",
                                                                "rotate_completed", "final_yaw_within_tol_of_cad_seat",
                                                                "retained_by_fitting")) else "FAIL"
    res["strict_outcome"] = "PASS" if res["functional_outcome"] == "PASS" and seated else "FAIL"
    return res
