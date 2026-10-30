"""Stage 8 core (numpy + python-fcl + trimesh; no ROS): V1 INSERT/ROTATE experiments.

Experiments (all start from the Stage 7 ALIGN_INSERT end state, same grasp, same fitting):
  A   original alignment, original CDMP INSERT/ROTATE poses (base frame, as learned)
  B1  geometry-corrected start; CDMP replayed rigidly relative to the corrected start:
        T_B1(t) = T_corr_start @ inv(T_cdmp_start) @ T_cdmp(t)      (ROTATE: relative to the achieved start)
  B2  geometry-corrected start; geometry-conditioned CDMP: V1 follows the CDMP depth (base z)
      and yaw (about vertical) profiles along/about the fitting axis; lateral motion and tilt removed.

Common rules (identical for all experiments):
  * per-sample IK with the previous solution as seed (joint continuity), original CDMP timestamps
  * seating floor: commanded descent beyond floor contact is not executed (the TCP is held up so
    V1's lowest point stays on the floor); the unused commanded displacement is recorded
  * V1 <-> fitting_V1 contact classified against CAD mating regions (fitting frame):
      floor contact (pen <= FLOOR_TOL)            intended
      floor contact (pen >  FLOOR_TOL)            EXCESSIVE_PENETRATION (stop)
      tab on rotation stop at final angle         intended (ROTATE, |yaw| >= STOP_YAW_DEG)
      bore wall / entry slot or ring edge / channel side / rim   INTERFERENCE (stop) if pen > CONTACT_TOL
  * stop the trial at the first unacceptable contact; ROTATE only if INSERT reached a valid seat
Kinematic simulation only.
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("../scene", "../phase6_pickup_sim", "../phase7_transport_sim"):
    sys.path.insert(0, os.path.join(HERE, d))
import lift_profile as lp  # noqa: E402
import scene_model as sm  # noqa: E402
import transport_profile as tp  # noqa: E402
import ur_kinematics as uk  # noqa: E402

INSERT_CSV = "demonstrations/processed/phase4_assembly/v1_insert_cdmp_refined_depth_guard_SMOOTH_CANDIDATE.csv"
ROTATE_CSV = "demonstrations/processed/phase4_assembly/v1_rotate_cdmp_baseline_CANDIDATE.csv"
EXPERIMENTS = {"A": "v1_original_alignment", "B1": "v1_corrected_cdmp", "B2": "v1_geometry_conditioned",
               "B2R": "v1_geometry_conditioned_rotate_unseated"}
# B2R = B2, but ROTATE is run from the INSERT end state even if it is not seated (follow-up test:
# does the measured seating gap stop the locking rotation?). B2 itself is unchanged.
B2_FAMILY = ("B2", "B2R")

# Fitting geometry (fitting frame, mm) from cad/CAD_CHECKS.md
FLOOR_Z_MM, TOP_Z_MM, BORE_R_MM = 2.0, 7.0, 14.05   # bore Ø28.10 (revised CAD 30 Oct; was Ø28.0)
CHANNEL_TOP_Z_MM = 5.5         # V1 rotation channel occupies z 2.0..5.5 (above it: entry slots only)
RADIAL_CLEARANCE_MM, SLOT_HALF_ANGLE_DEG = 0.30, 1.0   # disc Ø27.50 in bore Ø28.10 (was 0.25)
# Tolerances (documented): STL chord deviation 0.01 mm -> contact tolerance 2x
CONTACT_TOL_MM = 0.02
FLOOR_TOL_MM = 0.05            # seating tolerance agreed for Stage 8
STOP_YAW_DEG = 25.5            # tab reaches the rotation stop from here (CAD: free to 26, blocked at 27)
CORRECTION_DURATION_S = 2.0    # NEW sim timing for the correction move (min-jerk)
PASS_YAW_RANGE_DEG = (23.5, 26.5)   # CW rotation accepted as assembled


# ------------------------------------------------------------------ data
def load_cdmp(root, rel):
    with open(os.path.join(root, rel), newline="") as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["time"]) for r in rows]); t = t - t[0]
    T = [sm.pose_to_matrix([float(r["x"]), float(r["y"]), float(r["z"])],
                           [float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])]) for r in rows]
    return t, T


def yaw_about_z(R):
    return math.atan2(R[1, 0], R[0, 0])


def rot_z(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M


def v1_in_fitting(T_v1, T_fit):
    """(lateral xy mm, bottom z mm, yaw deg in [-90,90) about fitting z, tilt deg)."""
    L = np.linalg.inv(T_fit) @ T_v1
    yaw = math.degrees(math.atan2(L[1, 0], L[0, 0])); yaw = (yaw + 90) % 180 - 90
    tilt = math.degrees(math.acos(np.clip(L[2, 2], -1, 1)))
    return L[:2, 3] * 1000, L[2, 3] * 1000, yaw, tilt


# ------------------------------------------------------------------ targets
def corrected_v1_start(T_v1_now, T_fit):
    """V1 upright on the fitting axis, tabs on the slot axis, at its current height."""
    L = np.linalg.inv(T_fit) @ T_v1_now
    yaw = math.atan2(L[1, 0], L[0, 0])
    yaw0 = 0.0 if math.cos(yaw) >= 0 else math.pi      # nearest tab/slot alignment (tabs are 180 deg apart)
    T = np.eye(4); T[:3, :3] = rot_z(yaw0)[:3, :3]; T[2, 3] = L[2, 3]
    return T_fit @ T


def commanded_tcp(exp, t_ins, T_ins, T_grasp, T_fit, T_v1_corr):
    """Commanded TCP poses for INSERT (before floor clamping)."""
    if exp == "A":
        return list(T_ins)
    T_tcp_corr = T_v1_corr @ np.linalg.inv(T_grasp)
    if exp == "B1":
        D = T_tcp_corr @ np.linalg.inv(T_ins[0])
        return [D @ T for T in T_ins]
    if exp in B2_FAMILY:
        return b2_poses(T_ins, T_v1_corr, T_grasp, T_fit)
    raise ValueError(exp)


def b2_poses(T_cdmp, T_v1_start, T_grasp, T_fit):
    """V1 follows the CDMP depth (base z) and yaw (about vertical) profiles on the fitting axis."""
    R_fit = T_fit[:3, :3]; out = []
    z0 = T_cdmp[0][2, 3]; R0 = T_cdmp[0][:3, :3]
    for T in T_cdmp:
        dz = T[2, 3] - z0
        dyaw = yaw_about_z(T[:3, :3] @ R0.T)
        M = np.eye(4); M[:3, 3] = R_fit @ np.array([0.0, 0.0, dz])
        Rv = rot_z(dyaw)[:3, :3]
        Tv = T_v1_start.copy()
        Tv[:3, :3] = R_fit @ Rv @ R_fit.T @ T_v1_start[:3, :3]
        Tv[:3, 3] = T_v1_start[:3, 3] + M[:3, 3]
        out.append(Tv @ np.linalg.inv(T_grasp))
    return out


def rotate_commanded(exp, T_rot, T_tcp_start_actual, T_grasp, T_fit):
    if exp == "A":
        return list(T_rot)
    if exp == "B1":
        D = T_tcp_start_actual @ np.linalg.inv(T_rot[0])
        return [D @ T for T in T_rot]
    if exp in B2_FAMILY:
        return b2_poses(T_rot, T_tcp_start_actual @ T_grasp, T_grasp, T_fit)
    raise ValueError(exp)


def correction_poses(T_from, T_to, duration=CORRECTION_DURATION_S, dt=lp.SAMPLE_DT_S):
    tg, frac = lp.min_jerk_grid(duration, dt)
    return tg, [tp.interp_pose(T_from, T_to, s) for s in frac]


# ------------------------------------------------------------------ geometry / contacts
class Geometry:
    """V1 <-> fitting_V1 contact evaluation against CAD mating regions.

    Contact within CONTACT_TOL is recognised exactly: if translating V1 by CONTACT_TOL along one of
    the fitting axes removes every FCL contact, status is "touching" and pen_mm is the minimum such
    shift (bisection, 0.0005 mm). The sampled signed distance below is only used beyond that; it can
    over-report on coplanar face contact because trimesh.contains() is ray-based.

    Pure seating is recognised exactly: if lifting V1 by FLOOR_TOL along the fitting axis
    removes all contact, it is floor contact with vertical penetration = floor - lowest point.

    FCL decides collision / clearance (exact mesh-mesh). Penetration depth and contact region
    are measured on DENSE SURFACE SAMPLES (~SAMPLE_DENSITY points/mm^2, fixed seed) of both
    meshes near FCL's contact points, so flat faces crossing edges are not missed
    (vertex-only tests miss them). Region rules use the fitting frame (mm):
      V1 point below the floor plane, or fitting floor point inside V1  -> floor
      z >= rim - tol                                                   -> rim
      r <  bore + 0.05                                                 -> bore_wall
      ROTATE with |yaw| >= STOP_YAW_DEG, in the rotation channel       -> rotation_stop
      z > channel top                                                  -> entry_slot_or_ring_edge
      otherwise                                                        -> channel_side
    """
    SAMPLE_DENSITY = 100.0     # points per mm^2 (~0.1 mm spacing)
    NEAR_CONTACT_MM = 1.0

    def __init__(self, objs, T_fit):
        import fcl
        import trimesh
        from scipy.spatial import cKDTree
        self.fcl, self.KD = fcl, cKDTree
        v, f = sm.read_stl(objs["pickup_V1"]["mesh"])
        self.v1 = trimesh.Trimesh(v, f, process=False)
        fv, ff = sm.read_stl(objs["fitting_V1"]["mesh"])
        self.fit = trimesh.Trimesh(fv, ff, process=False)      # fitting local frame
        self.T_fit = T_fit
        self.v1_obj = self._bvh(v, f); self.fit_obj = self._bvh(fv, ff)
        self.fit_obj.setTransform(fcl.Transform(T_fit[:3, :3], T_fit[:3, 3]))
        self.v1_verts = v
        rng = np.random.default_rng(0)
        self.v1_pts = np.vstack([v, self._sample(self.v1, rng)])          # V1 frame
        self.fit_pts = np.vstack([fv, self._sample(self.fit, rng)])        # fitting frame
        self.fit_tree = cKDTree(self.fit_pts)

    def _sample(self, mesh, rng):
        n = int(mesh.area * 1e6 * self.SAMPLE_DENSITY)
        tri = mesh.triangles; w = mesh.area_faces / mesh.area_faces.sum()
        idx = rng.choice(len(tri), size=n, p=w); u = rng.random((n, 2)); m = u.sum(1) > 1; u[m] = 1 - u[m]
        t = tri[idx]
        return t[:, 0] + u[:, :1] * (t[:, 1] - t[:, 0]) + u[:, 1:] * (t[:, 2] - t[:, 0])

    def _bvh(self, v, f):
        g = self.fcl.BVHModel(); g.beginModel(len(v), len(f)); g.addSubModel(v, f); g.endModel()
        return self.fcl.CollisionObject(g, self.fcl.Transform())

    def lowest_z_mm(self, T_v1):
        """Lowest point of V1 in the fitting frame (mm)."""
        L = np.linalg.inv(self.T_fit) @ T_v1
        return float((self.v1_verts @ L[:3, :3].T + L[:3, 3])[:, 2].min() * 1000)

    def evaluate(self, T_v1, stage, yaw_deg):
        fcl = self.fcl
        self.v1_obj.setTransform(fcl.Transform(T_v1[:3, :3], T_v1[:3, 3]))
        req = fcl.CollisionRequest(num_max_contacts=2000, enable_contact=True)
        c = fcl.CollisionResult(); fcl.collide(self.v1_obj, self.fit_obj, req, c)
        if not c.is_collision:
            r = fcl.DistanceResult(); fcl.distance(self.v1_obj, self.fit_obj, fcl.DistanceRequest(), r)
            return {"min_dist_mm": r.min_distance * 1000, "pen_mm": 0.0, "contact": "none", "status": "clear", "regions": {}}
        # Fast exact test: if lifting V1 by the floor tolerance along the fitting axis removes
        # every contact, the contact is pure seating on the floor (vertical penetration <= tol).
        low = self.lowest_z_mm(T_v1)
        T_up = T_v1.copy(); T_up[:3, 3] = T_v1[:3, 3] + self.T_fit[:3, 2] * FLOOR_TOL_MM / 1000
        self.v1_obj.setTransform(fcl.Transform(T_up[:3, :3], T_up[:3, 3]))
        cu = fcl.CollisionResult(); fcl.collide(self.v1_obj, self.fit_obj, fcl.CollisionRequest(), cu)
        if not cu.is_collision and low <= FLOOR_Z_MM + FLOOR_TOL_MM:
            pen = max(0.0, FLOOR_Z_MM - low)
            return {"min_dist_mm": 0.0, "pen_mm": pen, "contact": "floor", "status": "intended_contact",
                    "regions": {"floor": round(pen, 4)}}
        Tfi = np.linalg.inv(self.T_fit)
        cp = np.array([ct.pos for ct in c.contacts]) @ Tfi[:3, :3].T + Tfi[:3, 3]     # contacts, fitting frame
        # Exact within-tolerance test (same idea as the seating test): if translating V1 by
        # CONTACT_TOL along any fitting axis (+-x, +-y, +-z) removes every contact, the
        # penetration depth is <= CONTACT_TOL. Needed because the sampled signed distance is
        # unreliable for coplanar face-on-face contact (e.g. tab top on the ring underside).
        clear_dir = self._clears_within(T_v1, CONTACT_TOL_MM)
        if clear_dir is not None:
            regions = sorted({self._region(p, stage, yaw_deg, below_floor=False) for p in cp})
            pen = self._min_shift(T_v1, clear_dir, CONTACT_TOL_MM)     # exact upper bound on penetration
            return {"min_dist_mm": 0.0, "pen_mm": pen, "contact": "touching(" + "+".join(regions) + ")",
                    "status": "touching", "regions": {r: round(pen, 4) for r in regions}, "exact_clear_dir": clear_dir}
        return self._classify(T_v1, stage, yaw_deg, cp)

    _DIRS = {"+z": (2, 1), "-z": (2, -1), "+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1)}

    def _collides_shifted(self, T_v1, d, mm):
        ax, sgn = self._DIRS[d]
        T = T_v1.copy(); T[:3, 3] = T_v1[:3, 3] + sgn * self.T_fit[:3, ax] * mm / 1000
        self.v1_obj.setTransform(self.fcl.Transform(T[:3, :3], T[:3, 3]))
        r = self.fcl.CollisionResult(); self.fcl.collide(self.v1_obj, self.fit_obj, self.fcl.CollisionRequest(), r)
        return r.is_collision

    def _clears_within(self, T_v1, tol_mm):
        for d in self._DIRS:
            if not self._collides_shifted(T_v1, d, tol_mm):
                return d
        return None

    def _min_shift(self, T_v1, d, hi_mm, res_mm=0.0005):
        lo, hi = 0.0, hi_mm
        while hi - lo > res_mm:
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if self._collides_shifted(T_v1, d, mid) else (lo, mid)
        return hi

    def _classify(self, T_v1, stage, yaw_deg, contacts_fit):
        import trimesh
        L = np.linalg.inv(self.T_fit) @ T_v1                         # V1 -> fitting frame
        ctree = self.KD(contacts_fit)
        pv = self.v1_pts @ L[:3, :3].T + L[:3, 3]                   # V1 samples, fitting frame
        near_v = ctree.query(pv, distance_upper_bound=self.NEAR_CONTACT_MM / 1000)[0] < np.inf
        idx_f = sorted(set(i for lst in self.fit_tree.query_ball_point(contacts_fit, self.NEAR_CONTACT_MM / 1000) for i in lst))
        pf = self.fit_pts[idx_f]
        d1 = trimesh.proximity.signed_distance(self.fit, pv[near_v]) if near_v.any() else np.zeros(0)   # >0: inside fitting
        d2 = (trimesh.proximity.signed_distance(self.v1, (pf - L[:3, 3]) @ L[:3, :3]) if len(pf) else np.zeros(0))  # >0: inside V1
        regions = {}

        def put(k, d):
            regions[k] = max(regions.get(k, 0.0), d)
        floor_m = FLOOR_Z_MM / 1000
        for p, d in zip(pv[near_v][d1 > 0], d1[d1 > 0]):
            put(self._region(p, stage, yaw_deg, below_floor=p[2] < floor_m - 1e-7), d * 1000)
        for p, d in zip(pf[d2 > 0], d2[d2 > 0]):
            put(self._region(p, stage, yaw_deg, below_floor=abs(p[2] - floor_m) < 1e-6), d * 1000)
        if not regions:
            low = self.lowest_z_mm(T_v1)
            seated = abs(low - FLOOR_Z_MM) <= FLOOR_TOL_MM
            return {"min_dist_mm": 0.0, "pen_mm": 0.0, "contact": "touching" + ("(floor)" if seated else ""),
                    "status": "intended_contact" if seated else "touching", "regions": {}}
        status, worst = "clear", None
        for k, d in regions.items():
            limit = FLOOR_TOL_MM if k in ("floor", "rotation_stop") else CONTACT_TOL_MM
            if d > limit:
                bad = "EXCESSIVE_PENETRATION" if k == "floor" else "INTERFERENCE"
                if worst is None or d > regions[worst]:
                    status, worst = bad, k
        if status == "clear":
            status = "intended_contact" if any(k in ("floor", "rotation_stop") for k in regions) else "touching"
        contact = worst or "+".join(sorted(regions))
        return {"min_dist_mm": 0.0, "pen_mm": float(max(regions.values())), "contact": contact,
                "status": status, "regions": {k: round(v, 4) for k, v in regions.items()}}

    @staticmethod
    def _region(p, stage, yaw_deg, below_floor):
        r = math.hypot(p[0], p[1]) * 1000; z = p[2] * 1000
        if below_floor and r <= 16.6:
            return "floor"
        if z >= TOP_Z_MM - CONTACT_TOL_MM:
            return "rim"
        if r < BORE_R_MM + 0.05:
            return "bore_wall"
        if stage == "ROTATE" and abs(yaw_deg) >= STOP_YAW_DEG and z <= CHANNEL_TOP_Z_MM:
            return "rotation_stop"
        if z > CHANNEL_TOP_Z_MM:
            return "entry_slot_or_ring_edge"
        return "channel_side"


# ------------------------------------------------------------------ IK along a path with floor clamp
def solve_path(T_cmd, q_seed, calib, T_grasp, geom, clamp=True, max_step_rad=0.02):
    """Per-sample IK; holds V1 on the seating floor when the command goes deeper.

    Returns list of dicts with q, T_exec, clamp_mm (unused commanded descent)."""
    out, seed = [], list(q_seed)
    for k, T in enumerate(T_cmd):
        T_exec = T.copy(); clamp_mm = 0.0
        if clamp:
            low = geom.lowest_z_mm(T @ T_grasp)
            if low < FLOOR_Z_MM:
                clamp_mm = FLOOR_Z_MM - low
                T_exec[:3, 3] = T[:3, 3] + geom.T_fit[:3, 2] * clamp_mm / 1000.0
        q = uk.ik_local(seed, T_exec, calib).tolist()
        if k and lp.maxdiff(q, seed) > max_step_rad:
            raise RuntimeError(f"IK branch jump {lp.maxdiff(q, seed):.4f} rad at sample {k}")
        seed = q
        out.append({"q": q, "T_cmd": T, "T_exec": T_exec, "clamp_mm": clamp_mm})
    return out


def run_experiment(exp, q_align, calib, T_grasp, T_fit, objs, root, geom=None):
    """Offline evaluation of one experiment. Returns dict with per-sample rows and a summary."""
    geom = geom or Geometry(objs, T_fit)
    t_ins, T_ins = load_cdmp(root, INSERT_CSV)
    t_rot, T_rot = load_cdmp(root, ROTATE_CSV)
    T_tcp0 = uk.tcp(q_align, calib); T_v1_0 = T_tcp0 @ T_grasp
    lat0, z0, yaw0, tilt0 = v1_in_fitting(T_v1_0, T_fit)
    rows, stages = [], {}
    q_cur = list(q_align)

    def add_rows(stage, t, sol):
        nonlocal q_cur
        first_bad = None
        for k, (ti, s) in enumerate(zip(t, sol)):
            T_v1 = uk.tcp(s["q"], calib) @ T_grasp
            lat, zb, yaw, tilt = v1_in_fitting(T_v1, T_fit)
            ev = geom.evaluate(T_v1, stage, yaw)
            row = {"stage": stage, "sample": k, "t_s": float(ti), "q": s["q"],
                   "cmd_xyz_mm": (s["T_cmd"][:3, 3] * 1000).tolist(), "exec_xyz_mm": (s["T_exec"][:3, 3] * 1000).tolist(),
                   "clamp_mm": s["clamp_mm"], "lat_x_mm": lat[0], "lat_y_mm": lat[1], "lat_mm": float(np.hypot(*lat)),
                   "bottom_z_mm": zb, "lowest_z_mm": geom.lowest_z_mm(T_v1), "yaw_deg": yaw, "tilt_deg": tilt,
                   "min_dist_mm": ev["min_dist_mm"], "pen_mm": ev["pen_mm"], "contact": ev["contact"],
                   "status": ev["status"], "regions": ev["regions"], "feasible": True}
            if ev["status"] in ("INTERFERENCE", "EXCESSIVE_PENETRATION"):
                row["feasible"] = False; first_bad = row; rows.append(row); break
            rows.append(row); q_cur = s["q"]
        return first_bad

    result = {"experiment": exp, "initial": {"centre_offset_mm": float(np.hypot(*lat0)), "lateral_mm": lat0.tolist(),
                                             "tilt_deg": tilt0, "yaw_deg": yaw0, "bottom_z_mm": z0}}
    stop = None
    # ---- correction (B1, B2)
    if exp in ("B1",) + B2_FAMILY:
        T_v1_corr = corrected_v1_start(T_v1_0, T_fit)
        T_tcp_corr = T_v1_corr @ np.linalg.inv(T_grasp)
        t_c, T_c = correction_poses(T_tcp0, T_tcp_corr)
        sol = solve_path(T_c, q_cur, calib, T_grasp, geom, clamp=False)
        stop = add_rows("CORRECTION", t_c, sol)
        lat, zb, yaw, tilt = v1_in_fitting(uk.tcp(q_cur, calib) @ T_grasp, T_fit)
        stages["CORRECTION"] = {"duration_s": float(t_c[-1]), "samples": len(t_c),
                                "after": {"centre_offset_mm": float(np.hypot(*lat)), "tilt_deg": tilt, "yaw_deg": yaw,
                                          "bottom_z_mm": zb}, "timing": f"SIM {CORRECTION_DURATION_S} s min-jerk"}
    else:
        T_v1_corr = None
    # ---- insert
    if stop is None:
        T_cmd = commanded_tcp(exp, t_ins, T_ins, T_grasp, T_fit, T_v1_corr)
        handoff = sim_err(uk.tcp(q_cur, calib), T_cmd[0])
        sol = solve_path(T_cmd, q_cur, calib, T_grasp, geom)
        stop = add_rows("INSERT", t_ins, sol)
        ins_rows = [r for r in rows if r["stage"] == "INSERT" and r["feasible"]]
        last = ins_rows[-1] if ins_rows else None
        seated = bool(stop is None and last and abs(last["lowest_z_mm"] - FLOOR_Z_MM) <= FLOOR_TOL_MM
                      and last["lat_mm"] <= RADIAL_CLEARANCE_MM)
        stages["INSERT"] = {"cdmp_duration_s": float(t_ins[-1]), "samples_commanded": len(t_ins),
                            "samples_feasible": len(ins_rows), "handoff_mm": handoff,
                            "commanded_descent_mm": float((T_ins[0][2, 3] - T_ins[-1][2, 3]) * 1000),
                            "max_unused_commanded_descent_mm": max([r["clamp_mm"] for r in ins_rows] or [0.0]),
                            "seated": seated}
    else:
        seated = False
        stages["INSERT"] = {"status": "NOT_EXECUTED (correction failed)"}
    # ---- rotate
    insert_complete = bool(stop is None and stages["INSERT"].get("samples_feasible") == len(t_ins))
    if seated or (exp == "B2R" and insert_complete):
        T_start = uk.tcp(q_cur, calib)
        T_cmdr = rotate_commanded(exp, T_rot, T_start, T_grasp, T_fit)
        handoff = sim_err(T_start, T_cmdr[0])
        sol = solve_path(T_cmdr, q_cur, calib, T_grasp, geom)
        stop = add_rows("ROTATE", t_rot, sol)
        rot_rows = [r for r in rows if r["stage"] == "ROTATE" and r["feasible"]]
        stages["ROTATE"] = {"cdmp_duration_s": float(t_rot[-1]), "samples_commanded": len(t_rot),
                            "samples_feasible": len(rot_rows), "handoff_mm": handoff,
                            "max_unused_commanded_descent_mm": max([r["clamp_mm"] for r in rot_rows] or [0.0]),
                            "started_seated": seated,
                            "start_seating_error_mm": float(geom.lowest_z_mm(T_start @ T_grasp) - FLOOR_Z_MM)}
    else:
        stages["ROTATE"] = {"status": "NOT_EXECUTED"}
    result["stages"] = stages
    result["rows"] = rows
    result["first_unacceptable"] = None if stop is None else {
        k: stop[k] for k in ("stage", "sample", "t_s", "status", "contact", "pen_mm", "regions", "lat_mm", "bottom_z_mm", "tilt_deg", "yaw_deg")}
    feas = [r for r in rows if r["feasible"]]
    fin = feas[-1] if feas else None
    ins_feas = [r for r in feas if r["stage"] == "INSERT"]
    rotated = ("samples_commanded" in stages["ROTATE"] and
               stages["ROTATE"]["samples_feasible"] == stages["ROTATE"]["samples_commanded"])
    result["final"] = None if fin is None else {
        "stage": fin["stage"], "centre_offset_mm": fin["lat_mm"], "tilt_deg": fin["tilt_deg"], "yaw_deg": fin["yaw_deg"],
        "lowest_z_mm": fin["lowest_z_mm"], "seating_error_mm": fin["lowest_z_mm"] - FLOOR_Z_MM}
    result["max_insertion_depth_mm"] = max([TOP_Z_MM - r["lowest_z_mm"] for r in ins_feas] or [0.0])
    contact_rows = [r for r in rows if r["status"] not in ("clear",)]
    result["min_clearance_mm"] = min([r["min_dist_mm"] for r in rows] or [float("nan")])
    result["max_penetration_mm"] = max([r["pen_mm"] for r in rows] or [0.0])
    result["contact_samples"] = len(contact_rows)
    yaw_ok = fin is not None and PASS_YAW_RANGE_DEG[0] <= -fin["yaw_deg"] <= PASS_YAW_RANGE_DEG[1]
    result["outcome"] = "PASS" if (stop is None and seated and rotated and yaw_ok) else "FAIL"
    result["outcome_criteria"] = {"no_unacceptable_contact": stop is None, "seated": seated,
                                  "rotate_completed": bool(rotated), "final_cw_rotation_in_range": bool(yaw_ok),
                                  "rotation_range_deg": PASS_YAW_RANGE_DEG}
    if exp == "B2R":
        # Functional acceptance (separate from the strict criterion above, which stays in "outcome"):
        # insertion and rotation complete with no unacceptable contact and the final CW rotation in range.
        # The measured seating gap is reported, not judged.
        rot = [r for r in feas if r["stage"] == "ROTATE"]
        result["functional_outcome"] = "PASS" if (stop is None and insert_complete and rotated and yaw_ok) else "FAIL"
        result["functional_criteria"] = {
            "no_unacceptable_contact": stop is None, "insert_completed": insert_complete,
            "rotate_completed": bool(rotated), "final_cw_rotation_in_range": bool(yaw_ok),
            "seating_error_at_rotate_start_mm": stages["ROTATE"].get("start_seating_error_mm"),
            "seating_error_final_mm": None if fin is None else fin["lowest_z_mm"] - FLOOR_Z_MM,
            "max_cw_rotation_deg": max([-r["yaw_deg"] for r in rot] or [0.0]),
            "rotate_contact_regions": sorted({k for r in rot for k in r["regions"]}),
            "rotation_stop_reached": any("rotation_stop" in r["regions"] for r in rot),
            "note": "Seating gap is measured and reported; the strict 0.05 mm seat criterion is not applied here."}
    return result


def sim_err(A, B):
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000)
