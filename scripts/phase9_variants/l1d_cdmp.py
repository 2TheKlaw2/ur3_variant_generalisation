#!/usr/bin/env python3
"""L1D: DIRECT CDMP goal adaptation of the V1 INSERT and ROTATE skills (Victor Hernandez-Moreno's CartesianDMPs).

L1 scales the saved V1 INSERT/ROTATE trajectories by the CAD depth / seat yaw ratio. That is exactly what a
classic DMP (Ijspeert et al. 2013: forcing term multiplied by g - y0) produces for a goal change along one
axis. Victor's library uses the Pastor form (forcing term NOT multiplied by g - y0), where a goal change keeps
the learned forcing shape and the spring carries the extra distance. L1D does that directly:
  1. train one CDMP per stage on the SAME V1 trajectories L0/L1/L2 use (stage8_core.INSERT_CSV / ROTATE_CSV),
     with the Stage 10 parameters (alpha_s 7, k 100, 4 RBF/s);
  2. roll it out with the variant's CAD goal: INSERT goal moved down by (D_cad - D_demo) along the fitting axis
     (base z); ROTATE goal turned about base z by (seat yaw - demo rotation);
  3. resample to the same time grid as the other levels and express it as depth and yaw profiles.
Path conditioning (corridor) is NOT used: like L1 it only knows the goal. Kinematic simulation only.

  python3 scripts/phase9_variants/l1d_cdmp.py      # self-check: nominal rollout vs the saved V1 profiles
The library is loaded from $CDMP_LIBRARY or ~/ros2_ws/src/rs2_dmp_demo (GPLv3, not vendored).
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for d in ("../phase8_insert_sim", "../scene", "../phase6_pickup_sim", "../phase7_transport_sim", "."):
    p = str((HERE / d).resolve())
    if p not in sys.path:
        sys.path.insert(0, p)
import stage8_core as s8  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

CDMP_PARAMS = {"alpha_s": 7.0, "k_gain": 100.0, "rbfs_pSec": 4.0}     # as Stage 10 REACH/TRANSPORT


def library_path():
    return Path(os.environ.get("CDMP_LIBRARY", Path.home() / "ros2_ws/src/rs2_dmp_demo")).expanduser()


def library_file():
    return library_path() / "cdmp/src/cdmps.py"


class _quiet:
    def __enter__(self):
        self._o = sys.stdout; sys.stdout = open(os.devnull, "w")

    def __exit__(self, *e):
        sys.stdout.close(); sys.stdout = self._o


_MODELS = {}


def _demo(root, rel):
    t, T = s8.load_cdmp(root, rel)
    P = np.array([M[:3, 3] for M in T])
    Q = Rotation.from_matrix(np.array([M[:3, :3] for M in T])).as_quat()          # xyzw
    for k in range(1, len(Q)):                                                     # continuous sign
        if np.dot(Q[k], Q[k - 1]) < 0:
            Q[k] = -Q[k]
    return np.asarray(t, float), P, Q


def _model(root, stage):
    if stage in _MODELS:
        return _MODELS[stage]
    lib = library_path()
    if not library_file().is_file():
        raise RuntimeError(f"L1D needs Victor's CDMP library at {lib} (set CDMP_LIBRARY)")
    if str(lib) not in sys.path:
        sys.path.insert(0, str(lib))
    from cdmp.src.cdmps import CartesianDMPs
    rel = s8.INSERT_CSV if stage == "INSERT" else s8.ROTATE_CSV
    t, P, Q = _demo(root, rel)
    c = CartesianDMPs()
    with _quiet():
        c.load_demo(filename=os.path.basename(rel), dem_time=t, dem_pos=P, dem_quat=Q[:, [3, 0, 1, 2]])
        c.learn_cdmp(**CDMP_PARAMS)
    _MODELS[stage] = (c, t, P, Q)
    return _MODELS[stage]


def _rollout(root, stage, dpos_m, dyaw_deg):
    """Rollout from the demo start with the goal moved by dpos (base frame) and turned dyaw about base z."""
    c, t, P, Q = _model(root, stage)
    q_rel = Rotation.from_euler("z", dyaw_deg, degrees=True).as_quat()
    if q_rel[3] < 0:
        q_rel = -q_rel                                   # short rotation (see Stage 10, 58e5dfd)
    with _quiet():
        c.init_reproduction(tau_scaler=1.0, rep_abspose_start=np.r_[P[0], Q[0][[3, 0, 1, 2]]],
                            rep_relpose_goal=np.r_[np.asarray(dpos_m, float), q_rel[[3, 0, 1, 2]]])
        for s in c.rep_cs.s_track:
            c.rollout_step(curr_s_step=float(s))
    tr = np.asarray(c.rep_time, float).reshape(-1); tr = tr - tr[0]
    Pr = np.asarray(c.rep_pos, float); Qr = np.asarray(c.rep_quat, float)[:, [1, 2, 3, 0]]
    Qr /= np.linalg.norm(Qr, axis=1)[:, None]
    return tr, Pr, Qr


def _profile(t_grid, tr, Pr, Qr):
    """Depth [mm] and yaw [deg] relative to the start, resampled at the same normalised time as t_grid."""
    u = np.asarray(t_grid, float) / t_grid[-1]; ur = tr / tr[-1]
    z = np.interp(u, ur, Pr[:, 2])
    R = Rotation.from_quat(Qr).as_matrix(); R0 = R[0]
    yaw = np.unwrap([s8.yaw_about_z(Rk @ R0.T) for Rk in R])
    return (z - z[0]) * 1000.0, np.degrees(np.interp(u, ur, yaw))


def l1d_profiles(root, prof, D_cad_mm, y_goal_deg):
    """{'dz_ins','yaw_ins','dz_rot','yaw_rot'} on prof's time grids for this variant's CAD goal."""
    tr, Pr, Qr = _rollout(root, "INSERT", [0.0, 0.0, -(D_cad_mm - prof["D_demo"]) / 1000.0], 0.0)
    dz_i, y_i = _profile(prof["t_ins"], tr, Pr, Qr)
    tr, Pr, Qr = _rollout(root, "ROTATE", [0.0, 0.0, 0.0], y_goal_deg - prof["Y_demo"])
    dz_r, y_r = _profile(prof["t_rot"], tr, Pr, Qr)
    return {"dz_ins": dz_i, "yaw_ins": y_i, "dz_rot": dz_r, "yaw_rot": y_r}


def main():
    import variant_core as vc
    root = str(HERE.parents[1])
    prof = vc.demo_profiles(root)
    nom = l1d_profiles(root, prof, prof["D_demo"], prof["Y_demo"])        # no goal change = reproduce V1
    print(f"library: {library_path()}")
    print(f"nominal rollout vs the saved V1 profiles: INSERT depth max dev "
          f"{np.abs(nom['dz_ins'] - prof['dz_ins']).max():.3f} mm, end {nom['dz_ins'][-1]:.3f} vs {prof['dz_ins'][-1]:.3f} mm; "
          f"ROTATE yaw max dev {np.abs(nom['yaw_rot'] - prof['yaw_rot']).max():.3f} deg, end {nom['yaw_rot'][-1]:.2f} vs "
          f"{prof['yaw_rot'][-1]:.2f} deg")
    for D, Y in ((8.12, -24.33), (18.12, -1.61), (3.12, 24.39)):
        p = l1d_profiles(root, prof, D, Y)
        print(f"goal depth {D:5.2f} mm, seat yaw {Y:+6.2f}: INSERT end {-p['dz_ins'][-1]:.2f} mm (min {-p['dz_ins'].min():.2f}"
              f" deepest), ROTATE end {p['yaw_rot'][-1]:+.2f} deg (range {p['yaw_rot'].min():+.2f}..{p['yaw_rot'].max():+.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
