#!/usr/bin/env python3
"""CAD feature extraction: collision-free yaw corridor vs insertion height, per variant.

For each variant, the moving part is placed upright and centred on its fitting axis
(the state after the Stage 8 CORRECTION). For each bottom height z (fitting frame) the
yaw range [-45, 45] deg is swept with exact FCL mesh-mesh collision; free yaw intervals
are refined to 0.05 deg by bisection. No demonstration data is used here.

"Free" uses the same acceptance as the Stage 8 contact classifier: no FCL contact, or a
contact that a CONTACT_TOL (0.02 mm) shift along one fitting axis removes ("touching").
V5 needs this: its fitting bore narrows to r 13.73 mm against the part disc at r 13.75 mm
(the 0.02 mm CAD issue #1), so a strict no-contact corridor closes at bottom z ~14.3 mm.

  python3 scripts/phase9_variants/cad_corridor.py [--variants 1 2 3 4 5] [--out DIR]

Output: <out>/V<i>_corridor.csv (z_mm, interval_lo_deg, interval_hi_deg per free
interval) and cad_features.json (straight-entry depth, helix rate, seat corridor,
lock angle) used by the L1/L2 conditions. Geometric CAD analysis only.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../scene"))
import scene_model as sm  # noqa: E402

np = sm.np
FLOOR_Z_MM = 2.0
SEAT_Z_MM = FLOOR_Z_MM + 0.01        # just above the floor (floor contact is intended, not a corridor limit)
CONTACT_TOL_MM = 0.02               # same as stage8_core.CONTACT_TOL_MM
YAW_RANGE_DEG = 45.0
YAW_STEP_DEG = 0.5
Z_STEP_MM = 0.1
START_ABOVE_TOP_MM = 0.70            # V1 corrected start: part bottom 0.70 mm above the fitting top (Stage 8)


class PairCollider:
    """Exact FCL collision between moving part i and fitting i, part pose in the fitting frame."""

    def __init__(self, i, objs=None):
        import fcl
        self.fcl = fcl
        objs = objs or sm.scene_objects()
        pv, pf = sm.read_stl(objs[f"pickup_V{i}"]["mesh"])
        fv, ff = sm.read_stl(objs[f"fitting_V{i}"]["mesh"])
        self.part = self._obj(pv, pf); self.fit = self._obj(fv, ff)
        self.top_mm = float(fv[:, 2].max() * 1000)

    def _obj(self, v, f):
        g = self.fcl.BVHModel(); g.beginModel(len(v), len(f)); g.addSubModel(v, f); g.endModel()
        return self.fcl.CollisionObject(g, self.fcl.Transform())

    def collides(self, z_mm, yaw_deg, dx_mm=0.0, dy_mm=0.0):
        a = math.radians(yaw_deg); c, s = math.cos(a), math.sin(a)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        self.part.setTransform(self.fcl.Transform(R, np.array([dx_mm, dy_mm, z_mm]) / 1000))
        r = self.fcl.CollisionResult(); self.fcl.collide(self.part, self.fit, self.fcl.CollisionRequest(), r)
        return r.is_collision

    def passable(self, z_mm, yaw_deg, tol_mm=CONTACT_TOL_MM):
        if not self.collides(z_mm, yaw_deg):
            return True
        for dx, dy, dz in ((0, 0, tol_mm), (0, 0, -tol_mm), (tol_mm, 0, 0), (-tol_mm, 0, 0), (0, tol_mm, 0), (0, -tol_mm, 0)):
            if not self.collides(z_mm + dz, yaw_deg, dx, dy):
                return True
        return False


def free_intervals(col, z, step=YAW_STEP_DEG, rng=YAW_RANGE_DEG, tol=0.05):
    ys = np.arange(-rng, rng + 1e-9, step)
    free = np.array([col.passable(z, y) for y in ys])
    out, k = [], 0
    while k < len(ys):
        if not free[k]:
            k += 1; continue
        j = k
        while j + 1 < len(ys) and free[j + 1]:
            j += 1
        lo, hi = ys[k], ys[j]
        if k > 0:            # refine lower edge between ys[k-1] (blocked) and ys[k] (free)
            a, b = ys[k - 1], ys[k]
            while b - a > tol:
                m = 0.5 * (a + b); a, b = (a, m) if col.passable(z, m) else (m, b)
            lo = b
        if j < len(ys) - 1:
            a, b = ys[j], ys[j + 1]
            while b - a > tol:
                m = 0.5 * (a + b); a, b = (m, b) if col.passable(z, m) else (a, m)
            hi = a
        out.append((float(lo), float(hi)))
        k = j + 1
    return out


def sweep(i, objs=None, z_step=Z_STEP_MM):
    col = PairCollider(i, objs)
    z_top = col.top_mm + START_ABOVE_TOP_MM
    zs = list(np.round(np.arange(z_top, SEAT_Z_MM, -z_step), 4)) + [SEAT_Z_MM]
    return col, [(float(z), free_intervals(col, z)) for z in zs]


def interval_containing(ints, y):
    for lo, hi in ints:
        if lo - 1e-9 <= y <= hi + 1e-9:
            return lo, hi
    return None


def features(i, col, rows):
    """Summarise the corridor followed from yaw 0 at the start height down to the seat."""
    path, y = [], 0.0
    for z, ints in rows:
        iv = interval_containing(ints, y)
        if iv is None:                       # corridor moved: take the nearest free interval
            iv = min(ints, key=lambda t: min(abs(t[0] - y), abs(t[1] - y))) if ints else None
        if iv is None:
            path.append((z, None, None)); continue
        y = min(max(y, iv[0]), iv[1]); path.append((z, iv[0], iv[1]))
    seat = rows[-1][1]
    seat_iv = interval_containing(seat, path[-1][1] if path[-1][1] is not None else 0.0) or (seat[0] if seat else None)
    # straight entry: depth below the top over which the corridor contains yaw 0 and is narrow (slot)
    top = col.top_mm
    straight_to = None
    for z, lo, hi in path:
        if lo is None or not (lo <= 0.0 <= hi):
            break
        straight_to = z
    centres = [(z, 0.5 * (lo + hi), hi - lo) for z, lo, hi in path if lo is not None]
    seat_lo, seat_hi = seat_iv if seat_iv else (None, None)
    # lock angle: the seat-corridor end farthest from 0 (rotation direction from CAD)
    if seat_lo is None:
        lock = None
    else:
        lock = seat_lo if abs(seat_lo) >= abs(seat_hi) else seat_hi
    return {"variant": f"V{i}", "fitting_top_mm": top, "floor_mm": FLOOR_Z_MM,
            "start_bottom_mm": rows[0][0], "seat_depth_from_start_mm": rows[0][0] - FLOOR_Z_MM,
            "straight_entry_until_bottom_z_mm": straight_to,
            "seat_corridor_deg": [seat_lo, seat_hi], "seat_corridor_width_deg": None if seat_lo is None else seat_hi - seat_lo,
            "lock_end_deg": lock,
            "corridor_centre_path": [(round(z, 3), round(c, 3), round(w, 3)) for z, c, w in centres]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--out", default=os.path.join(sm.PKG, "results/cad_features"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    objs = sm.scene_objects(); allf = {}
    jpath = os.path.join(a.out, "cad_features.json")
    if os.path.exists(jpath):
        allf = json.load(open(jpath))
    for i in a.variants:
        col, rows = sweep(i, objs)
        with open(os.path.join(a.out, f"V{i}_corridor.csv"), "w") as f:
            f.write("z_mm,interval,lo_deg,hi_deg\n")
            for z, ints in rows:
                for k, (lo, hi) in enumerate(ints):
                    f.write(f"{z:.3f},{k},{lo:.3f},{hi:.3f}\n")
                if not ints:
                    f.write(f"{z:.3f},-1,,\n")
        ft = features(i, col, rows); allf[f"V{i}"] = ft
        print(f"V{i}: top {ft['fitting_top_mm']:.2f}, straight entry down to z={ft['straight_entry_until_bottom_z_mm']}, "
              f"seat corridor {ft['seat_corridor_deg']}, lock end {ft['lock_end_deg']}", flush=True)
        json.dump(allf, open(jpath, "w"), indent=1)


if __name__ == "__main__":
    main()
