#!/usr/bin/env python3
"""Compute provisional plate-to-base registration for the static RViz scene.

Inputs (read-only):
  config/saved_poses/v1_pickup_V{1..5}_geometry.yaml  measured TCP at each pickup part
  config/saved_poses/v1_insertion_end.yaml            measured TCP with V1 seated in its fitting
  cad/manifest.yaml                                   component poses in each plate frame

Output:
  config/scene/scene_registration.yaml

Method:
  * Pickup plate: least-squares fit of the five measured TCP xy points to the CAD
    pocket centres (plate-local x = 0, 40, ..., 160 mm). Gives plate origin xy and yaw.
  * Placement plate: if config/saved_poses/v1_place_V{i}_geometry.yaml exist for >= 2 fittings
    (lab registration: the robot's own grasp, part seated in each fitting), Kabsch fit of their
    TCP xy to the CAD fitting centres. Otherwise origin xy = V1 TCP xy at insertion end and
    yaw ASSUMED parallel to the pickup plate (one point cannot give yaw).
  * Heights: the robot base sits on the trolley top (z = trolley_surface_z, 0 = robot
    mounting plane). The slotted aluminium fixture plate lies on the trolley top and both
    blue plates lie on it, so plate underside z = trolley_surface_z + fixture thickness.
  * Environment collision boxes (written as STL to cad/environment/):
      fixture_plate  aluminium plate, top = plate underside z, 495 x 320 mm (lab ruler), blue
                     plates' outer ends at its ends, far edge FIXTURE_FAR_EDGE beyond the blue plates
      trolley_top    trolley top under the workspace, top 0.5 mm below the mounting plane
                     (avoids a permanent resting contact with the robot base)

Nothing here is a verified calibration. Usage:
  python3 scripts/scene/compute_scene_registration.py [--trolley-surface-z 0.0] [--fixture-thickness 0.0125]
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def load(path):
    with open(os.path.join(PKG, path)) as f:
        return yaml.safe_load(f)


def tcp_xyz(name):
    return np.array(load(f"config/saved_poses/{name}.yaml")["tcp"]["position_m"], float)


PICKUP_X = (-0.020, 0.180)        # pickup plate extent along its row (V1 axis = 0), 200 mm
PLACEMENT_X = (-0.025, 0.225)     # placement plate extent, 250 mm
PLATE_HALF_W = 0.035              # both plates 70 mm wide
FIXTURE_LENGTH = 0.495            # aluminium fixture plate along the row (lab ruler, 30 Oct 2026)
FIXTURE_WIDTH = 0.320             # across the row (lab ruler)
FIXTURE_FAR_EDGE = 0.077          # far edge beyond the blue plates' far long edge (lab ruler, 3 readings 76-77 mm)
TROLLEY_TOP_SIZE = (0.80, 0.80, 0.020)
TROLLEY_TOP_DY = 0.20             # shift the trolley-top box toward the robot (covers the base area)


def write_box_stl(path, size_m):
    """Binary STL (mm) of a box centred at the origin."""
    import struct
    hx, hy, hz = (v * 500.0 for v in size_m)
    V = np.array([[x, y, z] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)])
    F = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
         (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"box".ljust(80, b" ")); f.write(struct.pack("<I", len(F)))
        for a, b, c in F:
            n = np.cross(V[b] - V[a], V[c] - V[a]); n = n / np.linalg.norm(n)
            f.write(struct.pack("<12fH", *n, *V[a], *V[b], *V[c], 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trolley-surface-z", type=float, default=0.0,
                    help="z of the trolley top (robot mounting plane) in the robot base frame [m]")
    ap.add_argument("--fixture-thickness", type=float, default=0.0125,
                    help="slotted aluminium fixture plate thickness [m] (12.5 mm from the ur3_lfd trolley mesh)")
    args = ap.parse_args()

    man = load("cad/manifest.yaml")["meshes"]
    # CAD pocket centres in the pickup plate frame (m)
    local = np.array([np.array(man[f"V{v}_moving"]["pose_in_pickup_plate"]["xyz_mm"][:2]) / 1000.0
                      for v in range(1, 6)])
    meas = np.array([tcp_xyz(f"v1_pickup_V{v}_geometry")[:2] for v in range(1, 6)])

    # 2D rigid fit (Kabsch) of local -> measured
    lc, mc = local.mean(0), meas.mean(0)
    H = (local - lc).T @ (meas - mc)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    yaw = math.atan2(R[1, 0], R[0, 0])
    origin = mc - R @ lc
    resid = meas - (local @ R.T + origin)

    ins = tcp_xyz("v1_insertion_end")
    # Placement plate: if the lab "place" poses exist (part seated in each fitting, measured with
    # the same grasp), fit xy + yaw to them like the pickup plate; otherwise keep the single
    # V1 insertion-end point and assume yaw parallel to the pickup plate.
    place = {v: f"v1_place_V{v}_geometry" for v in range(1, 6)
             if os.path.exists(os.path.join(PKG, "config", "saved_poses", f"v1_place_V{v}_geometry.yaml"))}
    place_fit = None
    if len(place) >= 2:
        lp_ = np.array([np.array(man[f"V{v}_fitting"]["pose_in_placement_plate"]["xyz_mm"][:2]) / 1000.0 for v in place])
        mp_ = np.array([tcp_xyz(n)[:2] for n in place.values()])
        lc2, mc2 = lp_.mean(0), mp_.mean(0)
        U2, _, Vt2 = np.linalg.svd((lp_ - lc2).T @ (mp_ - mc2))
        R2 = Vt2.T @ U2.T
        if np.linalg.det(R2) < 0:
            Vt2[-1] *= -1; R2 = Vt2.T @ U2.T
        o2 = mc2 - R2 @ lc2
        r2 = mp_ - (lp_ @ R2.T + o2)
        place_fit = {"origin": o2, "yaw": math.atan2(R2[1, 0], R2[0, 0]), "resid": r2, "used": list(place.values())}
    t = args.trolley_surface_z
    s = t + args.fixture_thickness            # blue plate undersides

    out = {
        "status": "PROVISIONAL - derived from TCP measurements, not a verified calibration",
        "frame": "base",
        "trolley_surface_z_m": t,
        "trolley_surface_z_source": "robot base sits on the trolley top (lab photo 30 Oct 2026)",
        "fixture_thickness_m": args.fixture_thickness,
        "fixture_thickness_source": "ur3_lfd trolley mesh, measured in Bambu Studio (12.5 mm); NOT yet measured on the real plate",
        "plate_underside_z_m": s,
        "pickup_plate": {
            "xyz_m": [round(float(origin[0]), 5), round(float(origin[1]), 5), s],
            "yaw_rad": round(yaw, 6),
            "yaw_deg": round(math.degrees(yaw), 3),
            "source": "Kabsch fit of v1_pickup_V1..V5_geometry TCP xy to CAD pocket centres",
            "fit_residual_mm": [[round(float(a) * 1000, 2), round(float(b) * 1000, 2)] for a, b in resid],
            "fit_rms_mm": round(float(np.sqrt((resid ** 2).sum(1).mean())) * 1000, 2),
        },
        "placement_plate": ({
            "xyz_m": [round(float(ins[0]), 5), round(float(ins[1]), 5), s],
            "yaw_rad": round(yaw, 6),
            "yaw_deg": round(math.degrees(yaw), 3),
            "source": "origin xy = v1_insertion_end TCP xy; yaw ASSUMED parallel to pickup plate",
        } if place_fit is None else {
            "xyz_m": [round(float(place_fit["origin"][0]), 5), round(float(place_fit["origin"][1]), 5), s],
            "yaw_rad": round(place_fit["yaw"], 6),
            "yaw_deg": round(math.degrees(place_fit["yaw"]), 3),
            "source": "Kabsch fit of " + ", ".join(place_fit["used"]) + " TCP xy to CAD fitting centres",
            "fit_residual_mm": [[round(float(a) * 1000, 2), round(float(b) * 1000, 2)] for a, b in place_fit["resid"]],
            "fit_rms_mm": round(float(np.sqrt((place_fit["resid"] ** 2).sum(1).mean())) * 1000, 2),
        }),
    }
    # ---- environment boxes (in the plates' yaw frame)
    c, sn = math.cos(yaw), math.sin(yaw)
    Rm = np.array([[c, -sn], [sn, c]])
    corners = []
    for o, (x0, x1) in ((origin, PICKUP_X), (ins[:2], PLACEMENT_X)):
        for x in (x0, x1):
            for y in (-PLATE_HALF_W, PLATE_HALF_W):
                corners.append(Rm.T @ (o + Rm @ np.array([x, y])))      # into the yaw frame
    corners = np.array(corners)
    lo = corners.min(0); hi = corners.max(0)                   # blue plates' extent in the yaw frame
    # Measured in the lab (ruler, +/- 1-2 mm): plate 495 x 320 mm; the blue plates' outer ends line up
    # with its ends; its far edge (away from the robot) is 77 mm beyond the blue plates' far edge.
    span = max(FIXTURE_LENGTH, hi[0] - lo[0])
    robot_y = (Rm.T @ np.zeros(2))[1]
    far_is_hi = abs(hi[1] - robot_y) > abs(lo[1] - robot_y)
    if far_is_hi:
        y_far = hi[1] + FIXTURE_FAR_EDGE; y_near = y_far - FIXTURE_WIDTH
    else:
        y_far = lo[1] - FIXTURE_FAR_EDGE; y_near = y_far + FIXTURE_WIDTH
    lo = np.array([(lo[0] + hi[0]) / 2 - span / 2, min(y_far, y_near)])
    hi = np.array([(lo[0]) + span, max(y_far, y_near)])
    ctr = Rm @ ((lo + hi) / 2)
    out["environment"] = {
        "fixture_plate": {"size_m": [round(float(hi[0] - lo[0]), 4), round(float(hi[1] - lo[1]), 4), args.fixture_thickness],
                          "xyz_m": [round(float(ctr[0]), 5), round(float(ctr[1]), 5), round(t + args.fixture_thickness / 2, 5)],
                          "yaw_rad": round(yaw, 6),
                          "comment": "slotted aluminium plate: 495 x 320 mm (lab ruler, +/- 1-2 mm), blue plates' outer ends at its ends, "
                                     "far edge 77 mm beyond the blue plates (thickness from the trolley mesh)"},
        "trolley_top": {"size_m": list(TROLLEY_TOP_SIZE), "xyz_m": [round(float(ctr[0]), 5), round(float(ctr[1]) + TROLLEY_TOP_DY, 5),
                                                                   round(t - 0.0005 - TROLLEY_TOP_SIZE[2] / 2, 5)],
                        "yaw_rad": round(yaw, 6),
                        "comment": "trolley top under the workspace, top 0.5 mm below the robot mounting plane"},
    }
    write_box_stl(os.path.join(PKG, "cad", "environment", "fixture_plate.stl"), out["environment"]["fixture_plate"]["size_m"])
    write_box_stl(os.path.join(PKG, "cad", "environment", "trolley_top.stl"), out["environment"]["trolley_top"]["size_m"])

    dst = os.path.join(PKG, "config", "scene", "scene_registration.yaml")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w") as f:
        f.write("# Generated by scripts/scene/compute_scene_registration.py - do not hand-edit numbers.\n")
        yaml.safe_dump(out, f, sort_keys=False)
    print(yaml.safe_dump(out, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
