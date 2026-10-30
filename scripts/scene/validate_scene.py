#!/usr/bin/env python3
"""Offline validation of the static V1-V5 RViz scene (no ROS runtime needed).

Checks the EXPANDED scene URDF (exactly what RViz shows) against CAD and the
measured lab poses, and places the real RG2 collision meshes with UR3e FK from
the lab calibration to check finger clearance at every measured pickup pose.

  python3 scripts/scene/validate_scene.py --urdf /tmp/variant_scene.urdf \
      --rg2-mesh-dir $(ros2 pkg prefix onrobot_description)/share/onrobot_description/meshes/rg2/collision \
      --out results/scene_check

If --urdf is omitted the xacro is expanded with the xacro Python API.
Kinematic/geometric checks only: not physical results.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TCP_TO_FINGER_END = 0.0148   # RG2 inner_finger mesh extends 14.8 mm past gripper_tcp
GRIP_W, OPEN_W = 0.0082, 0.040  # physical RG2 settings (run_v1_physical_sequence.py)

results = []


# Gaps (slots) of the aluminium fixture plate, measured from its far edge (away from the robot):
#   first gap edge 30 mm from the far edge on the REAL plate (ruler, 30 Oct 2026; the ur3_lfd trolley
#   CAD says 28 mm - do not use it), gap pitch 50 mm (confirmed). Gap width 6 mm (measured).
SLOT_FIRST_EDGE_MM = 30.0
SLOT_PITCH_MM = 50.0
SLOT_WIDTH_MM = 6.0         # measured on the real plate (30 Oct 2026)
SLOT_LINES_MM = tuple(SLOT_FIRST_EDGE_MM + SLOT_WIDTH_MM / 2 + k * SLOT_PITCH_MM for k in range(4))
SLOT_TOL_MM = 0.5           # hole centre vs gap centre
HOLE_R = 0.00275            # Ø5.5 mm clearance holes in the blue plates
BOLT_D_MM = 5.0             # M5


def check(name, ok, detail):
    tag = "INFO" if ok is None else ("PASS" if ok else "FAIL")
    results.append((tag, name, detail))
    print(f"[{tag}] {name}: {detail}")


# ---------------------------------------------------------------- transforms
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T(xyz=(0, 0, 0), r=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = rpy(*r); M[:3, 3] = xyz; return M


def rot_axis(axis, a):
    return T(r=(0, 0, 0)) if a == 0 else _axis_angle(np.asarray(axis, float), a)


def _axis_angle(k, a):
    k = k / np.linalg.norm(k); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    M = np.eye(4); M[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; return M


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---------------------------------------------------------------- scene URDF
def expand_xacro():
    import xacro
    doc = xacro.process_file(os.path.join(PKG, "urdf", "variant_scene.urdf.xacro"))
    return doc.toprettyxml()


def scene_links(urdf_xml):
    root = ET.fromstring(urdf_xml)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0, 0, 0]
        r = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0, 0, 0]
        joints[j.find("child").get("link")] = (j.find("parent").get("link"), T(xyz, r))
    world = {}

    def pose(link):
        if link not in joints: return np.eye(4)
        if link not in world:
            p, M = joints[link]; world[link] = pose(p) @ M
        return world[link]

    meshes = {}
    for l in root.findall("link"):
        m = l.find("visual/geometry/mesh")
        if m is None: continue
        fn = m.get("filename").replace("package://ur3_variant_generalisation/", PKG + "/")
        s = [float(v) for v in m.get("scale", "1 1 1").split()]
        mesh = trimesh.load(fn, force="mesh"); mesh.apply_scale(s); mesh.apply_transform(pose(l.get("name")))
        meshes[l.get("name")] = mesh
    return meshes, {k: pose(k) for k in joints}


# ---------------------------------------------------------------- UR3e + RG2 FK
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ur_kinematics import ur_fk  # noqa: E402  (shared calibrated FK)


def rg2_links(rg2_base, width):
    th = 0.85 * ((-0.558505 - 0.785398) / 0.110) * width + 0.785398
    L = {"base_link": rg2_base}
    lo = rg2_base @ T((0, -0.017178, 0.125797)) @ _axis_angle([-1, 0, 0], th)
    L["l_outer_knuckle"] = lo
    L["l_inner_finger"] = lo @ T((0, -(0.056770 - 0.017178), 0.163974 - 0.125797)) @ _axis_angle([1, 0, 0], th)
    L["l_inner_knuckle"] = rg2_base @ T((0, -0.007678, 0.142297)) @ _axis_angle([1, 0, 0], -th)
    ro = rg2_base @ T((0, 0.017178, 0.125797), (0, 0, math.pi)) @ _axis_angle([1, 0, 0], -th)
    L["r_outer_knuckle"] = ro
    L["r_inner_finger"] = ro @ T((0, -(0.056770 - 0.017178), 0.163974 - 0.125797)) @ _axis_angle([1, 0, 0], th)
    L["r_inner_knuckle"] = rg2_base @ T((0, 0.007678, 0.142297), (0, 0, -math.pi)) @ _axis_angle([1, 0, 0], -th)
    return L


def rg2_meshes(rg2_base, width, mesh_dir):
    out = {}
    for name, M in rg2_links(rg2_base, width).items():
        f = name.split("_", 1)[1] if name != "base_link" else "base_link"
        m = trimesh.load(os.path.join(mesh_dir, f + ".stl"), force="mesh"); m.apply_transform(M); out[name] = m
    return out


# ---------------------------------------------------------------- collision helpers
def _bvh(m):
    import fcl
    g = fcl.BVHModel(); g.beginModel(len(m.vertices), len(m.faces)); g.addSubModel(m.vertices, m.faces); g.endModel()
    return fcl.CollisionObject(g, fcl.Transform())


def dist(a, b):
    import fcl
    r = fcl.DistanceResult(); fcl.distance(_bvh(a), _bvh(b), fcl.DistanceRequest(), r)
    c = fcl.CollisionResult(); fcl.collide(_bvh(a), _bvh(b), fcl.CollisionRequest(), c)
    return (0.0 if c.is_collision else r.min_distance), c.is_collision


def lifted(m, dz):
    m = m.copy(); m.apply_translation([0, 0, dz]); return m


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf"); ap.add_argument("--rg2-mesh-dir"); ap.add_argument("--out", default=os.path.join(PKG, "results", "scene_check"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    xml = open(a.urdf).read() if a.urdf else expand_xacro()
    meshes, poses = scene_links(xml)
    reg = yaml.safe_load(open(os.path.join(PKG, "config/scene/scene_registration.yaml")))
    calib = yaml.safe_load(open(os.path.join(PKG, "config/ur3e_lab_calibration.yaml")))
    s = reg.get("plate_underside_z_m", reg["trolley_surface_z_m"])
    sp = lambda n: yaml.safe_load(open(os.path.join(PKG, "config/saved_poses", n + ".yaml")))

    # ---- A. dimensions and units (mm-accurate extents in metres)
    # expected local extents (x = tab/slot axis, y, z) in metres; part tab span differs (V4/V5 revised)
    exp = {"pickup_plate": (0.200, 0.143, 0.003), "placement_plate": (0.250, 0.143, 0.003)}   # 141 = 70 + 71 mm prongs (30 Oct)
    hts_p, hts_f, span = [25, 25, 30, 30, 40], [7, 7, 12, 12, 22], [32.06, 32.06, 32.06, 31.46, 31.46]
    for i in range(5):
        exp[f"pickup_V{i+1}"] = (span[i] / 1000, 0.0275, hts_p[i] / 1000)
        exp[f"fitting_V{i+1}"] = (0.040, 0.040, hts_f[i] / 1000)
    for n, e_exp in exp.items():
        local = meshes[n].copy(); local.apply_transform(np.linalg.inv(poses[n])); e = local.extents
        ok = all(abs(v - w) < 2e-4 for v, w in zip(e, e_exp))
        check(f"size {n}", ok, f"{e[0]*1000:.2f} x {e[1]*1000:.2f} x {e[2]*1000:.2f} mm "
                               f"(expected {e_exp[0]*1000:.2f} x {e_exp[1]*1000:.2f} x {e_exp[2]*1000:.2f})")

    # ---- B. heights in base frame
    for i in range(1, 6):
        zb = meshes[f"pickup_V{i}"].bounds[0][2]; zf = meshes[f"fitting_V{i}"].bounds[0][2]
        check(f"seat pickup_V{i}", abs(zb - (s + 0.001)) < 1e-5, f"part bottom z = {zb*1000:.2f} mm (pocket floor = surface + 1.00)")
        check(f"seat fitting_V{i}", abs(zf - (s + 0.003)) < 1e-5, f"fitting base z = {zf*1000:.2f} mm (plate top = surface + 3.00)")

    # ---- C. static collisions (lift 0.02 mm to ignore exact resting contacts)
    names = list(meshes)
    for n in [k for k in names if k.startswith(("pickup_V", "fitting_V"))]:
        plate = meshes["pickup_plate" if n.startswith("pickup") else "placement_plate"]
        d, col = dist(lifted(meshes[n], 2e-5), plate)
        check(f"no penetration {n} vs plate", not col, f"min dist {d*1000:.3f} mm (resting contact excluded)")
    d, col = dist(meshes["pickup_plate"], meshes["placement_plate"])
    check("plates do not overlap", not col, f"gap {d*1000:.1f} mm")

    # ---- C2. revised plates: prongs on the fixture far edge, bolt holes over the slot lines
    if "fixture_plate" in meshes:
        Fi = np.linalg.inv(poses["fixture_plate"]); fx = meshes["fixture_plate"].copy(); fx.apply_transform(Fi)
        robot_local = (Fi @ np.array([0, 0, 0, 1.0]))[1]
        far = fx.bounds[1][1] if abs(fx.bounds[1][1] - robot_local) > abs(fx.bounds[0][1] - robot_local) else fx.bounds[0][1]
        sgn = 1.0 if far > robot_local else -1.0
        rows_all = {}
        for key in ("pickup_plate", "placement_plate"):
            m = meshes[key].copy(); m.apply_transform(Fi)
            tip = m.bounds[1][1] if sgn > 0 else m.bounds[0][1]
            mode = reg.get("layout", "measured")
            check(f"prongs on far edge {key}", abs(tip - far) < 5e-4 or mode != "prongs",
                  f"prong tip {abs(far - tip)*1000:.2f} mm from the fixture far edge (layout: {mode})")
            sec = m.section([0, 0, 1], [0, 0, m.bounds[0][2] + 0.0015])
            holes = []
            for e in sec.discrete:
                c = e[:, :2].mean(0); r = np.linalg.norm(e[:, :2] - c, axis=1)
                if abs(r.mean() - HOLE_R) < 2e-4 and r.std() < 1e-4:
                    holes.append(c)
            dists = sorted(round(float(sgn * (far - c[1])) * 1000, 2) for c in holes)
            off = [min(abs(dd - L) for L in SLOT_LINES_MM) for dd in dists]
            rows_all[key] = dists
            # does an M5 bolt through each hole drop into a gap without touching the rails?
            def bolt_fits(dd):
                return any(L - SLOT_WIDTH_MM / 2 <= dd - BOLT_D_MM / 2 and dd + BOLT_D_MM / 2 <= L + SLOT_WIDTH_MM / 2
                           for L in SLOT_LINES_MM)
            nfit = sum(bolt_fits(dd) for dd in dists)
            check(f"holes on slot lines {key}", len(holes) > 0 and max(off) <= SLOT_TOL_MM,
                  f"{len(holes)} x Ø{2*HOLE_R*1000:.1f} holes at {dists} mm from the far edge; gap centres "
                  f"{[round(v, 1) for v in SLOT_LINES_MM[:3]]} mm (first edge {SLOT_FIRST_EDGE_MM:.0f}, pitch "
                  f"{SLOT_PITCH_MM:.0f}, width {SLOT_WIDTH_MM:.0f}); worst hole-to-gap-centre {max(off or [0]):.2f} mm; "
                  f"M5 bolt clears the rails in {nfit}/{len(holes)} holes")
    for i in range(1, 5):
        for kind in ("pickup", "fitting"):
            d, col = dist(meshes[f"{kind}_V{i}"], meshes[f"{kind}_V{i+1}"])
            check(f"gap {kind}_V{i}-V{i+1}", not col, f"{d*1000:.2f} mm")

    # ---- D. order and orientation vs measured robot data
    # Taught TCP poses belong to the layout they were taught in (measured), so in the predicted
    # "prongs" layout compare them with the plates' measured positions and report the shift separately.
    shift = {}
    for key in ("pickup_plate", "placement_plate"):
        r = reg[key]
        shift[key] = np.array(r["xyz_m"][:2]) - np.array(r.get("measured_xyz_m", r["xyz_m"])[:2])
        if np.linalg.norm(shift[key]) > 0:
            check(f"predicted re-teach shift {key}", True, f"{np.linalg.norm(shift[key])*1000:.2f} mm away from the robot "
                  "(taught poses compared at the measured layout below; re-teach/register in the lab)")
    def axis_xy(n):
        return poses[n][:2, 3] - shift["pickup_plate" if n.startswith("pickup") else "placement_plate"]
    def xdir(n): return poses[n][:3, 0]
    for i in range(1, 6):
        tcp = sp(f"v1_pickup_V{i}_geometry")["tcp"]
        off = np.array(tcp["position_m"][:2]) - axis_xy(f"pickup_V{i}")
        check(f"TCP over pickup_V{i}", np.linalg.norm(off) < 0.004, f"TCP xy offset from part axis {np.linalg.norm(off)*1000:.2f} mm")
        Rg = quat_to_R(tcp["quaternion_xyzw"])
        ang = math.degrees(math.acos(min(1, abs(Rg[:, 1] @ xdir(f"pickup_V{i}")))))
        check(f"finger direction vs 10 mm block faces V{i}", ang < 5, f"gripper closing axis vs part tab axis: {ang:.2f} deg")
        z = tcp["position_m"][2]; fb = z - TCP_TO_FINGER_END
        top = meshes[f"pickup_V{i}"].bounds[1][2]; disc = s + 0.001 + (0.005 if i <= 2 else 0.010)
        check(f"finger height V{i}", fb > disc and fb < top,
              f"finger end {fb*1000:.1f} mm, {(fb-disc)*1000:.1f} mm above disc, engages top {(top-fb)*1000:.1f} mm of block")
    ang = math.degrees(math.acos(min(1, abs(xdir("pickup_V1") @ xdir("fitting_V1")))))
    check("pickup tab axis parallel to fitting slot axis", ang < 1, f"{ang:.2f} deg -> no yaw change needed for insertion")
    xs = [axis_xy(f"pickup_V{i}")[0] for i in range(1, 6)] + [axis_xy(f"fitting_V{i}")[0] for i in range(1, 6)]
    check("row order V1..V5 (robot-side left to right = decreasing base x)", all(np.diff(xs[:5]) < 0) and all(np.diff(xs[5:]) < 0),
          f"pickup x {[round(v*1000) for v in xs[:5]]} mm, fitting x {[round(v*1000) for v in xs[5:]]} mm")
    ready, end, done = (sp(n)["tcp"] for n in ("v1_insertion_ready", "v1_insertion_end", "v1_assembled_pose"))
    off = np.array(ready["position_m"][:2]) - axis_xy("fitting_V1")
    check("insertion-ready TCP over fitting_V1", np.linalg.norm(off) < 0.003, f"{np.linalg.norm(off)*1000:.2f} mm")
    grip_h = sp("v1_pickup_V1_geometry")["tcp"]["position_m"][2] - (s + 0.001)
    seat = end["position_m"][2] - grip_h
    floor = s + 0.003 + 0.002
    check("V1 seated depth consistency", abs(seat - floor) < 0.0015,
          f"implied part bottom {seat*1000:.2f} mm vs fitting floor {floor*1000:.2f} mm (diff {(seat-floor)*1000:+.2f} mm)")
    yaw = lambda q: math.degrees(math.atan2(quat_to_R(q)[1, 0], quat_to_R(q)[0, 0]))
    dyaw = (yaw(done["quaternion_xyzw"]) - yaw(end["quaternion_xyzw"]) + 180) % 360 - 180
    check("V1 rotation sense and size", -27 < dyaw < -20, f"measured {dyaw:.1f} deg (CW); CAD stop at 26 deg CW")

    # ---- E. robot FK + RG2 finger clearance (needs RG2 meshes)
    fk_err = []
    for n in sorted(os.listdir(os.path.join(PKG, "config/saved_poses"))):
        d_ = sp(n[:-5])
        if d_.get("tool_frame") != "gripper_tcp": continue
        _, tcp = ur_fk(d_["arm"]["positions_rad"], calib)
        fk_err.append(np.linalg.norm(tcp[:3, 3] - np.array(d_["tcp"]["position_m"])) * 1000)
    check("UR3e+RG2 FK reproduces recorded TCP", max(fk_err) < 1.0, f"max {max(fk_err):.3f} mm over {len(fk_err)} saved poses")
    finger_sets = {}
    if a.rg2_mesh_dir and os.path.isdir(a.rg2_mesh_dir):
        for i in range(1, 6):
            q = sp(f"v1_pickup_V{i}_geometry")["arm"]["positions_rad"]; base_, _ = ur_fk(q, calib)
            for w, lab in ((OPEN_W, "open 40 mm"), (GRIP_W, "grip 8.2 mm")):
                g = rg2_meshes(base_, w, a.rg2_mesh_dir); finger_sets[(i, lab)] = g
                fingers = trimesh.util.concatenate([g[k] for k in g if "finger" in k or "knuckle" in k])
                nb = [j for j in (i - 1, i + 1) if 1 <= j <= 5]
                dn = min(dist(fingers, meshes[f"pickup_V{j}"])[0] for j in nb)
                dp = dist(fingers, meshes["pickup_plate"])[0]
                own, own_col = dist(fingers, meshes[f"pickup_V{i}"])
                check(f"RG2 at pickup_V{i} ({lab})", dn > 0.002 and dp > 0.001,
                      f"to neighbours {dn*1000:.1f} mm, to plate {dp*1000:.1f} mm, own part {'contact' if own_col else f'{own*1000:.1f} mm'}")
    else:
        check("RG2 finger clearance", None, "skipped (pass --rg2-mesh-dir)")

    render(meshes, finger_sets, sp, a.out)
    n_fail = sum(1 for r in results if r[0] == "FAIL")
    with open(os.path.join(a.out, "scene_check.txt"), "w") as f:
        f.write("Static scene validation (kinematic/geometric only; PROVISIONAL registration)\n")
        for r in results: f.write(f"[{r[0]}] {r[1]}: {r[2]}\n")
    print(f"\n{len(results)} checks, {n_fail} FAIL -> {a.out}")
    return 1 if n_fail else 0


def render(meshes, fingers, sp, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    col = lambda n: "#8a8f98" if "plate" in n else ("#2f5fe0" if n.startswith("pickup_V") else "#1f3fb0")
    fig, axs = plt.subplots(2, 1, figsize=(12, 9), gridspec_kw={"height_ratios": [1.3, 1]})
    for ax, (i, j), title in ((axs[0], (0, 1), "Top view (base frame) as seen from the robot side, robot at bottom"), (axs[1], (0, 2), "Side view from the robot side (base frame)")):
        order = sorted(meshes, key=lambda n: 0 if "plate" in n else 1)
        for n in order:
            m = meshes[n]; P = m.triangles[:, :, [i, j]] * 1000
            ax.add_collection(PolyCollection(P, facecolor=col(n), edgecolor="none", alpha=0.9 if "plate" not in n else 0.5))
        if fingers:
            g = fingers[(1, "grip 8.2 mm")]
            for k, m in g.items():
                ax.add_collection(PolyCollection(m.triangles[:, :, [i, j]] * 1000, facecolor="#e07b20", edgecolor="none", alpha=0.35))
        for v in range(1, 6):
            p = np.array(sp(f"v1_pickup_V{v}_geometry")["tcp"]["position_m"]) * 1000
            ax.plot(p[i], p[j], "r+", ms=10, mew=1.5)
            c = meshes[f"pickup_V{v}"].bounds.mean(0) * 1000; ax.text(c[i], (c[j] - 28 if j == 1 else -8), f"V{v}", ha="center", fontsize=9)
            c = meshes[f"fitting_V{v}"].bounds.mean(0) * 1000; ax.text(c[i], (c[j] - 28 if j == 1 else -8), f"V{v}", ha="center", fontsize=9)
        for nme in ("v1_insertion_ready", "v1_insertion_end"):
            p = np.array(sp(nme)["tcp"]["position_m"]) * 1000; ax.plot(p[i], p[j], "mx", ms=8)
        ax.set_aspect("equal"); ax.autoscale(); ax.set_title(title, fontsize=10)
        ax.set_xlabel("base x [mm]"); ax.set_ylabel("base y [mm]" if j == 1 else "base z [mm]"); ax.grid(alpha=0.3)
    # Viewer stands at the robot (y = 0) looking toward -y: +x on the left, robot side at the bottom.
    axs[0].invert_xaxis(); axs[0].invert_yaxis(); axs[1].invert_xaxis()
    if fingers: axs[1].set_ylim(-15, 140)
    fig.text(0.01, 0.005, "red + = measured pickup TCPs, magenta x = insertion-ready/end TCPs, orange = RG2 at V1 grasp (FK from lab joints)", fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(out, "scene_check.png"), dpi=120)


if __name__ == "__main__":
    sys.exit(main())
