"""Build cad/ folder: Z-up local frames (mm), manifest.yaml, and fit checks.
Usage: python3 -I build_cad.py <src_dir> <regen_dir> <out_cad_dir>
"""
import sys, os, shutil, numpy as np, trimesh, yaml, fcl

src, regen, out = sys.argv[1:4]
for sub in ["pickup_plate", "placement_plate", "moving_parts", "stationary_fittings", "step"]:
    os.makedirs(os.path.join(out, sub), exist_ok=True)

def frame(x, y, z, o):
    """4x4 transform mapping SolidWorks coords -> new frame given new axes (in SW coords) and origin."""
    R = np.array([x, y, z], float)            # rows = new axes
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = -R @ np.asarray(o, float)
    assert abs(np.linalg.det(R) - 1) < 1e-9
    return T

def save(mesh, T, path):
    m = mesh.copy(); m.apply_transform(T); m.export(path); return m

def load(p): return trimesh.load(p, force="mesh")

# ---------- local frames ----------
# Moving part (SW part frame: Y up, tabs along Z, origin at axis/bottom) -> x = tab axis, z = up
T_part = frame([0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 0])
# Fitting (SW assembly frame, Y up, entry slots along X) -> x = entry-slot axis, z = up, origin axis/bottom
def T_fit(ctr_x, ctr_z, ybot): return frame([1, 0, 0], [0, 0, -1], [0, 1, 0], [ctr_x, ybot, ctr_z])

# ---------- pickup plate ----------
# Basea frame: Y up, row along -Z, cutouts at z=-20..-180 (x=35.02). ASSUMPTION: V1 cutout is z=-20.
cut_z = [-19.99, -59.98, -99.99, -139.99, -179.99]; cut_x = 35.02
T_pick = frame([0, 0, -1], [-1, 0, 0], [0, 1, 0], [cut_x, 0, cut_z[0]])
pick_plate = save(load(f"{src}/Basea.STL"), T_pick, f"{out}/pickup_plate/pickup_plate.stl")

moving_src = {1: f"{regen}/V1_moving.stl", 2: f"{regen}/V2_moving.stl", 3: f"{regen}/V3_moving.stl",
              4: f"{regen}/V4_moving.stl", 5: f"{regen}/V5_moving.stl"}
moving = {}
for v, p in moving_src.items():
    moving[v] = save(load(p), T_part, f"{out}/moving_parts/V{v}_moving.stl")

# ---------- placement plate ----------
pb = load(f"{src}/PlaceAssem - Base-1-1.STL")
fits_sw = {v: load(f"{src}/PlaceAssem - Varient{v}-1-1.STL") for v in range(1, 6)}
ax = {v: (m.bounding_box.centroid[0], m.bounding_box.centroid[2], m.bounds[0][1]) for v, m in fits_sw.items()}
T_place = frame([0, 0, -1], [-1, 0, 0], [0, 1, 0], [ax[1][0], pb.bounds[0][1], ax[1][1]])
place_plate = save(pb, T_place, f"{out}/placement_plate/placement_plate.stl")
fitting = {v: save(fits_sw[v], T_fit(*ax[v]), f"{out}/stationary_fittings/V{v}_fitting.stl") for v in range(1, 6)}

for f in os.listdir(src):
    if f.upper().endswith(".STEP"): shutil.copy(os.path.join(src, f), f"{out}/step/{f}")

# ---------- poses in plate frames ----------
def pose_in(T_plate, p_sw, yaw_deg):
    p = (T_plate @ np.r_[p_sw, 1])[:3]
    return {"xyz_mm": [round(float(c), 3) for c in p], "rpy_deg": [0.0, 0.0, yaw_deg]}

# moving part local x (tabs) = SW part Z; in plate, tabs lie across the plate (plate y) -> yaw 90
# pocket centres from circle fit of keyhole pockets; parts rest on 1.0 mm pocket floor
pick_poses = {v: {"xyz_mm": [round(40.0 * (v - 1) + 0.01, 3), 0.02, 1.0], "rpy_deg": [0.0, 0.0, 90.0]} for v in range(1, 6)}
# fitting local x = SW X = -plate y -> yaw -90 (tabs/slots also across the plate)
place_poses = {v: pose_in(T_place, [ax[v][0], ax[v][2], ax[v][1]], -90.0) for v in range(1, 6)}

# ---------- fit checks (part seated in fitting, local fitting frame) ----------
def bvh(m):
    g = fcl.BVHModel(); g.beginModel(len(m.vertices), len(m.faces))
    g.addSubModel(m.vertices, m.faces); g.endModel(); return g
def min_dist(a, Ta, b):
    oa = fcl.CollisionObject(bvh(a), fcl.Transform(Ta[:3, :3], Ta[:3, 3]))
    ob = fcl.CollisionObject(bvh(b), fcl.Transform())
    r = fcl.DistanceResult(); fcl.distance(oa, ob, fcl.DistanceRequest(), r)
    c = fcl.CollisionResult(); fcl.collide(oa, ob, fcl.CollisionRequest(num_max_contacts=1), c)
    return r.min_distance, c.is_collision
def Tz(z, yaw):
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw)); T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]; T[2, 3] = z; return T

floor = 2.0  # fitting floor thickness (bore starts 2 mm above fitting base)
report = {}
for v in range(1, 6):
    top = float(fitting[v].bounds[1][2]); depth = top - floor
    rows = []
    for yaw in [0, -10, -20, -25, -26, -27, -30, 10]:
        for z in [floor + 0.05, top + 0.5]:
            d, col = min_dist(moving[v], Tz(z, yaw), fitting[v])
            rows.append({"yaw_deg": yaw, "part_bottom_mm": round(z, 2), "min_dist_mm": round(d, 3), "collision": bool(col)})
    report[v] = {"bore_depth_mm": round(depth, 2), "checks": rows}

manifest = {
    "units": "mm (STL); scale 0.001 for ROS",
    "axes": "all meshes Z-up. Moving part: origin on axis at bottom face, +x = tab axis. "
            "Fitting: origin on axis at base, +x = entry-slot axis. Plates: origin on V1 axis at plate underside, "
            "+x along row V1->V5, +z up.",
    "assumptions": ["Pickup: V1 is the cutout at SolidWorks z=-20 (not in CAD; parts exported in part frames)",
                    "Pickup parts rest on the 1.0 mm floor of the 2 mm-deep keyhole pockets",
                    "Plate-to-robot transforms NOT included; placement row provisional"],
    "meshes": {
        "pickup_plate": {"file": "pickup_plate/pickup_plate.stl", "source": "Basea.STL (SolidWorks)"},
        "placement_plate": {"file": "placement_plate/placement_plate.stl", "source": "PlaceAssem - Base-1-1.STL"},
        **{f"V{v}_moving": {"file": f"moving_parts/V{v}_moving.stl",
                            "source": ("Varient%da.STEP via OpenCascade, 0.01 mm / %d deg" % (v, 1 if v <= 3 else 5)),
                            "triangles": int(len(moving[v].faces)), "watertight": bool(moving[v].is_watertight),
                            "pose_in_pickup_plate": pick_poses[v]} for v in range(1, 6)},
        **{f"V{v}_fitting": {"file": f"stationary_fittings/V{v}_fitting.stl",
                             "source": f"PlaceAssem - Varient{v}-1-1.STL (assembly frame)",
                             "triangles": int(len(fitting[v].faces)), "watertight": bool(fitting[v].is_watertight),
                             "pose_in_placement_plate": place_poses[v]} for v in range(1, 6)},
    },
}
with open(f"{out}/manifest.yaml", "w") as f: yaml.safe_dump(manifest, f, sort_keys=False)
with open(f"{out}/fit_checks.yaml", "w") as f: yaml.safe_dump(report, f, sort_keys=False)

for v in range(1, 6):
    p = pick_poses[v]["xyz_mm"]; T = Tz(0, 90); T[:3, 3] = p
    d, col = min_dist(moving[v], T, pick_plate); print(f"pickup V{v} seated: min_dist={d:.3f} collision={col}")
