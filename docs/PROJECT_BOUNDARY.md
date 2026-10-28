# Research boundary & milestones

## This is the NEW project

- The moving part has rectangular opposing tabs and a gripping block.
- The five stationary circular fittings have grooves and are mounted to one plate.
- One V1 freedrive demonstration: insert 5 mm, rotate clockwise 10 degrees about central axis.
- V2: same insertion, anticlockwise 10 degrees.
- V3: insert 10 mm, clockwise 10 degrees.
- V4: insert 10 mm, anticlockwise 10 degrees.
- V5: insert 5 mm then clockwise 10 degrees **while descending 5 mm**.
- Fittings V3–V5 are 5 mm taller than V1; V2 is same top height as V1.
- Measured moving-tab centre radius: 28.8 mm. V5 10° sweep means ~5.027 mm arc, ~7.09 mm path with 5 mm descent.

## Not the old project

Do not label old `variant_slide_in.yaml` etc as these new V1–V5. Do not overwrite `~/ros2_ws/src/ur3_lfd/demonstrations/lab_work/` or `ur3_lfd/config/final_home.yaml`.

## Planned implementation

1. Validate synthetic V1 linear insertion + axial rotation in C++ offline.
2. Read measured CAD/fixture geometry and compute per-variant spatial/time mapping.
3. Train/replay reference V1 demonstration after safe lab collection.
4. Adapt reference phases using CDMPs; correct orientation changes, not just XYZ.
5. Validate pin and full-gripper clearance, planner continuity; simulation before real hardware.
6. Compare ordinary CDMP, geometry-conditioned CDMP and CAD-only baseline, with stated control conditions.

**Code caveat:** Existing `ur3_lfd::CartesianDMP` trains XYZ DMPs but currently interpolates stored orientations using normalised quaternion blending. It does NOT yet train an orientation DMP or automatically reverse the learned rotation. The NEW research implementation must handle this explicitly.
