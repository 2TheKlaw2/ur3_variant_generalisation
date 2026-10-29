# CAD checks — V1–V5 pickup parts, fittings and plates

Source: STLs&STEP.zip (SolidWorks export, 29 Oct 2026). All values measured from the meshes (trimesh + python-fcl).
Kinematic/geometric CAD analysis only — not physical results.

## Files and frames
- Units: mm (use scale 0.001 in ROS). SolidWorks exported Y-up; all meshes here are re-expressed **Z-up**.
- Moving part: origin on axis at bottom face, +x = tab axis.
- Fitting: origin on axis at base, +x = entry-slot axis.
- Plates: origin on the V1 axis at the plate underside, +x along the row V1→V5.
- Poses of each component in its plate frame: `manifest.yaml`.
- V1–V3 moving parts re-meshed from STEP (0.01 mm / 1°, ~6.5k tris, watertight). V4/V5 from the revised SolidWorks fine STL (319k tris, watertight).
- Placement meshes: the fine `-1-1` exports. The coarse `-1` duplicates and `Varient1.STL` (duplicate of the V1 fitting) are not used.

## Geometry
| | V1 | V2 | V3 | V4 | V5 |
|---|---|---|---|---|---|
| Part height (mm) | 25 | 25 | 30 | 30 | 40 |
| Fitting height (mm) | 7 | 7 | 12 | 12 | 22 |
| Bore depth (mm) | 5 | 5 | 10 | 10 | 20 |

- Part disc Ø27.50 in bore Ø28.00 → **0.25 mm radial clearance**. Fitting floor 2.0 mm.
- Two tabs at 180°, reaching r = 16.03 mm. Tab width: 4.80 mm (V1–V3); V4/V5 revised (not re-measured).
- Grip block 15 × 10 mm.

## Rotation (fitting frame, + = CCW viewed from above)
- **V1, V3:** seated part is collision-free from 0° to −26°, and hits the stop at −27°. So the maximum rotation is **≈26° CW**, which matches the ~25° demo.
- **V2:** mirror of V1 (CCW).
- **V4:** straight entry for about the top 4 mm, then a helix of ≈4°/mm. It seats at about **−20° (free range −16.5° to −23.5° just above the floor)**, which is less than V1.
- **V5:** straight entry (~4 mm), then a CW helix to ≈−22° at 11 mm, a short straight section (~2 mm), then a reverse helix back to **≈0° at the bottom**.

## Issues found
1. **V5 0.02 mm interference with the bore** — RESOLVED: Enrique supplied revised V4/V5 moving-part CAD (29 Oct 2026, tab span now 31.46 mm). Not re-checked, at his request.
2. **V4/V5 tabs wider than the pickup notches** — RESOLVED by the same revised CAD.
3. **Placement spacing is not uniform 50 mm:** V1→V2 50.00, V2→V3 49.75, V3→V4 50.00, V4→V5 48.99 mm. V5 is also 0.10 mm off-row. Confirmed to match the real plate (small); use the CAD positions, not nominal 50 mm.
4. **Height pattern:** V3 and V4 are both +5 mm and V5 is +15 mm, for parts and fittings. Confirmed correct; the plan table needs updating.
5. **Pickup pockets:** blind keyhole pockets 2 mm deep (floor at 1.0 mm), Ø27.8 mm. Parts rest at z = 1.0 mm. A resting contact counts as a collision in FCL, so allowed-collision entries or a small offset are needed in the scene.
6. **Pickup order:** the pickup parts were exported in part frames, so the V1 pocket is **assumed** to be the one at SolidWorks z = −20. This needs confirming.

## Plate layout
- Pickup: 200 × 70 × 3 mm. Placement: 250 × 70 × 3 mm. Each has M5 clearance holes (Ø5.5 mm) on a 50 × 50 mm pattern, which can be used for trolley registration.
- Tabs and entry slots both lie across the plate width. If the plates are parallel on the trolley, no yaw change is needed between pickup and insertion.
