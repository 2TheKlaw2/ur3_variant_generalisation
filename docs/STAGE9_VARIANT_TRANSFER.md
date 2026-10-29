# Stage 9: one V1 demonstration transferred to V1–V5 (L0 / L1 / L2), offline

**Kinematic simulation, offline (no ROS, no MoveIt).** These are not physical results. Only the INSERT and ROTATE phases are simulated, from a common aligned start above each fitting. Pickup and transport for V2–V5 are not simulated yet. Robot-vs-scene collisions are not checked here (that needs MoveIt). Only part ↔ fitting contact is checked. Fitting registration is PROVISIONAL, as in Stages 5–8. This was run overnight on 30 Oct without Enrique; the level definitions below are my working definitions and are open to his review.

## Run
```bash
python3 scripts/phase9_variants/cad_corridor.py           # CAD feature extraction -> results/cad_features/
python3 scripts/phase9_variants/run_variants_offline.py   # 5 variants x 3 levels -> results/simulation/stage9_offline/
python3 scripts/figures/make_stage9_figures.py            # docs/figures/stage9_*.png|svg
```
Reference results: `results/sim_reference/stage9_offline_20261030/` (`V<i>_<level>/{trajectory.csv, collision_log.csv, summary.json}`, `stage9_summary.json`).

## Common to all conditions
- **Start:** the Stage 8 B2 state after CORRECTION, transferred to each fitting.
  - The part is upright and centred on the fitting axis, with tabs on the slot axis.
  - Part yaw is 180° in the fitting frame, as in Stage 8. All five fittings are 2-fold symmetric (checked: 0/300 mismatches each).
  - The part bottom is 0.703 mm above the fitting top.
  - The robot start is found by IK continuation from the V1 start.
- **Grasp:**
  - V1 uses the Stage 6–8 grasp.
  - V2–V5 use the measured `v1_pickup_V<i>_geometry` poses.
  - The CORRECTION already removes grasp tilt and offset, so grasp only affects reachability.
- **Demonstration:**
  - INSERT CDMP (13.0 s, 5.417 mm descent).
  - ROTATE CDMP (17.0 s, −24.39°, and a 0.23 mm rise at the end).
  - Both are applied along and about the fitting axis as in B2 (lateral motion and tilt removed). Original timestamps are kept.
- **Contact:** the Stage 8 classifier (`VariantGeometry`):
  - exact FCL;
  - `touching` if a 0.02 mm single-axis shift clears the contact;
  - floor tolerance 0.05 mm;
  - otherwise INTERFERENCE.
  - A trial stops at the first unacceptable contact.
  - ROTATE runs if INSERT completed, seated or not (the B2R protocol).
- **IK:** per sample, seeded by the previous sample.
  - The continuity limit is 0.1 rad per 20 ms. Stage 8 used 0.02, but the scaled V5 descent legitimately moves wrist 3 faster.
  - Joint speeds are then checked against the UR3e limits (180°/s for the base, shoulder and elbow; 360°/s for the wrists). The peak was 26% of the limit (V5 L2).

## CAD feature extraction (`cad_corridor.py`, no demonstration data)
For each variant, the part is placed centred and upright. For every 0.1 mm of height, from the start down to the floor, yaw is swept ±45° and checked with exact FCL using the classifier's 0.02 mm tolerance. Edges are refined to 0.05°.

| | V1 | V2 | V3 | V4 | V5 |
|---|---|---|---|---|---|
| Fitting top / floor (mm) | 7 / 2 | 7 / 2 | 12 / 2 | 12 / 2 | 22 / 2 |
| Seat depth from start (mm) | 5.70 | 5.70 | 10.70 | 10.70 | 20.70 |
| Straight entry (yaw 0 free) down to bottom z | floor | floor | floor | 6.2 mm | 16.3 mm |
| Seat corridor (°, + = CCW) | −26.5 … +1.3 | −1.3 … +26.5 | −26.4 … +1.3 | −25.7 … −17.1 | **closed** |
| CAD seat yaw (target) | −24.39 | +24.39 | −24.33 | −23.58 | — |

**Seat-yaw rule:** take the seat-corridor end in the rotation direction, less the margin the V1 demo kept from V1's lock end (26.5 − 24.39 = 2.11°), but not past the corridor centre. For V1 this reproduces the demonstrated angle.

**V5 finding (not adjusted):** the free corridor follows the CW helix (about 3.7°/mm) and then closes at bottom z = 13.9 mm, 8.1 mm into a 20 mm bore. No yaw within ±45° is free below that height, even allowing 0.5 mm single-axis shifts.
- At those heights, the only overlap is the part's disc (r = 13.75 mm) against the V5 bore. On opposite sides, the bore narrows to r ≈ 13.73 mm, so about 0.02 mm per side, i.e. diametral.
- This is CAD issue #1 ("V5 0.02 mm interference with the bore"). It was marked resolved by the revised V5 part, which was not re-checked at Enrique's request. The revised part changed the tabs, not the Ø27.50 disc, so the interference remains.
- The V4 bore is r 14.00 throughout and the V4 corridor reaches the floor.
- **This needs checking in SolidWorks** (V5 bore diameter about 9–15 mm above the fitting base, where sections give r 13.73–13.77 mm). If the real fitting is Ø28, the V5 STL export is the cause.

## Conditions
| | INSERT | ROTATE |
|---|---|---|
| **L0** no adaptation | V1 depth profile as demonstrated (5.42 mm) | V1 yaw profile (−24.39°) and rise |
| **L1** CAD goal scaling | Demo depth profile scaled to the CAD seat depth (DMP spatial-scaling property, ×D_cad/D_demo). Yaw as demonstrated (≈0) | Demo yaw profile scaled to the CAD seat yaw (sign from CAD) |
| **L2** CAD geometry-conditioned | Depth as L1. Yaw follows the CAD corridor as a function of depth: hold the yaw while it is ≥1° inside the free interval, otherwise move the minimum amount; a jump of more than 1° per 0.1 mm means the corridor is broken | From the corridor yaw at the seat to the CAD seat yaw, using the demo's yaw progress and rise |

**Functional PASS** requires all of:
- no unacceptable contact;
- INSERT and ROTATE complete;
- final yaw within 1.5° of the CAD seat yaw;
- **retained**: lifting the part 1 mm straight up collides, so it is locked;
- within the UR3e joint speed limits.

**Strict PASS** additionally requires the part to be seated within 0.05 mm at the end of INSERT.

## Results (`docs/figures/stage9_outcomes.png`, `stage9_corridors.png`)
| | L0 | L1 | L2 |
|---|---|---|---|
| **V1** | functional **PASS**, strict FAIL (= B2R: 0.497 mm gap, 24.41° CW) | **PASS** (seated, 24.41° CW) | **PASS** (24.39° CW) |
| **V2** (mirror) | FAIL: ROTATE goes CW, interference 0.043 mm at −1.5° | **PASS** (24.36° CCW) | **PASS** (24.39° CCW) |
| **V3** (deeper) | FAIL: only 5.45/10.70 mm deep; ROTATE interference at −1.5° | **PASS** (seated, 24.35° CW) | **PASS** |
| **V4** (helix) | FAIL: 5.45/10.70 mm; ROTATE hits the bore wall at −5.1° | FAIL: straight descent hits the helix at 6.39 mm (0.113 mm) | **PASS** (seated, 23.58° CW, follows the helix) |
| **V5** (helix out and back) | FAIL: 5.45/20.70 mm; ROTATE hits at −5.1° | FAIL: straight descent stops at 6.14 mm (0.026 mm) | FAIL: follows the helix to 8.85 mm, then the bore-narrowing obstruction at bottom z 13.46 mm (CAD finding above) |

- **V1 L0 reproduces Stage 8 B2/B2R exactly:** INSERT and ROTATE joints match the references to 6e-13 rad.
- On V1, the "final gap 0.211 mm" for L1 and L2 is the demo's 0.21 mm rise at the end of ROTATE. Both were seated at the end of INSERT. L1's CAD seat-depth goal removes B2's 0.29 mm shortfall.
- **Penetration depths above 0.02 mm** come from the sampled estimate. They can overstate the depth, but not the verdict.

**Reading (kinematic, relative to the modelled fittings):**
- Without adaptation (L0), the V1 demo assembles only V1.
- CAD goal scaling (L1) is enough for variants that differ only in depth or rotation direction (V2, V3). It fails where the path shape differs (V4 helix).
- Conditioning the motion on the CAD corridor (L2) also assembles V4.
- V5 cannot be judged until the bore/disc interference is resolved in CAD. With the current meshes, no condition can pass V5.

## Not done / next
- Pickup → lift → transport for V2–V5: the reach and transport CDMPs need retargeting to each pickup pocket and fitting.
- MoveIt robot-vs-scene check and a fake-hardware run of these INSERT/ROTATE paths.
- A CAD-only baseline (the L2 path with min-jerk timing, no demo), for the "ordinary CDMP vs geometry-conditioned CDMP vs CAD-only" comparison in `PROJECT_BOUNDARY.md`.
