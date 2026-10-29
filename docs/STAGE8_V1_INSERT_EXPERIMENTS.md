# Stage 8: V1 INSERT/ROTATE experiments A, B1, B2 (fake hardware)

Kinematic simulation on fake hardware (ROS_DOMAIN_ID=73). These are not physical results. All results are relative to the modelled fitting: `fitting_V1` is registered from the `v1_insertion_end` TCP, on the assumption that the TCP was on V1's axis (PROVISIONAL).

## Run
```bash
pip install trimesh rtree python-fcl scipy                              # once
./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh             # terminal 1
./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh            # terminal 2
export ROS_DOMAIN_ID=73
python3 scripts/phase8_insert_sim/run_v1_insert_sim.py --experiment all --run   # type START
python3 scripts/phase8_insert_sim/preview_stage8_offline.py                     # same experiments, no ROS
python3 scripts/phase8_insert_sim/test_stage8_classifier.py                     # classifier known-pose checks
```
Results: `results/simulation/stage8/<run>/{v1_original_alignment, v1_corrected_cdmp, v1_geometry_conditioned}/` with `trajectory.csv`, `collision_log.csv`, `summary.json` and `stage7_summary.json`.

## Design
Every experiment starts from a fresh Stage 7 run (pickup → lift → bridge → transport → ALIGN_INSERT). All three use the same grasp, fitting registration, tolerances and success criteria.

| | Start alignment | INSERT/ROTATE trajectory |
|---|---|---|
| **V1-A** original | as transported (2.60 mm off-axis, 3.81° tilt) | CDMP poses as learned (base frame) |
| **V1-B1** corrected CDMP | CORRECTION move: V1 upright on the fitting axis, tabs on the slot axis, same height | `T_B1(t) = T_corr,start · T_CDMP,start⁻¹ · T_CDMP(t)` (ROTATE: relative to the achieved start) |
| **V1-B2** geometry-conditioned | same CORRECTION | V1 follows the CDMP **depth** (base z) and **yaw** (about vertical) profiles along/about the fitting axis; lateral motion and tilt removed |

- **CDMP sources:** `v1_insert_cdmp_refined_depth_guard_SMOOTH_CANDIDATE.csv` (651 samples, 13.0 s) and `v1_rotate_cdmp_baseline_CANDIDATE.csv` (851 samples, 17.0 s). Original 20 ms timestamps are preserved. IK is solved per sample, seeded from the previous solution; branch jumps are rejected.
- **B1 consequence:** the rigid transform also rotates the CDMP descent direction by the 3.8° tilt correction. That is about 0.36 mm of lateral drift over 5.4 mm, and B1 tests it along with the CDMP's own lateral wander (x 0.31 mm, y 0.70 mm range).
- **CORRECTION:** min-jerk Cartesian move with IK per 20 ms sample. NEW sim timing 2.0 s, logged separately.
- **Seating floor:** descent commanded below floor contact is not executed. The TCP is held so V1's lowest point stays on the floor, and the unused commanded descent is recorded.
- **ROTATE** runs only if INSERT reached a valid seat (lowest point within 0.05 mm of the floor, centre offset ≤ 0.25 mm). Otherwise it is recorded as NOT_EXECUTED.
- **Stop rule:** a trial stops at the first unacceptable contact. Only the feasible part is executed on fake hardware.

## Contact evaluation (`stage8_core.Geometry`)
- **V1↔fitting_V1:** exact FCL mesh-mesh collision and clearance. If contact exists:
  - **Pure seating** is recognised exactly: lifting V1 by 0.05 mm along the fitting axis removes all contact. Vertical penetration is then floor − lowest point.
  - **Otherwise**, penetration depth and region are measured on dense surface samples of both meshes (~0.1 mm spacing) near the FCL contacts. This catches flat faces crossing edges, which vertex-only tests miss.
- **Regions (fitting frame):** floor, rotation_stop (ROTATE at ≥ 25.5° CW, in the channel), bore_wall, channel_side, entry_slot_or_ring_edge, rim.
- **Tolerances:** contact 0.02 mm (2× the 0.01 mm STL chord deviation); floor and rotation stop 0.05 mm.
- **Classification:** floor or stop within tolerance is intended contact. Beyond tolerance it is EXCESSIVE_PENETRATION (floor) or INTERFERENCE (stop). Any other region above 0.02 mm is INTERFERENCE.
- **All other collisions** (robot vs scene, V1 vs other objects) are checked by MoveIt on every executed sample. V1↔fitting_V1 is allowed in MoveIt only during these stages, because the classifier judges it.
- **Exact within-tolerance test (added 30 Oct, before B2R):** if shifting V1 by 0.02 mm along any fitting axis removes every FCL contact, the contact is `touching` and its penetration is that minimum shift (bisection, 0.0005 mm). Reason: the sampled signed distance uses ray-based `contains()`, which over-reports on coplanar face contact. B2R's tab-on-ring-underside contact read 0.115 mm sampled, but lowering V1 by 0.0005 mm removed it. Beyond 0.02 mm the sampled depth is still used and may overstate the depth, but not the verdict. A, B1 and B2 were re-run with the change: identical results (their first contacts do not clear within 0.02 mm).
- **Known-pose checks** (`test_stage8_classifier.py`): 15/15 correct. These cover seated, 0.03/0.10 mm into the floor, 0.24/0.40 mm off-axis, rotated 25/26/27° CW, 3° CCW, the Stage 7 arrival pose lowered to 6.9 mm (exact overlap 0.011 mm, now `touching`; it was previously expected as INTERFERENCE from the sampled depth) and to 6.5 mm, and the tab under the ring at 24.4° CW (0.49 mm high clear; 0.015 mm in `touching`; 0.05 mm in INTERFERENCE).

## Assembly success (identical for all)
No unacceptable contact, **and** seated, **and** ROTATE completed, **and** final CW rotation within 23.5–26.5°.

## Results (`results/sim_reference/stage8_20261029_233555/`)
Fake hardware and the offline preview agree exactly. Executed end states match the plans within 0.008 mm. MoveIt FK agrees with the planning FK to 0.000 mm. MoveIt found no other collisions in any experiment.

| | **V1-A** original | **V1-B1** corrected CDMP | **V1-B2** geometry-conditioned |
|---|---|---|---|
| Initial centre offset / tilt | 2.598 mm / 3.807° | 2.598 mm / 3.807° | 2.598 mm / 3.807° |
| After CORRECTION (2.0 s) | n/a | 0.0000 mm / 0.0000° | 0.0000 mm / 0.0000° |
| INSERT samples feasible / commanded | 179 / 651 | 189 / 651 | **651 / 651** |
| First unacceptable contact | INSERT t = 3.58 s, sample 179, entry slot / ring edge, 0.047 mm | INSERT t = 3.78 s, sample 189, entry slot / ring edge, 0.056 mm | **none** |
| V1 centre offset at that point | 3.01 mm (tilt 3.81°) | 0.58 mm (tilt 0.16°) | 0.000 mm throughout |
| Max insertion depth | 0.61 mm | 0.32 mm | 4.75 mm (of 5.0) |
| Min clearance / max penetration | 0 / 0.047 mm | 0 / 0.056 mm | **0.249 mm** / 0 |
| Final seating error | 4.39 mm | 4.68 mm | **0.286 mm above floor** |
| Seated | no | no | **no** (needs ≤ 0.05 mm) |
| ROTATE | NOT_EXECUTED | NOT_EXECUTED | NOT_EXECUTED |
| CDMP / correction time | 13.0 s / 0 | 13.0 s / 2.0 s | 13.0 s / 2.0 s |
| **Outcome** | **FAIL** | **FAIL** | **FAIL** |

**Interpretation (kinematic, relative to the modelled fitting):**
- **A** reproduces the physical failure mode. The tilted, off-centre part hits the entry/ring edge after 0.6 mm.
- **B1:** correcting the start alone is not enough. At failure, V1 has drifted 0.58 mm off-axis, against 0.25 mm of radial clearance. That drift is mostly the CDMP's own lateral TCP motion (−0.14, +0.46 mm by t = 3.78 s), plus 0.10 mm from its 0.16° orientation change over the 34.85 mm lever and 0.07 mm from the 3.8°-tilted descent. In the physical demo the fitting walls guided the part; a rigid kinematic replay cannot use that guidance.
- **B2:** geometry-conditioning removes the interference. V1 stays exactly centred, with 0.249 mm clearance all the way down. **But it stops 0.286 mm above the floor**, because the CDMP commands 5.42 mm of descent and V1 needs 5.70 mm from the corrected start height. That is a vertical disagreement between the grasp transform (TCP→V1 bottom 34.85 mm, from the pickup registration) and the demonstrated insertion depth. Stage 5 had already flagged it: the saved insertion_end implies 0.65 mm above the floor. Per the agreed rule, ROTATE was not executed.

**Remaining question (not adjusted):** is the 0.29 mm a grasp-height or registration error (pickup pocket floor, fitting floor, or the TCP→part z), or should the insert goal come from CAD (a CAD-defined seat, i.e. L1 goal scaling)? Either change is a method decision, and is left open.

## Follow-up B2R: ROTATE from B2's unseated end state (30 Oct)
`run_v1_insert_sim.py --experiment B2R`. This is identical to B2 (INSERT joints match the B2 reference to 5e-13 rad), but ROTATE is run although INSERT stopped 0.286 mm above the floor. The strict outcome is unchanged (FAIL: not seated). A separate **functional** criterion is applied: INSERT and ROTATE complete, no unacceptable contact, and final CW rotation within 23.5–26.5°. The seating gap is reported, not judged. Offline result: `results/sim_reference/stage8_b2r_offline_20261030/` (fake-hardware run pending).

| | B2R |
|---|---|
| ROTATE samples feasible / commanded | 851 / 851 |
| Rotation 0 → 24.4° CW (t 0–13.2 s) | clear. V1 rises 0.06 mm (CDMP), min clearance 0.157 mm |
| t 13.2–14.0 s | CDMP rises another 0.17 mm (total +0.229 mm). Tab top meets the ring underside at t = 13.82 s |
| Contact t 13.82–16.94 s (100 samples) | `touching`, tab top on ring underside, max exact overlap **0.015 mm** (tolerance 0.02) |
| Final | 24.41° CW, centred, 0.497 mm above the floor (just under the ring) |
| Rotation stop reached | no (stop at 26°; the demo rotates ~24.4°, as in the CDMP) |
| **Strict / functional outcome** | **FAIL** (not seated) / **PASS** |

**Interpretation (kinematic):** the 0.29 mm gap does not stop the locking rotation. Under the ring, V1 has 0.50 mm of vertical play (bottom z 2.00–2.50 mm); the tab rotates freely within it. At the end of the demonstrated rotation, the CDMP lifts 0.23 mm. Starting 0.29 mm high, that pulls the tab up against the ring underside: the retaining feature takes the load. The margin is small (0.015 mm overlap vs 0.02 mm tolerance). With a seated start, the same lift would leave 0.27 mm clearance. In hardware this is a real upward pull on a locked part, which compliance absorbs; a rigid simulation cannot represent that.
