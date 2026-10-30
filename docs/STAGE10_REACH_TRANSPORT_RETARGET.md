# Stage 10: REACH and TRANSPORT retargeted to V1–V5 with the CDMP (offline)

`scripts/phase10_full_sequence/retarget_reach_transport.py` (kinematic simulation only, no ROS, no robot).

**Method.** Victor Hernandez-Moreno's `CartesianDMPs` (GPLv3, loaded from `--library`, not copied into this
repo). One CDMP is trained per skill on the single V1 demonstration (`v1_reach_001_motion`, `v1_transport_001_motion`;
alpha_s 7, K 100, 4 RBF/s, as Phase 2/3), and the same weights are reused for every variant. Only the goal (and,
for TRANSPORT, the start) changes:

| Stage | Start | Goal | Notes |
|---|---|---|---|
| REACH (CDMP) | demo start (HOME) | grip TCP of V<i> = T(pickup_V<i>) · inv(grasp_i) | gripper open 40 mm |
| ALIGN_GRIP | CDMP end | exact grip TCP | 1.0 s straight min-jerk (CDMP residual 0.2–0.8 mm) |
| LIFT | grip | +30 mm | Stage 6, 3.0 s min-jerk |
| TRANSPORT (CDMP) | lift end | Stage 9 start of V<i> (part upright, 0.70 mm above the fitting top; where the V1 demo ends) | |
| ALIGN_INSERT | CDMP end | exact Stage 9 start | 1.0 s straight min-jerk (≈0.3 mm) |

Goal settling: Phase 2 bounds (2 mm / 0.5°). Every 20 ms sample is solved with the calibrated-UR3e IK seeded from
the previous one; RG2 links (and the held part) are FCL-checked against every scene object (every 5th sample of the
CDMP stages, every sample of the short stages). Arm links above the RG2 are not checked offline (MoveIt does that on
fake hardware). The DMP path shape does not depend on tau; joint speeds scale with 1/tau.

## Result (tau 2, prongs layout; `results/sim_reference/stage10_retarget_20261030_tau2/`)

| Variant | Result | Min clearance REACH / LIFT / TRANSPORT (mm) | Peak joint speed (rad/s) | Hand-over to Stage 9 |
|---|---|---|---|---|
| V1 | PASS | 4.2 / 0.6 / 1.0 | 0.45 | 1.5e-13 rad |
| V2 | PASS | 2.3 / 0.6 / 1.0 | 0.41 | 8e-14 rad |
| V3 | PASS | 4.9 / 0.6 / 0.9 | 0.44 | 8e-13 rad |
| V4 | **FAIL: IK** | TRANSPORT reaches an elbow singularity (elbow ≈ −0.04 rad) at TCP (−169, −350, 201) mm, t = 11.6 s | | |
| V5 | **FAIL: IK** | same, at TCP (−119, −355, 223) mm, t = 9.4 s | | |

V4/V5: the demo carries the part ~195 mm high; with the goal moved 150–200 mm further from the robot the arm
straightens fully on its current configuration branch. Those points are reachable in other arm configurations,
so this is a branch/singularity problem of the retargeted path, not of the cell. Not yet resolved.

Speed: the demonstration itself peaks at 0.86 rad/s; at tau 2 the CDMP stages peak at 0.41–0.45 rad/s. The 0.14 rad/s
physical-pilot guard would need tau ≈ 6.5 (REACH ≈ 104 s, TRANSPORT ≈ 81 s).

## V4/V5 fix (31 Oct 2026, overnight): arc-height scaling of the learned z forcing term

Three fixes were tested for the V4/V5 elbow singularity (`v45_fix_experiments.py`, `posture_variant.py`,
`--z-scale` in `retarget_reach_transport.py`). Joint limits are screened at the UR3e manual's elbow ±160°
(PolyScope 5.24; the UR datasheet says ±360°, so the stricter value is used until the lab controller is checked).

**A. Different arm posture, same Cartesian path: no usable posture.** Distinct IK branches at the lift end
were searched (search stopped early: 18 for V4, 7 for V5); most put the elbow or wrist below the fixture.
The candidates whose TRANSPORT walk succeeded all failed another check:
- V4 branch 1: elbow reaches −208° (beyond ±160°), and it needs a 4.0 rad re-configuration at HOME.
- V4 branch 6: passes within σ_min = 1e-4 of a singularity during TRANSPORT and branch-jumps in ALIGN_INSERT.
- V4 branch 14: the gripper/part collides.
- V5 branch 0: the arm comes within 38 mm of the fixture (proxy limit 60 mm).
- V5 branch 3: the elbow reaches −169° at the fitting (beyond ±160°).

**B. Height-ceiling coupling term (ext_force, spring + damper above z_cap): rejected.** The ceiling takes
energy out of the z dynamics, so the end of the motion undershoots the goal:
- V4: passes at 180 mm with only 0.18 mm clearance; at 160/140 mm the part is driven into its fitting.
- V5: IK fails at 180 mm; at 160/140 mm the part dips ~23 mm below the goal into the fitting.
- With the goal 5 mm higher, V4 passes at 160/180 mm, but V5 still collides.

**B′. Scaling the learned z forcing term (arc height): adopted.** Victor's CDMP weights for z
(`wp[2, :]`) are multiplied by k ≤ 1. x/y forcing, orientation, start, goal, timing and the attractor are
unchanged, so the demonstrated shape is kept and only the height of the arc is reduced. The end converges
normally, with no undershoot.

Sweep at tau 2, ALIGN_INSERT included, RG2 + part FCL vs the whole scene:

| k | V4 max TCP z | V4 result (σ_min) | V5 max TCP z | V5 result (σ_min) |
|---|---|---|---|---|
| 1.0 | ~201 | FAIL, elbow singularity | ~223 | FAIL, elbow singularity |
| 0.9 | 196 mm | PASS (0.035) | 213 mm | FAIL, IK |
| 0.8 | 181 mm | PASS (0.051) | 198 mm | FAIL, IK |
| 0.7 | 167 mm | PASS (0.063) | 183 mm | PASS (0.022) |
| 0.6 | 153 mm | PASS (0.072) | 168 mm | PASS (0.043) |

**Selection rule, the same for every variant and fixed before running V1–V5:** use the demo's amplitude
(k = 1). If the TRANSPORT IK fails, or σ_min of the 6×6 Jacobian (m, rad) along TRANSPORT drops below
0.04, try k = 0.9, 0.8, … 0.5 in turn.

Result (`results/sim_reference/stage10_retarget_20261031_zscale/`, tau 2, prongs layout):

| Variant | k | σ_min | Result | Peak joint speed |
|---|---|---|---|---|
| V1 | 1.0 | 0.058 | PASS | 0.45 rad/s |
| V2 | 1.0 | 0.071 | PASS | 0.41 rad/s |
| V3 | 1.0 | 0.046 | PASS | 0.44 rad/s |
| V4 | 0.8 | 0.051 | PASS | 0.31 rad/s |
| V5 | 0.6 | 0.043 | PASS | 0.34 rad/s |

- V1–V3 are identical to the 30 Oct reference.
- All five hand over to Stage 9 L2 within ≤1e-10 rad.

Wording for the report: the arc height is a spatial-scaling parameter of the learned forcing term, applied
by a fixed feasibility rule. It is not a new demonstration, and it is not a guarantee of singularity
avoidance: σ_min is logged, and arm-link collisions are checked by MoveIt on fake hardware.

## RELEASE → RETREAT → HOME (31 Oct 2026)
`plan_release_retreat.py` uses the same steps as the physical V1 sequence (`phase5_integration/run_v1_physical_auto.py`):
1. **RELEASE:** RG2 to 40 mm. The part stays seated at its final Stage 9 L2 pose and becomes a scene object.
2. **RETREAT:** +30 mm in base z, 3 s min-jerk.
3. **HOME:** joint min-jerk back to the taught HOME, peak ≤ 0.3 rad/s.

The RG2 (and the seated part) is checked against the whole scene. All five variants pass:

| Variant | Fingers clear the part at | Min to the fitting while opening | RETREAT min clearance | HOME min clearance |
|---|---|---|---|---|
| V1 | 14.2 mm | 3.08 mm | 6.9 mm | 22.3 mm |
| V2 | 18.1 mm | 1.23 mm | 6.7 mm | 21.4 mm |
| V3 | 12.2 mm | 1.28 mm | 5.0 mm | 21.5 mm |
| V4 | 12.2 mm | 4.56 mm | 8.7 mm | 24.5 mm |
| V5 | 14.2 mm | 4.21 mm | 7.8 mm | 23.1 mm |

The fingers clear the part only above ~12 mm opening because the modelled 8.2 mm grip overlaps the grip block (squeeze).

The RViz runner `--full` now plays the whole cycle for every variant:
HOME → REACH → ALIGN_GRIP → GRIP → LIFT → TRANSPORT → ALIGN_INSERT → INSERT → ROTATE → RELEASE (detach, part left seated) → RETREAT → HOME.
