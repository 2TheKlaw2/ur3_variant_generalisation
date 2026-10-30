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
