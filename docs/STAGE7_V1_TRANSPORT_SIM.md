# Stage 7: V1 transport to the insertion-ready pose (fake hardware)

HOME → OPEN → REACH → ALIGN_PICKUP → GRIP → ATTACH → LIFT 30 mm → **BRIDGE → TRANSPORT → ALIGN_INSERT → STOP**

Kinematic simulation on fake hardware (ROS_DOMAIN_ID=73), stopping **before insertion**. The physical pilot needed an E-stop at insertion, so this stage measures how V1 arrives at `fitting_V1` before anything is inserted. These are not physical results.

## Run
```bash
./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh       # terminal 1
./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh      # terminal 2
export ROS_DOMAIN_ID=73
python3 scripts/phase7_transport_sim/run_v1_transport_sim.py --run      # type START
python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset       # afterwards
```
Optional: `pip install python-fcl` adds the exact V1↔fitting minimum distance to the report.

## Method
| Stage | Method | Timing |
|---|---|---|
| Pickup + LIFT | Stage 6, unchanged | 3.0 s min-jerk (SIM) |
| BRIDGE | Re-planned with V1 attached and all CAD objects loaded, using the physical raised-bridge geometry (`run_v1_lift_bridge_transport_staged.py`). From the lift end it moves horizontally at lift height to above the join row, then straight to that row's pose. Join row is the first learned row ≥ 31 mm above the grip TCP, which is **row 24, the same as physically**. MoveIt Cartesian feasibility check, then IK every 20 ms. | 6.0 s, min-jerk per segment (SIM; physical used MoveIt timing ≥ 6 s) |
| TRANSPORT | Learned joint trajectory rows 24–245 | original 0.1 s spacing ×1.5 (as tested) |
| ALIGN_INSERT | Existing `v1_align_to_insertion_joint_plan_001.csv` | ×2.0 (as `run_v1_integrated_fake.py`) |

Every sample is collision-checked, and so are the joint-linear in-between states the controller executes (TCP step ≤ 1 mm). V1 is attached throughout. `V1↔fitting_V1` is **checked**, not allowed.

## Reference result (`results/sim_reference/v1_transport_20261029_225905/`)
**Motion: PASS (7/7).**
- **BRIDGE:** 302 samples; join gap to row 24 is 4e-15 rad; never lower than 30.0 mm above the pickup; peak 0.079 rad/s.
- **TRANSPORT:** 222 samples + 457 in-between states; 33.1 s; peak within the 0.14 rad/s physical guard.
- **ALIGN_INSERT:** 10 samples.
- No collisions on any stage. V2–V5 unchanged; V1 still attached.

**Alignment of V1 at fitting_V1 (measurement):**
| Quantity | Value | Allowance |
|---|---|---|
| Lateral offset of V1 axis | **2.598 mm** | radial clearance 0.25 mm |
| Tilt of V1 relative to the fitting | **3.81°** | none |
| Tab/slot yaw error | 0.35° | about ±1° |
| V1 bottom above fitting top | 0.70 mm | none |
| Min distance V1↔fitting (FCL) | 0.53 mm | none |
| Straight-down insertion probe (no motion) | **contact after 0.75 mm** (`fitting_V1↔pickup_V1`) | none |
| Within insertion clearance | **NO** | none |

**Where the 2.6 mm comes from:**
- **TCP vs fitting axis: 0.29 mm.** The learned transport and alignment bring the gripper to the right place.
- **Grasp offset: 1.61 mm.** V1's axis is off the TCP at pickup, as measured from the pickup registration.
- **Tilt: about 2.3 mm.** The TCP orientation differs by 3.8° between `v1_grip_pose` (tilted 2.6° toward −9°) and the demonstration's `v1_insertion_ready` (tilted 1.8° toward −126°). V1's bottom is 34.8 mm below the TCP, so 3.8° moves it about 2.3 mm.

So, in this simulation the arm path is accurate and the misalignment is a **grasp** effect. This matches the physical pilot finding (traced to part placement/grasp, not the arm path). A rigid grasp that is consistent with the demonstration would have to differ from the pickup grasp by about 3.8°. The demo's grasp and the pilot's grasp are probably not the same. With only ~5 mm of finger engagement on V1's grip block, tilting in the fingers is plausible.

## Caveats
- The fitting position is registered from the `v1_insertion_end` TCP, assuming the TCP was on V1's axis. That assumption may not hold if the demo grasp was also off-centre. PROVISIONAL.
- The grasp is rigid in simulation. Real RG2 grasps can slip or tilt.
- REACH/ALIGN replay existing joint CSVs; their timing is not CDMP timing.
