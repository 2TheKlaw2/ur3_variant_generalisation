# Stage 6: V1 pickup + 30 mm lift on fake hardware

HOME → OPEN 40 mm → REACH → ALIGN → GRIP 8.2 mm → ATTACH V1 → LIFT 30 mm → STOP

Kinematic simulation on fake hardware (ROS_DOMAIN_ID=73). These are not physical results, and no physical robot commands are sent.

## Run
```bash
./scripts/phase6_pickup_sim/start_fake_robot_calibrated.sh       # terminal 1
./scripts/phase6_pickup_sim/start_fake_moveit_calibrated.sh      # terminal 2 (MoveIt + RViz)
export ROS_DOMAIN_ID=73                                          # terminal 3
python3 scripts/phase6_pickup_sim/run_v1_pickup_lift_sim.py              # preflight, no motion
python3 scripts/phase6_pickup_sim/run_v1_pickup_lift_sim.py --run        # one START prompt
python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset        # put V1 back in its pocket
```
In RViz (MotionPlanning display → Scene Geometry), V1 turns into an attached object at ATTACH and follows the gripper. V2–V5 stay in their pockets.

Logs go to `results/simulation/v1_pickup_lift/<timestamp>/` (gitignored). The reference run is copied to `results/sim_reference/`.

## Important: fake robot now uses the lab calibration
The upstream `ur_onrobot_control/start_robot.launch.py` and `ur_onrobot_moveit.launch.py` **ignore** `kinematics_parameters_file`. The MoveIt one passes `kinematics_params:=`, which the xacro does not use. As a result, the older `phase1_reach/start_fake_*.sh` scripts ran the **default** UR3e kinematics. That put the simulated TCP ~1.2 mm from where the real robot was for the same joints.

`launch/sim/fake_robot_calibrated.launch.py` and `launch/sim/moveit_calibrated.launch.py` are verbatim copies (MIT, Tony Le) with only the calibration wired through. The upstream packages are untouched. Earlier fake-hardware results (phases 1–5) used default kinematics.

## Method
- **Planning scene:** `planning_scene_tools.py` loads all 12 CAD objects from `cad/manifest.yaml` + `config/scene/scene_registration.yaml` (same poses as the RViz URDF scene; verified by readback).
- **Attach:** at GRIP, `T(gripper_tcp→V1) = inv(T(base→gripper_tcp)) · T(base→V1)` (MoveIt FK; TF agrees to 0.0 mm). This is held fixed. Touch links are the RG2 fingers/knuckles. `pickup_V1↔pickup_plate` is allowed only during LIFT (resting contact in the 2 mm pocket) and restored afterwards.
- **Lift geometry:** from `run_v1_lift_bridge_transport_staged.py`: +30 mm base Z, constant orientation, 15 waypoints. MoveIt Cartesian feasibility check: fraction 1.0. Wrist_3 2π correction.
- **Samples:** one IK solution per 20 ms timestamp of the 3.0 s minimum-jerk profile (151 samples), each seeded from the previous one (damped least squares on the lab-calibrated model, `scripts/scene/ur_kinematics.py`). The executed trajectory therefore carries the profile itself, with zero speed at both ends and no joint-space resampling. Every sample is checked with MoveIt FK (≤0.01 mm from the target) and MoveIt collision checking with V1 attached.
  - MoveIt's KDL `/compute_ik` was not used: 5 ms solver timeout, and branch jumps from random restarts.
  - This MoveIt build also resamples Cartesian results at 0.1 s, so the raw Cartesian output has only 5 points.
- **Timing:** NEW 3.0 s minimum-jerk on path length (V1 speed 0 → 18.75 mm/s peak → 0). The physical lift used ≥ 5 s MoveIt timing. These are reported separately and are not compared.

## Reference result (`results/sim_reference/v1_pickup_lift_20261029_222514/`)
| Check | Result |
|---|---|
| Attach jump | 0.00000 mm / 0.00000° |
| Grasp transform T(tcp→V1) | (1.431, 1.861, 34.848) mm. The 2.35 mm off-axis grip is preserved |
| V1 rise (FK of executed joints / TF at STOP) | 30.000 / 30.000 mm |
| V1 xy drift, tilt | 0.0002 mm, 0.000° |
| IK sample deviation from min-jerk target | < 0.001 µm |
| Peak joint step / speed | 0.0017 rad / 0.085 rad/s (physical guard 0.045 / 0.14) |
| LIFT endpoint joint error | 0.000000 rad |
| V2–V5, plates, fittings | unchanged |
| Collisions (all 151 timed samples, V1 attached) | none |
| Final state with V1↔plate checked again | valid |

The offline preview (`preview_v1_lift_offline.py`, no ROS) predicted the same grasp transform, rise and joint speeds. It also gives clearances: fingers ≥22.4 mm from V2 and ≥18.1 mm from the plate. V1 clears the pocket at t = 0.7 s.

Negative checks (collision checking is live): V1 pushed 2 mm into the plate → `pickup_plate↔pickup_V1`. V1 moved onto V2 → V2 vs V1 and both fingers.

## Not yet covered
- REACH/ALIGN replay the existing joint CSVs ×2.0 (evenly retimed). This is not CDMP timing; do not report REACH timing from this run.
- Transport, insertion, rotation, release: next stages.
