# Phase 2 — Six-DOF CDMP goal and temporal adaptation (V1 reach)

**Status:** Offline tooling prepared; actual cleaned-demo rollouts and ROS fake-hardware tests must be run on Ubuntu. **Nothing here commands the physical UR3e.**

## Goal of this experiment

Train the existing Victor/Tony `CartesianDMPs` **once** on `v1_reach_001_motion.csv`, then reuse the learned forcing weights to reproduce:

| Trial | Position/orientation goal | CDMP `tau_scaler` | Purpose |
|---|---|---:|---|
| `baseline_tau2` | Original cleaned-demonstration endpoint | 2.0 | Original learned reach reference |
| `adapted_tau2` | `v1_grip_pose.yaml` | 2.0 | Spatial **and orientation** goal adaptation |
| `adapted_tau1` | `v1_grip_pose.yaml` | 1.0 | Fast CDMP reproduction |
| `adapted_tau3` | `v1_grip_pose.yaml` | 3.0 | Slow CDMP reproduction |

The input file's starting and ending TCP poses are in `base -> gripper_tcp`. The saved grip YAML must use **the same frames**. This experiment performs `rep_relpose_goal` adaptation **inside the original 6-DOF Cartesian DMP**, rather than shifting already generated CSV points.

**Important nuance:** The commonly quoted 8.31 mm is between the **raw reach recording's last sample** and the saved final grip pose. The cleaned reach clip ends slightly earlier. The generator prints and saves the exact goal shift it calculates from the **cleaned** demonstration.

A single V1 goal shift is a *within-variant adaptation test*, **not yet evidence of unseen V2–V5 generalisation**.

## Inputs already on the Ubuntu laptop

- `~/ros2_ws/src/ur3_variant_generalisation/demonstrations/processed/v1_reach_001_motion.csv`
- `~/ros2_ws/src/ur3_variant_generalisation/config/saved_poses/v1_grip_pose.yaml`
- `~/ros2_ws/src/ur3_variant_generalisation/config/saved_poses/v1_home_for_moveit.yaml`
- `~/ros2_ws/src/rs2_dmp_demo/cdmp/src/cdmps.py`
- Existing installed `ur_onrobot_description`, `ur_onrobot_moveit_config`, `ur3_lfd` packages
- Phase 1 scripts in `scripts/phase1_reach/`, including its *sim-only* guarded executor, plan generator, trajectory collector and C++ validator

This package only **adds** files under `scripts/phase2_reach/` and this readme under `docs/`. It neither touches raw data nor edits the original `ur3_lfd` code.

## A — Generate and evaluate the DMP goal adaptation (no ROS robot required)

In Ubuntu with your existing dependencies installed:

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
python3 scripts/phase2_reach/generate_phase2_cdmp.py
python3 scripts/phase2_reach/compare_phase2_cdmp.py
```

The generator trains **once**, makes four separate goal/time rollouts, requires each to converge within 2 mm/0.5° (offline numerical tolerances), and writes to:

```text
demonstrations/processed/phase2_reach/
  v1_reach_baseline_tau2_moveit.csv
  v1_reach_adapted_tau2_moveit.csv
  v1_reach_adapted_tau1_moveit.csv
  v1_reach_adapted_tau3_moveit.csv
  phase2_manifest.json
  phase2_comparison.json
  phase2_path_comparison.png
  phase2_temporal_comparison.png
```

Existing outputs are **not overwritten**. Keep the hashes recorded in the manifest for experimental reproducibility.

Inspect `phase2_comparison.json`: record the Cartesian goal displacement, orientation displacement, endpoint errors, trial durations, and difference between trajectory shapes. Reject outliers, non-unit quaternions and implausible paths before using MoveIt.

## B — Prepare PLAN-ONLY MoveIt runs in fake hardware

**First finish Phase 1 setup.** Start fake UR3e and MoveIt in isolated `ROS_DOMAIN_ID=73`, verify `use_fake_hardware:=true`, and ensure the robot is at saved V1 home. Ideally disconnect the Ethernet from the real robot. Never launch the real robot driver in the simulation domain.

In another terminal in domain 73:

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation

# Phase 1's known fake-hardware and collision-planning configuration,
# /tmp/v1_phase1_FULL_REACH_PLAN_ONLY.yaml, must exist first.
python3 scripts/phase2_reach/prepare_phase2_sim_plans.py
```

This creates four fake-hardware parameter YAMLs under `/tmp/v1_phase2_sim_plans/` with `moveit.execute: false`, using the same URDF, SRDF, kinematics and planning settings as Phase 1.

### First run: adapted_tau2

**Terminal A: capture a candidate joint trajectory BEFORE starting the planner:**

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation

python3 scripts/phase1_reach/capture_full_plan.py \
  --output demonstrations/processed/phase2_reach/v1_reach_adapted_tau2_joint_candidate.csv
```

**Terminal B: run PLAN-ONLY Cartesian planner:**

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation

ros2 run ur3_lfd execute_cartesian_path_moveit --ros-args \
  --params-file /tmp/v1_phase2_sim_plans/adapted_tau2_PLAN_ONLY.yaml
```

Require `Cartesian DMP path fraction: 100.0%`, `DMP Cartesian planning: PASS`, `PLAN-ONLY PASS`. The capture script adjusts each entire continuous joint to a consistent home branch (Wrist 3's earlier `-2π` issue). The candidate is **not automatically safe**: check actual joint positions, all modelled obstacles, kinematic limits and full path in RViz. The existing pickup/placement fixtures may still be missing as collision objects.

**Compile/use the Phase 1 independent C++ numeric checker:**

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  scripts/phase1_reach/validate_joint_trajectory.cpp -o /tmp/v1_phase1_check

/tmp/v1_phase1_check \
  demonstrations/processed/phase2_reach/v1_reach_adapted_tau2_joint_candidate.csv 2
```

The C++ checker has conservative *project review* thresholds; passing is not a collision/physical safety certification.

## C — Simulate the adapted reach (FAKE only)

Make sure the **fake** robot is at V1 saved home. If the fake arm was left at the Phase 1 reach endpoint, reset it using Phase 1's guarded `--home` mode; **that mock home action is not a collision-checked physical movement**.

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation

python3 scripts/phase2_reach/sim_phase2_replay.py \
  --replay demonstrations/processed/phase2_reach/v1_reach_adapted_tau2_joint_candidate.csv \
  --slowdown 2 \
  --log results/phase2_reach/adapted_tau2_sim_actual_001.csv
```

Requires typed `CONFIRM_FAKE_ADAPTED_REACH`, verified mock hardware and isolated domain, valid joint trajectory, and robot physically *represented by fake hardware only*. Never point it at a physical scaled trajectory controller. RG2 remains open; there is no pickup command.

Record action result, simulated end joints and measured trace; **joint endpoint matching does not establish Cartesian gripping accuracy** without FK/TCP verification.

## D — Temporal-adaptation trials

The offline CDMP generator produces `tau=1,2,3` using the **same CDMP learned weights** with different time scales. These already support an offline temporal scaling comparison.

Crucial: `execute_cartesian_path_moveit.cpp` **retimes** the Cartesian paths using MoveIt time parameterization. A newly planned joint trajectory will NOT automatically preserve the CDMP's `tau` values. Don't claim a `tau=3` simulated runtime unless verified using the executed joint timing. You can separately vary the fake-controller `--slowdown` (e.g. 1 versus 2 versus 3) on a **single validated adapted_tau2 joint trajectory** to test playback timing, while labeling that as controller time scaling rather than CDMP-native temporal scaling.

Only after successful `adapted_tau2` simulation should you plan, capture, validate and compare `adapted_tau1` and `adapted_tau3` trajectories. Use separate output paths and move the **fake** robot back to home before each playback.

## Acceptance criteria

- [ ] Source demo hashes and saved goal hashes captured; no raw file overwritten
- [ ] Adapted DMP goal matches `v1_grip_pose.yaml` position and orientation within offline tolerances
- [ ] Baseline and adapted trajectories differ in a way consistent with new goal, not a simple copied CSV
- [ ] All three tau rollouts produced from **one** trained CDMP
- [ ] Adapted Cartesian MoveIt path plans at **100%** in fake hardware
- [ ] Correct joint wrapping, continuity, speed and acceleration validated
- [ ] Goal and full trajectory clear of modelled obstacles; missing trolley/plates acknowledged
- [ ] Fake UR3e completes an adapted reach and simulated states are logged
- [ ] Offline and simulation results summarized with correct limitations

**Later, in lab:** geometry-calibrated pickup clearance, safety-approved physical adapted reach, physical RGB/RG2 gripping, and unseen-variant trials are separate stages.
