# Phase 1 — Complete learned V1 reach: isolated fake-hardware simulation

Prepared from the successful 27 October 2026 physical reach preview. This is **additive**; does not replace source code or raw demonstrations.

## Our actual Phase 1 goal

1. Preserve the human-taught V1 reach and trained six-DOF CDMP.
2. Verify a **full** CDMP Cartesian plan using `ur3_lfd/execute_cartesian_path_moveit` in plan-only mode.
3. Capture the time-parameterized **joint** plan and correct equivalent `2π` representations against saved taught home.
4. Independently validate starting pose, waypoints, timing, speeds and accelerations in **C++17**.
5. On a fake UR3e ROS2 system **with no connection to the real driver**, execute the captured 6-joint trajectory and measure final error.
6. Later: separately evaluate moving the CDMP goal to the final grip pose 8.31 mm away. Not included in the baseline reproduction.

## Prerequisites

- Ubuntu 22.04 / ROS 2 Humble.
- Existing `ur_onrobot_control`, `ur_onrobot_moveit_config`, `ur3_lfd`, `ur3_variant_generalisation` packages, with installed environment sourced.
- This **existing** project's files: `demonstrations/processed/v1_reach_cdmp_moveit_001.csv` (1595 points), `v1_reach_001_motion.csv` (798 points), and `config/saved_poses/v1_home_for_moveit.yaml`.
- Verified `config/ur3e_lab_calibration.yaml`.
- `python3-yaml`, `xacro`, and `g++`.

## Safety: don't accidentally connect the real UR3e

- Fake ROS2 processes use **ROS_DOMAIN_ID=73**, which must be exported in **every** separate terminal.
- `start_fake_robot.sh` uses `use_fake_hardware:=true`, `robot_ip:=127.0.0.1`, and the ordinary **`joint_trajectory_controller`**, not the physical `scaled_joint_trajectory_controller`.
- `sim_reach_action.py` **refuses** action unless domain = 73, `/robot_state_publisher`'s expanded URDF contains a `mock_components/GenericSystem` hardware plugin, the fake controller is active and scaled controller is inactive.
- **Do not** run any real driver/UR ROS2 control node in ROS_DOMAIN_ID=73. Ideally work offline, with Ethernet disconnected from the real robot. These checks are defense in depth, not a substitute for system isolation.
- The original C++ MoveIt planner MUST retain `moveit.execute: false` for all of these steps.
- Setting the fake arm at HOME is a **mock-only configuration transition**. It does not validate collision safety for a real return-home motion.
- MoveIt's collision fraction does **not** cover the pickup plate, trolley, fingers or part unless they are genuinely present as collision objects in the planning scene. No planned or simulated pass proves real-world clearance.

## Install

Extract the provided ZIP into `~/ros2_ws/src/`, preserving the existing package. Only these new paths are added:

```text
ur3_variant_generalisation/scripts/phase1_reach/{scripts and .cpp}
ur3_variant_generalisation/docs/PHASE1_FULL_REACH_SIM_README.md
```

No source build is needed for new Python scripts or the old C++ MoveIt planner. The standalone C++ checker is compiled with `g++`, without ROS dependencies.

## Part A — C++ offline validation (can run before opening ROS)

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
S=scripts/phase1_reach

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  "$S/validate_joint_trajectory.cpp" -o /tmp/v1_phase1_check

# Our existing successful physical preview CSV should PASS:
/tmp/v1_phase1_check demonstrations/processed/v1_reach_preview_joint_trajectory_001.csv 2
```

This verifies numerical plausibility, **not** controller limits or clearance.

## Part B — Start the isolated fake robot and MoveIt

Four or more Ubuntu terminals, **not connected to the physical UR3e driver**.

**Terminal 1 — fake UR3e + RG2:**

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash scripts/phase1_reach/start_fake_robot.sh
```

**Terminal 2 — MoveIt + RViz for fake robot:**

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash scripts/phase1_reach/start_fake_moveit.sh
```

**Terminal 3 — move fake hardware to our actual taught home** (this is NOT MoveIt collision planning; no physical movement):

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation
python3 scripts/phase1_reach/sim_reach_action.py --home
```

Expected fake controller: `joint_trajectory_controller active`; `scaled_joint_trajectory_controller inactive`. If the script fails the fake-hardware checks, **stop and diagnose; never bypass the guard**.

## Part C — Full learned reach PLAN-ONLY and capture

**Terminal 3** (with ROS 2 environment and `ROS_DOMAIN_ID=73`):

```bash
python3 scripts/phase1_reach/prepare_full_reach_plan.py
```

This creates `/tmp/v1_phase1_FULL_REACH_PLAN_ONLY.yaml` with `moveit.execute: false` using the verified kinematics and fake-hardware URDF.

**Terminal 4 — start the read-only trajectory capture BEFORE planning:**

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation
python3 scripts/phase1_reach/capture_full_plan.py
```

**Terminal 3 — generate full Cartesian plan** (NO movement):

```bash
ros2 run ur3_lfd execute_cartesian_path_moveit --ros-args \
  --params-file /tmp/v1_phase1_FULL_REACH_PLAN_ONLY.yaml
```

Require `Cartesian DMP path fraction: 100.0%`, `DMP Cartesian planning: PASS`, `PLAN-ONLY PASS`, no discontinuities and correct path in RViz. The collector will ignore the one-point pre-position plan, capture the longer full Cartesian plan, and save:

`demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv`

The collector chooses each joint's `2π` branch to align with the saved V1 home. For Wrist 3 it will likely subtract `2π` from all waypoints. **This file is a simulation candidate, not independently approved for a physical robot.**

### C++ inspection of the captured full plan

```bash
/tmp/v1_phase1_check demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv 2
```

Record its duration, maximum waypoint step, peak speeds and accelerations. Inspect path and its end position in RViz. If planning or validation fails, stop, capture log, fix the plan before continuing.

## Part D — Simulate the actual full reach

With the fake arm at saved V1 home, and its entire planned route reviewed in RViz:

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ur3_variant_generalisation
python3 scripts/phase1_reach/sim_reach_action.py \
  --replay demonstrations/processed/v1_reach_full_joint_sim_candidate_001.csv \
  --slowdown 2 \
  --log results/v1_full_sim_actual_001.csv
```

The script demands proof of fake-hardware ROS URDF and explicit typed confirmation before sending a goal to **`/joint_trajectory_controller/follow_joint_trajectory`**, then logs simulated joint states. It never addresses the physical `scaled_joint_trajectory_controller`.

Record action result and endpoint joint error; compare the joint states to the planned path. The fake simulator mirrors position commands but is NOT a physics-level validation of contacts, force, cable twist or collision.

## Definition of done for Phase 1

- [ ] Full V1 Cartesian CDMP path plans at 100% in the isolated fake system.
- [ ] All planned joint states align to taught home without a `2π` jump; independent C++ checker passes.
- [ ] All physical obstacles needed for collision checking are represented or unmodelled gaps explicitly acknowledged.
- [ ] Fake UR3e at saved home, full reach goal accepted by fake controller and reaches intended endpoint.
- [ ] Simulated actual-versus-commanded motion recorded and error analysed.
- [ ] Original `ur3_lfd` source and all raw V1 CSVs remain unchanged.

**After Phase 1:** Phase 2 is adapting the CDMP to the authoritative saved `v1_grip_pose.yaml`, changing its goal (position and orientation) without another demonstration, then comparing baseline against adapted reach in simulation.
