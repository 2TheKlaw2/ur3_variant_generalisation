# Stage 3: V1 demonstration capture (UR3e + RG2, ROS2 Humble)

## Scope and protections

These new scripts use the **existing** `ur3_lfd` C++ `demo_recorder` but save new files only under `ur3_variant_generalisation`. `ur3_lfd` source, `config/home.yaml`, and earlier demonstrations are never modified. This package does **not** switch controllers, command the robot, operate the RG2, or execute any trajectories.

**Before lab:** inspect gripper-to-component axis offset, registration, plate clearances and safety controls. The separate freedrive procedure is approved and activated by lab staff/operators. Never use a Cartesian analysis CSV as a robot command. Recordings capture **gripper_tcp**, not a directly observed tab path.

## Build (after extracting update)

In a fresh terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ur3_variant_generalisation
source install/setup.bash
colcon test --packages-select ur3_variant_generalisation
colcon test-result --verbose
```

## Session operation

Start robot driver and gripper/TF stack using your previously validated lab commands. Confirm `/joint_states` publishes all six UR joint names and that `base -> gripper_tcp` TF is available. Establish and verify approved robot freedrive. Do not operate the gripper via the keyboard on the same moment you're manually guiding the arm. A second person at the laptop is beneficial.

Launch the interactive menu **after** the ROS driver/TF stack is up:

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash scripts/teach_v1.sh
```

Choose actions one at a time. For each recording, ENTER starts; guide the robot; ENTER stops and saves. For pose capture, **hold the robot stationary for approximately 1–2 seconds** while the saver checks joint stability and recent TF; any active movement triggers refusal. Do not use a pose file as an automatic safe move target.

Recommended order:

1. Existing home / safe initial pose. **Menu 1:** record `v1_reach_001.csv` (home -> gripping position). Keep part ungripped while guiding to it.
2. **Menu 2:** save `config/saved_poses/v1_grip_pose.yaml` when stationary at the desired grasp.
3. Grip V1 using existing approved RG2 controls, *outside* these scripts. Confirm secure grip and clearance.
4. **Menu 3:** record lift/transport to a position directly above placement V1 (`v1_transport_001.csv`).
5. **Menu 4:** save stationary `v1_insertion_ready.yaml`. This is a checkpoint, not the robot's global home.
6. **Menu 5 (priority):** record **one uninterrupted V1 assembly demonstration**, from insertion-ready through 5 mm insertion then 10° clockwise rotation (`v1_assembly_full_001.csv`). A brief pause at the transition is helpful.
7. **Menu 9:** save the final assembled pose if stationary. Stop and inspect data; do not replay the robot yet.

**Optional independent-phase captures** (after safe reset/repositioning):

- **Menu 6:** `v1_insertion_001.csv` (5 mm straight insertion); then **menu 7** save `v1_insertion_end.yaml`.
- **Menu 8:** `v1_rotation_001.csv` (10° CW rotation only, starts at the correct insertion-end pose); then **menu 9** to save final pose if not already saved.

For a primary research claim, use **one complete V1 assembly recording** and split it into phases *offline*. The optional individual recordings are diagnostics and should not be secretly substituted into the single-demonstration experiment.

File names auto-increment for recordings (`_001`, `_002`, etc.) and raw recordings are never overwritten. Named-pose files cannot be overwritten; use a distinct name if you need a second take, e.g. `v1_grip_pose_002` with the direct command below.

## Direct commands (without menu)

```bash
bash scripts/record_v1.sh v1_reach
bash scripts/save_named_pose.sh v1_grip_pose
bash scripts/record_v1.sh v1_transport
bash scripts/save_named_pose.sh v1_insertion_ready
bash scripts/record_v1.sh v1_assembly_full
bash scripts/save_named_pose.sh v1_assembled_pose
```

Other labels: `v1_insertion`, `v1_rotation`, `v1_insertion_end`.

## Files written

- `demonstrations/raw/v1_*.csv` — untouched, timestamped samples, 15-column `DemoIO` format.
- `demonstrations/logs/v1_*.log` — recorder output for debugging.
- `config/saved_poses/v1_*.yaml` — named joint/TCP snapshots, **not** executable commands.

A YAML pose stores 6 joint positions (rad) and `base -> gripper_tcp` position (m) and quaternion (x,y,z,w). It **does not** overwrite the canonical home pose or calibrate the fixture. `tool0` is not an acceptable substitution without updating the consistent tool transforms.

## Post-record checks

```bash
ls -lh demonstrations/raw/v1_*.csv config/saved_poses/v1_*.yaml
head -n 2 demonstrations/raw/v1_assembly_full_001.csv
wc -l demonstrations/raw/v1_assembly_full_001.csv
```

Later work: offline phase detection/splitting, continuous signed-angle extraction from quaternions relative to the *calibrated assembly axis*, real DMP training, pin offset correction, simulation and collision checking. No robot playback or MoveIt execution is added by this update.

## If a recording fails

The start/stop service response is checked; if interrupted the shell attempts a best-effort stop-and-save. A recorder can create a valid CSV only when it receives joint states and a valid TCP TF. Inspect `demonstrations/logs/<name>.log` on failure. **Do not erase a failed raw recording**; select a new take name and document it.

## If pose saving fails

The C++ saver requires 10 valid observations over ≥0.7 seconds with joint span ≤0.005 rad, observed joint speeds ≤0.03 rad/s (when velocity exists), a fresh TF (≤2 seconds old) and a valid quaternion. It times out after 12 seconds. Check joint messages, frame names, freedrive drift and `/tf`; do not bypass safety gates to save an unstable pose.
