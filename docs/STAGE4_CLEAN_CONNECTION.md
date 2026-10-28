# Stage 4: Clean connection baseline (UR3e hardware only)

This update is **additive**. It only creates scripts under `scripts/connection/` and this document. It does **not** replace `ur3_lfd/src/robot_controller.cpp`, modify the UR driver, switch controllers, load the RG2 driver, run MoveIt, enable freedrive, or send motions.

## Why this matters

`ur3_lfd/src/robot_controller.cpp` is a ROS joint-state subscriber and FollowJointTrajectory action client. The original `ur3_lfd/scripts/11_launch_real_robot.sh` launches the combined `ur_onrobot_control start_robot.launch.py` with RG2 and RViz. To isolate External Control issues, test the **official UR driver alone**.

## Lab startup

1. Physically stop previously launched UR and combined-stacks under the lab's approved procedure. Do not run two drivers against the same robot.
2. Confirm which UR3e unit you have, its robot IP and pendant External Control host IP. Connect the dedicated Ethernet interface.
3. Terminal A:

   ```bash
   cd ~/ros2_ws/src/ur3_variant_generalisation
   bash scripts/connection/01_connect_ur3e_minimal.sh
   ```

   For a verified calibration belonging to this exact robot, you MAY instead use:

   ```bash
   UR3E_CALIBRATION_FILE="$HOME/ur3e_calibration.yaml" bash scripts/connection/01_connect_ur3e_minimal.sh
   ```

   Do not assume a calibration from a different UR3e is interchangeable. Without verified calibration, connection diagnostics are fine; precision motion is not.

4. Only after the driver is up, press Play on the robot pendant's External Control program.
5. Terminal B:

   ```bash
   cd ~/ros2_ws/src/ur3_variant_generalisation
   bash scripts/connection/02_connection_diagnostics.sh
   ```

6. Observe connection status and pendant program state with the robot **stationary** for a useful interval, then save the launch console output if connection drops. If stable here but unstable with the combined stack, add RG2 separately **later** to isolate its impact. Do not automatically start the combined stack in parallel.

## Save new home

With the arm stationary in a deliberately selected, physically safe home pose:

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash scripts/connection/03_save_new_research_home.sh
```

This uses the already built Stage 3 ROS2 C++ `save_named_pose` executable. It saves a new timestamped `v1_lab_home_YYYYmmdd_HHMMSS.yaml` under `config/saved_poses/` and **never overwrites** the prior `ur3_lfd/config/home.yaml` or `final_home.yaml`. A saved pose is NOT a validated motion plan or collision-safe home; verify separately before ever commanding the robot to it.

With the combined RG2 TF model later, the same script can select `gripper_tcp` explicitly:

```bash
UR3E_HOME_TOOL_FRAME=gripper_tcp bash scripts/connection/03_save_new_research_home.sh
```

## Diagnosing External Control

- `Connection to reverse interface dropped`: External Control program may have stopped, robot may have entered protective stop, other URScript commands/pendant actions may have interrupted it, or communication may have dropped. Check driver console and pendant; do not immediately blame the C++ action client.
- Missing `/joint_states` or `robot_program_running` information: inspect ROS nodes, driver output and network. These are read-only diagnostics.
- Controller name/action mismatch is an execution issue, not necessarily the UR reverse connection.
- `ur_robot_driver` startup creates its normal required driver/controller/broadcaster nodes even in this minimal test. 'Minimal' here means *no additional application stacks*.

## Safety boundaries

- Do not attempt robot motion from any synthetic `ANALYSIS_ONLY_*` CSV.
- Do not remove safety limits or force commands to diagnose connectivity.
- Do not start recording the new V1 task using `gripper_tcp` while only the bare UR driver is active, because that frame comes from your combined gripper model. The baseline connection test is intentionally separate from the assembly task.
- No parts of this update modify the robot's internal PolyScope configuration.

Official reference: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/startup.html
