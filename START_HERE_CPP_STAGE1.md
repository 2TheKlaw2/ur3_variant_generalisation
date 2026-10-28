# Stage 1 C++ setup — install OVER the existing scaffold safely

This update is meant for the directory you **already extracted** at:

`~/ros2_ws/src/ur3_variant_generalisation/`

It adds source/header files, CMakeLists.txt, package.xml, C++ tests and an offline test script. **It does not overwrite `config/variant_motion_parameters.csv` or your old demonstrations.** The old sibling `../ur3_lfd/` remains untouched.

### Install update ZIP

```bash
cd ~/ros2_ws/src
unzip -n ~/Downloads/UR3e_Variant_CPP_Stage1_UPDATE.zip
```

If you already have an older Stage 1 copy and need to update it, first back up any edits and compare files before replacing them. Avoid blindly overwriting your work.

### Run first tests without ROS, robot or colcon

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash RUN_CPP_STAGE1.sh
```

This uses **C++17** and CMake and compiles the **existing DMP implementation** from `../ur3_lfd/src/dmp.cpp` read-only. It writes fixture-relative CSVs for V1–V5 under `trajectories/synthetic/`. Two timing policies are generated.

### ROS2 build (optional next step)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ur3_variant_generalisation
source install/setup.bash
```

The ROS2 build path is prepared but has not been compiled in the file-creation environment (ROS2 is unavailable there). If build errors occur, save the full terminal output and make fixes in the new package, not `ur3_lfd`.

### Safety + scope

Outputs have deliberately incompatible CSV headers with your original robot executor. Do **not** rename headers or execute them. We still need the actual fixture-to-robot transform, the gripper-to-axis offset, original tool orientation, real V1 freedrive training, full robot collision checks and validation before producing executable paths.

Read `docs/IMPLEMENTATION_STATUS.md`.
