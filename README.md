# UR3e Guided Pin-and-Slot Product-Variant Generalisation (2026)

**Scope:** This directory is for the NEW research experiment only. The existing `ur3_lfd` package and previous `lab_work` demonstrations remain unchanged.

**Objective:** Learn one real Variant 1 insert-and-rotate skill with kinesthetic teaching, adapt the learned CDMP to variants V2–V5 using CAD-derived geometry and measure reproduction accuracy/robustness. Implement in **C++17**, ROS2 Humble.

**Important:** This is a folder scaffold, NOT a buildable ROS2 package yet. We will add `package.xml`, `CMakeLists.txt`, headers, C++ implementation and tests after confirming how it links to `ur3_lfd_core`. No robot is commanded by this scaffold.

**Existing reusable components:** `../ur3_lfd/` provides robot/gripper control, recording, demo CSV I/O, basic DMP, MoveIt Cartesian executor and prior successful lab recordings. Avoid changing existing source while developing the new adapter.

**Existing calibration tooling:** `~/ur3e_calibration_fix_ws/` can remain a separate colcon workspace. Do not blindly source both workspaces into the same terminal. Save the generated robot-specific correction YAML separately; verify its path on the real laptop.

Read `docs/PROJECT_BOUNDARY.md`, `docs/LAB_INPUTS.md`, `config/variant_motion_parameters.csv` before adding code.
