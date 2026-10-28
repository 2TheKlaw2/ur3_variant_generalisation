UR3e Variant Generalisation — Stage 3 V1 Recording Update
===========================================================

Adds a C++ named-pose saver, Enter-to-start/stop recording wrapper, and menu.
Does NOT alter original ur3_lfd package or existing home file.

1) First make backups of your current new-project build configuration:

   cd ~/ros2_ws/src/ur3_variant_generalisation
   cp CMakeLists.txt CMakeLists.txt.stage2.bak
   cp package.xml package.xml.stage2.bak
   cd ~/ros2_ws/src
   unzip -o ~/Downloads/UR3e_Variant_Stage3_V1_Recording_UPDATE.zip

2) cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-up-to ur3_variant_generalisation
   source install/setup.bash
   colcon test --packages-select ur3_variant_generalisation
   colcon test-result --verbose

3) With real driver, joint states and TF active AND approved freedrive enabled separately:
   cd ~/ros2_ws/src/ur3_variant_generalisation
   bash scripts/teach_v1.sh

Read docs/STAGE3_REAL_V1_RECORDING.md for the full lab order and warnings.
ROS2-dependent C++ node needs the Ubuntu ROS2 Humble build; offline C++ testing
of the existing adapter does not test the new live ROS2 node.
