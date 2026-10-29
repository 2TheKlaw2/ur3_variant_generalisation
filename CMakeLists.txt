cmake_minimum_required(VERSION 3.8)
project(ur3_variant_generalisation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# On ROS2 Humble: use ur3_lfd's exported library without editing it.
# On a workstation without ROS2: explicitly choose STANDALONE and compile
# the existing DMP source read-only from ../ur3_lfd. This is for OFFLINE tests.
option(UR3_VARIANT_STANDALONE "Build offline without ROS2 (still requires ../ur3_lfd)" OFF)

if(UR3_VARIANT_STANDALONE)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ur3_lfd/src/dmp.cpp")
    message(FATAL_ERROR "Offline mode requires the existing sibling ../ur3_lfd/ source tree")
  endif()
  add_library(ur3_existing_dmp STATIC ../ur3_lfd/src/dmp.cpp)
  target_include_directories(ur3_existing_dmp PUBLIC ../ur3_lfd/include)
else()
  find_package(ament_cmake REQUIRED)
  find_package(ur3_lfd REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
endif()

add_library(variant_adapter_core STATIC
  src/variant_geometry.cpp
  src/variant_trajectory_generator.cpp
  src/plate_layout.cpp)
target_include_directories(variant_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(UR3_VARIANT_STANDALONE)
  target_link_libraries(variant_adapter_core PUBLIC ur3_existing_dmp)
else()
  # ur3_lfd exports the library name, but in some isolated colcon installs
  # ament_target_dependencies does not supply its library search directory.
  # Resolve the actual installed archive/shared library and link that path.
  get_filename_component(UR3_LFD_PREFIX "${ur3_lfd_DIR}/../../.." ABSOLUTE)
  find_library(UR3_LFD_CORE_LIBRARY
    NAMES ur3_lfd_core
    PATHS "${UR3_LFD_PREFIX}/lib"
    NO_DEFAULT_PATH
    REQUIRED)
  target_include_directories(variant_adapter_core PUBLIC "${UR3_LFD_PREFIX}/include")
  target_link_libraries(variant_adapter_core PUBLIC "${UR3_LFD_CORE_LIBRARY}")
endif()

add_executable(variant_offline_generator src/main.cpp)
target_link_libraries(variant_offline_generator PRIVATE variant_adapter_core)

add_executable(plate_layout_preview src/plate_layout_preview.cpp)
target_link_libraries(plate_layout_preview PRIVATE variant_adapter_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_variant_adapter tests/test_variant_adapter.cpp)
  target_link_libraries(test_variant_adapter PRIVATE variant_adapter_core)
  add_test(NAME variant_adapter_unit_tests COMMAND test_variant_adapter)
  add_executable(test_plate_layout tests/test_plate_layout.cpp)
  target_link_libraries(test_plate_layout PRIVATE variant_adapter_core)
  add_test(NAME plate_layout_unit_tests COMMAND test_plate_layout
    "${CMAKE_CURRENT_SOURCE_DIR}/config/pickup_plate.csv"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/placement_plate.csv"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/variant_motion_parameters.csv")
endif()

if(NOT UR3_VARIANT_STANDALONE)
  # Independent, read-only ROS2 checkpoint capture. No robot-controller dependencies.
  add_executable(save_named_pose src/save_named_pose.cpp)
  ament_target_dependencies(save_named_pose
    rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

  install(TARGETS save_named_pose
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/record_v1.sh scripts/save_named_pose.sh scripts/teach_v1.sh
    DESTINATION share/${PROJECT_NAME}/scripts)
  install(TARGETS variant_adapter_core variant_offline_generator plate_layout_preview
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})
  # Static V1-V5 RViz scene (CAD meshes, scene xacro, launch, RViz config)
  install(DIRECTORY cad urdf launch rviz DESTINATION share/${PROJECT_NAME})
  # V1 grip alignment (SIM, PLAN ONLY; no motion execution)
  find_package(moveit_ros_planning_interface REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(yaml-cpp REQUIRED)
  add_executable(align_to_grip_plan_only src/align_to_grip_plan_only.cpp)
  ament_target_dependencies(align_to_grip_plan_only
    rclcpp geometry_msgs moveit_ros_planning_interface moveit_core moveit_msgs)
  target_link_libraries(align_to_grip_plan_only yaml-cpp)
  install(TARGETS align_to_grip_plan_only
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  # V1 insertion alignment (SIM, PLAN ONLY; no motion execution)
  add_executable(align_to_insertion_plan_only src/align_to_insertion_plan_only.cpp)
  ament_target_dependencies(align_to_insertion_plan_only
    rclcpp geometry_msgs moveit_ros_planning_interface moveit_core moveit_msgs)
  target_link_libraries(align_to_insertion_plan_only yaml-cpp)
  install(TARGETS align_to_insertion_plan_only
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  ament_package()
endif()
