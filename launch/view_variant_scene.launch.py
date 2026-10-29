"""Static V1-V5 scene in RViz: UR3e + RG2 (lab calibration) plus plates, parts and fittings.

    ros2 launch ur3_variant_generalisation view_variant_scene.launch.py
    ros2 launch ur3_variant_generalisation view_variant_scene.launch.py pose:=v1_pickup_V1_geometry
    ros2 launch ur3_variant_generalisation view_variant_scene.launch.py gui:=true

The scene is a SEPARATE description (namespace /scene) whose root is the robot's `base`
frame, so the UR3e/RG2 description is not modified. No controllers, no motion.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "ur3_variant_generalisation"


def _setup(context):
    share = get_package_share_directory(PKG)
    pose_name = LaunchConfiguration("pose").perform(context)
    width = float(LaunchConfiguration("gripper_width").perform(context))

    with open(os.path.join(share, "config", "saved_poses", pose_name + ".yaml")) as f:
        arm = yaml.safe_load(f)["arm"]
    zeros = dict(zip(arm["joint_names"], arm["positions_rad"]))
    zeros["finger_width"] = width

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]),
        " robot_ip:=192.168.0.192 ur_type:=ur3e onrobot_type:=rg2 name:=ur_onrobot",
        " kinematics_parameters_file:=", os.path.join(share, "config", "ur3e_lab_calibration.yaml"),
    ]), value_type=str)

    scene_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(share, "urdf", "variant_scene.urdf.xacro"),
        " show_trolley:=", LaunchConfiguration("show_trolley"),
        " trolley_xyz:='", LaunchConfiguration("trolley_xyz"), "'",
        " trolley_rpy:='", LaunchConfiguration("trolley_rpy"), "'",
    ]), value_type=str)

    gui = LaunchConfiguration("gui")
    tf_remap = [("tf", "/tf"), ("tf_static", "/tf_static")]
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
             parameters=[{"zeros": zeros}], condition=UnlessCondition(gui)),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", name="joint_state_publisher_gui",
             parameters=[{"zeros": zeros}], condition=IfCondition(gui)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             namespace="scene", parameters=[{"robot_description": scene_description}], remappings=tf_remap),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "variant_scene.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pose", default_value="v1_home_for_moveit",
                              description="Saved pose in config/saved_poses (joints only are used)"),
        DeclareLaunchArgument("gripper_width", default_value="0.040", description="RG2 finger_width [m]"),
        DeclareLaunchArgument("gui", default_value="false", description="Use joint_state_publisher_gui sliders"),
        # Trolley pose in `base`, composed from ur3_lfd/urdf/trolley.urdf.xacro:
        #   base->base_link Rz(pi) * base_link->trolley_link (0,-0.2,-1.047) * visual (0,0.4,0) Rz(pi)
        #   = (0,-0.2,-1.047), rpy 0.  Trolley top ~ base z 0 (robot mount height 1.047 m).
        DeclareLaunchArgument("show_trolley", default_value="false",
                              description="Show the ur3_lfd trolley mesh (read-only reference)"),
        DeclareLaunchArgument("trolley_xyz", default_value="0 -0.2 -1.047"),
        DeclareLaunchArgument("trolley_rpy", default_value="0 0 0"),
        OpaqueFunction(function=_setup),
    ])
