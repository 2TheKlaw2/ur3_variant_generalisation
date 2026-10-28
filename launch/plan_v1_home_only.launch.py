import os

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_onrobot_moveit_config.launch_common import load_yaml


def generate_launch_description():

    project = os.path.expanduser(
        "~/ros2_ws/src/ur3_variant_generalisation"
    )

    home = os.path.join(
        project, "config/saved_poses/v1_home_for_moveit.yaml"
    )

    calibration = os.path.join(
        project, "config/ur3e_lab_calibration.yaml"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("ur_onrobot_description"),
                    "urdf", "ur_onrobot.urdf.xacro"
                ]),
                " robot_ip:=192.168.0.192",
                " ur_type:=ur3e",
                " onrobot_type:=rg2",
                " name:=ur_onrobot",
                " kinematics_parameters_file:=", calibration
            ]),
            value_type=str
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("ur_onrobot_moveit_config"),
                    "srdf", "ur_onrobot.srdf.xacro"
                ]),
                " name:=ur_onrobot"
            ]),
            value_type=str
        )
    }

    kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_onrobot_moveit_config",
            "config/kinematics.yaml"
        )
    }

    planner = Node(
        package="ur3_lfd",
        executable="go_home",
        name="go_home",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            {
                "planning_group": "ur_onrobot_manipulator",
                "home_file": home,
                "execute_real_robot": False,
                "use_sim_time": False
            }
        ]
    )

    return LaunchDescription([planner])
