import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch.actions import OpaqueFunction


def load_yaml(package_name, file_path):
    """Load a yaml file from the specified package"""
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


# remember to start the moveit_controllers.launch.py file from the igus_rebel_moveit_config package first
# it will start the necessary controllers and the moveit dependencies
# start it with the correct gripper and hardware protocol arguments, and leaving rviz_file:=none (default)


# launches only the URDF version 2 robot description
def generate_launch_description():
    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "camera"],
        description="Gripper mount to attach to the last joint",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation"],
        description="Which hardware protocol or mock hardware should be used",
    )

    return LaunchDescription(
        [gripper_arg, hardware_protocol_arg, OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    if LaunchConfiguration("gripper").perform(context) == "camera":
        srdf_file = "igus_rebel_camera.srdf"
    else:
        srdf_file = "igus_rebel_base.srdf"

    robot_description_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_description_ros2"),
            "urdf",
            "igus_rebel.urdf.xacro",  # baseline version of the robot
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
            " gripper:=",
            LaunchConfiguration("gripper"),
        ]
    )

    robot_description_semantic = PathJoinSubstitution(
        [get_package_share_directory("igus_rebel_moveit_config"), "config", srdf_file]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description, value_type=str)
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic, value_type=str
        )
    }

    kinematics_yaml = load_yaml("igus_rebel_moveit_config", "config/kinematics.yaml")

    planning_yaml = load_yaml("igus_rebel_moveit_config", "config/ompl_planning.yaml")

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    commander_node = Node(
        package="igus_rebel_commander",
        executable="igus_rebel_commander",
        name="igus_rebel_commander",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_yaml,
            planning_plugin,
        ],
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_commander"), "rviz", "cmd.rviz"]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_yaml,
            planning_plugin,
        ],
    )

    return [
        commander_node,
        rviz2_node,
    ]
