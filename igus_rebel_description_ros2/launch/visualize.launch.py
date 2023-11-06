# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction


def generate_launch_description():
    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "camera"],
        description="Which gripper mount to attach to the flange",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation"],
        description="Which hardware protocol or mock hardware should be used",
    )

    robot_desc_arg = DeclareLaunchArgument(
        name="urdf",
        default_value="mod",
        description="Robot Description file to use",
        choices=["base", "mod"],
    )

    return LaunchDescription(
        [
            gripper_arg,
            hardware_protocol_arg,
            robot_desc_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    if LaunchConfiguration("urdf").perform(context) == "mod":
        desc_file = "igus_rebel_mod.urdf.xacro"
    else:
        desc_file = "igus_rebel.urdf.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf",
            desc_file,
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

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_description_ros2"), "rviz", "rebel.rviz"]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]
