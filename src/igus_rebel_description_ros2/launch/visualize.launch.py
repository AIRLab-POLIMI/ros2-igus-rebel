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


def generate_launch_description():
    # Equals the name of the file in irc_ros_description/urdf/ except the file ending
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="igus_rebel_mod",
        description="Which robot to use",
    )

    # Required to concatenate name with .urdf.xacro
    xacro_filename_arg = DeclareLaunchArgument(
        name="xacro_filename",
        default_value=[LaunchConfiguration("robot_name"), ".urdf.xacro"],
        description="Name of the .urdf.xacro file to use",
    )

    # Fully qualified path to the urdf/xacro file, can also be used directly
    # for files outside the irc_ros_description package.
    xacro_path_arg = DeclareLaunchArgument(
        name="xacro_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("igus_rebel_description_ros2"),
                "urdf",
                LaunchConfiguration("xacro_filename"),
            ]
        ),
        description="Path to the .urdf.xacro file to use",
    )

    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )

    xacro_path = LaunchConfiguration("xacro_path")
    # Parameters for rebel.urdf.xacro
    gripper = LaunchConfiguration("gripper")

    robot_description = ParameterValue(Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " gripper:=",
            gripper,
        ]
    ), value_type=str)

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_description_ros2"), "rviz", "rebel.rviz"]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
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

    description = LaunchDescription()

    # Launch args
    description.add_action(robot_name_arg)
    description.add_action(xacro_filename_arg)
    description.add_action(xacro_path_arg)
    description.add_action(gripper_arg)

    # Nodes
    description.add_action(robot_state_publisher_node)
    description.add_action(joint_state_publisher_gui_node)

    # Visualization
    description.add_action(rviz_node)

    return description
