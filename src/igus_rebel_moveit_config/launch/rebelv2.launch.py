# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py
# and the moveit_py example
# https://github.com/peterdavidfagan/moveit2_config_tutorials/blob/moveit_py_motion_planning_python_api_tutorial/doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="mock_hardware",
        choices=["mock_hardware", "cri", "fake"],
        description="Which hardware protocol or mock hardware should be used",
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_moveit_config"), "rviz", "moveit.rviz"]
    )

    ros2_controllers_file = PathJoinSubstitution([
        FindPackageShare("igus_rebel_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    ])



    moveit_config = (
        MoveItConfigsBuilder(robot_name="igus_rebel")
        .robot_description(
            file_path="config/igus_rebel.urdf.xacro",
            mappings={
                "hardware_protocol": LaunchConfiguration("hardware_protocol"),
                "gripper": LaunchConfiguration("gripper")
            },
        )
        .robot_description_semantic(file_path="config/igus_rebel.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_planning_scene=True,
                                publish_geometry_updates=True,
                                publish_state_updates=True,
                                publish_transforms_updates=True,
                                publish_robot_description=False,
                                publish_robot_description_semantic=False)
		#.moveit_cpp(file_path="config/moveit_py.yaml")
        #.sensors_3d()
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_file],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="both"
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    rebel_arm_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_arm_controller",
                   "--controller-manager", "/controller_manager"],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_scene_monitor,
            moveit_config.trajectory_execution,
        ],
    )

    return LaunchDescription([
        gripper_arg,
        hardware_protocol_arg,
        move_group_node,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        rebel_arm_controller_node,
        rviz2_node,
    ])
