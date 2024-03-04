#!/usr/bin/env python3

# python imports
import os
import yaml

# ros2 imports
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue


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


def declare_arguments():
    """ Returns list of launch arguments """
    rviz_file_arg = DeclareLaunchArgument(
        name="rviz_file",
        default_value="none",
        description="Path to the RViz configuration file",
    )

    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="mount_v1",
        choices=["none", "mount_v1"],
        description="Mount to attach to the last joint",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="realsense",
        choices=["realsense", "oakd", "none"],
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="toucher_v1",
        choices=["toucher_v1", "none"],
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or simulation environment should be used",
    )

    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Which Gazebo version to launch",
    )

    load_octomap_arg = DeclareLaunchArgument(
        name="load_octomap",
        default_value="false",
        description="Load the octomap server inside the planning scene",
        choices=["true", "false"],
    )
    return [
        rviz_file_arg,
        load_base_arg,
        mount_arg,
        camera_arg,
        end_effector_arg,
        hardware_protocol_arg,
        load_gazebo_arg,
        load_octomap_arg
    ]


def load_robot_description():
    """Load the robot description URDF"""

    robot_description_filename = "robot.urdf.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf",
            robot_description_filename,
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " load_base:=",
            LaunchConfiguration("load_base"),
            " mount:=",
            LaunchConfiguration("mount"),
            " camera:=",
            LaunchConfiguration("camera"),
            " end_effector:=",
            LaunchConfiguration("end_effector"),
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
            " load_gazebo:=",
            LaunchConfiguration("load_gazebo"),
            " moveit:=true",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    return robot_description


def load_robot_description_semantic():
    """ Load robot semantic description SRDF file """

    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "igus_rebel.srdf.xacro",
        ]
    )

    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
            " load_base:=",
            LaunchConfiguration("load_base"),
            " mount:=",
            LaunchConfiguration("mount"),
            " camera:=",
            LaunchConfiguration("camera"),
            " end_effector:=",
            LaunchConfiguration("end_effector"),
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }
    return robot_description_semantic


def load_ros2_controllers():
    """ Load ROS2 controllers yaml """

    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    return ros2_controllers_file


def load_moveit(with_sensors3d: bool) -> list:
    """ Loads parameters required by move_group node interface """

    ompl_planning_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config = {"move_group": {}}
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_plugin": "ompl_interface/OMPLPlanner",
    }

    kinematics_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/kinematics.yaml")
    kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_controllers_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/moveit_controllers.yaml"
    )

    # unused
    move_it_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/moveit_py.yaml")

    joint_limits_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/joint_limits.yaml"
    )
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # add sensors3d to the list of moveit parameters
    if (with_sensors3d):
        sensors_3d_yaml = load_yaml(
            "igus_rebel_moveit_config", "config/sensors_3d.yaml"
        )
    else:
        sensors_3d_yaml = {"sensors:": ""}

    return [
        load_robot_description(),
        load_robot_description_semantic(),
        ompl_planning_pipeline_config,
        planning_scene_monitor_parameters,
        kinematics,
        moveit_controllers_yaml,
        joint_limits,
        sensors_3d_yaml
    ]


def load_camera_frame_arg():
    camera_config_yaml = load_yaml("aruco_pose_estimation", "config/aruco_parameters.yaml")
    # get camera frame from the yaml file
    camera_frame = camera_config_yaml["/aruco_node"]["ros__parameters"]["camera_frame"]

    # camera frame name argument to pass to the node
    camera_frame_arg = DeclareLaunchArgument(
        name="camera_frame",
        # set camera frame arg equal to the camera frame from the yaml file
        default_value=TextSubstitution(text=camera_frame),
        description="Camera frame of the aruco markers detected",
    )
    return camera_frame_arg
