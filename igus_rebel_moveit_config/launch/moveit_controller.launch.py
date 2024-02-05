# python imports
import os
import yaml

# ros2 imports
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition


def load_yaml(package_name, file_path):
    """Load a yaml file from the specified package"""
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path)
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except (
            EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


# use this script for launching everything in a single terminal window
# launches the moveit config and the commander node together with rviz2

# use demo.launch.py instead to launch the moveit config and the commander node in separate terminal windows


# launches only the URDF version 2 robot description
def generate_launch_description():

    rviz_file_arg = DeclareLaunchArgument(
        name="rviz_file",
        default_value="none",
        description="Path to the RViz configuration file",
    )

    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
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

    return LaunchDescription(
        [
            rviz_file_arg,
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            load_octomap_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):

   # Sim time
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

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

    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

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

    if LaunchConfiguration("load_octomap").perform(context) == "true":
        sensors_3d_yaml = load_yaml(
            "igus_rebel_moveit_config", "config/sensors_3d.yaml"
        )
    else:
        sensors_3d_yaml = {"sensors:": ""}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            sensors_3d_yaml,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        # IRON_ONLY: listening to /robot_description to have the complete URDF
        # parameters=[{"robot_description": ""}, ros2_controllers_file],
        # remappings=[("~/robot_description", "/robot_description")],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # cannot remap /joint_states topic
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rebel_arm_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebel_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_file")],
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("rviz_file"), "' != 'none' "]
            )
        ),
    )

    return [
        rviz2_node,
        joint_state_broadcaster_node,
        rebel_arm_controller_node,
        move_group_node,
        control_node,
        robot_state_publisher_node,
    ]
