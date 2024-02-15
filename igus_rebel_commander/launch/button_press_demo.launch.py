# python imports
import os
import yaml

# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
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
    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="mount_v1",
        choices=["mount_v1"], # required mount to attach to the last joint
        description="Mount to attach to the last joint",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="realsense",
        choices=["realsense"], # required realsense camera 
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="toucher_v1", 
        choices=["toucher_v1"], # required toucher end effector
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or simulation environment should be used",
    )

    # read camera frame from ros2_aruco config file
    config_file = os.path.join(
        get_package_share_directory("ros2_aruco_pose_estimation"), "config", "aruco_parameters.yaml"
    )

    # load yaml file
    with open(config_file, "r") as f:
        config_yaml = yaml.safe_load(f.read())
        camera_frame = config_yaml["/aruco_node"]["ros__parameters"]["camera_frame"]

    # camera frame name argument to pass to the node
    camera_frame_arg = DeclareLaunchArgument(
        name="camera_frame",
        # set camera frame arg equal to the camera frame from the yaml file
        default_value=TextSubstitution(text=camera_frame),
        description="Camera frame of the aruco markers detected",
    )

    return LaunchDescription(
        [
            mount_arg,
            end_effector_arg,
            camera_arg,
            hardware_protocol_arg,
            camera_frame_arg,
            load_base_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    robot_description_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_description_ros2"),
            "urdf",
            "robot.urdf.xacro",  # baseline version of the robot
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
            " load_base:=",
            LaunchConfiguration("load_base"),
            " mount:=",
            LaunchConfiguration("mount"),
            " camera:=",
            LaunchConfiguration("camera"),
            " end_effector:=",
            LaunchConfiguration("end_effector"),
            " load_gazebo:=false",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description, value_type=str)
    }
    
    robot_description_semantic_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_moveit_config"), "config", "igus_rebel.srdf.xacro"]
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

    kinematics_yaml = load_yaml("igus_rebel_moveit_config", "config/kinematics.yaml")
    kinematics = {"robot_description_kinematics": kinematics_yaml}

    ompl_planning_yaml = load_yaml(
        "igus_rebel_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config = {"move_group": {}}
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    button_presser_demo_node = Node(
        package="igus_rebel_commander",
        executable="button_presser_demo",
        name="button_presser_demo_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            ompl_planning_pipeline_config,
            planning_scene_monitor_parameters,
            planning_plugin,
            {
                "camera_frame": LaunchConfiguration("camera_frame"),
                "load_base": LaunchConfiguration("load_base"),
            }
        ],
    )	

    
    rviz_file_name = "button_presser.rviz"

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_commander"), "rviz", rviz_file_name]
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
            kinematics,
            ompl_planning_pipeline_config,
        ],
    )

    return [
        button_presser_demo_node,
        rviz2_node,
    ]
