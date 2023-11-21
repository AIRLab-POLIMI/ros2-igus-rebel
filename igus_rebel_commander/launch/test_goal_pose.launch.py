# python imports
import os
from py import test
import yaml

# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, Command, FindExecutable
from launch_ros.descriptions import ParameterValue
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

# requires launching separately the moveit_controller.launch.py file in moveit config

def generate_launch_description():
	gripper_arg = DeclareLaunchArgument(
		name="gripper",
		default_value="camera",
		choices=["none", "camera"],
		description="Gripper mount to attach to the last joint",
	)

	hardware_protocol_arg = DeclareLaunchArgument(
		name="hardware_protocol",
		default_value="simulation",
		choices=["mock_hardware", "cri", "simulation"],
		description="Which hardware protocol or mock hardware should be used",
	)

	load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

	# read camera frame from ros2_aruco config file
	config_file = os.path.join(
		get_package_share_directory("ros2_aruco"), "config", "aruco_parameters.yaml"
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
			gripper_arg,
			hardware_protocol_arg,
			camera_frame_arg,
			load_base_arg,
			OpaqueFunction(function=launch_setup),
		]
	)


def launch_setup(context, *args, **kwargs):
	robot_description_file = PathJoinSubstitution(
		[
			FindPackageShare("igus_rebel_description_ros2"),
			"urdf",
			"igus_rebel.urdf.xacro",
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
			" load_base:=",
            LaunchConfiguration("load_base"),
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
			" gripper:=",
			LaunchConfiguration("gripper"),
			" load_base:=",
			LaunchConfiguration("load_base"),
		]
	)

	robot_description_semantic = {
		"robot_description_semantic": ParameterValue(
			robot_description_semantic_content, value_type=str
		)
	}

	rviz_file = PathJoinSubstitution(
		[FindPackageShare("igus_rebel_commander"), "rviz", "goal_demo.rviz"]
	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_file],
		parameters=[robot_description, robot_description_semantic],
	)
	
	 # create node for goal pose publisher
	goal_pose_publisher_node = Node(
		package="igus_rebel_commander",
		executable="goal_pose_publisher",
		name="goal_pose_publisher_node",
		output="screen",
		parameters=[{"camera_frame": LaunchConfiguration("camera_frame")}],
	)
	
	# test node for publishing a goal pose to check whether the goal pose is computed correctly
	test_goal_pose_computation_node = Node(
		package="igus_rebel_commander",
		executable="test_goal_pose_computation",
		name="test_goal_pose_computation_node",
		output="screen",
	)

	return [
		rviz_node,
		goal_pose_publisher_node,
		test_goal_pose_computation_node,
	]
