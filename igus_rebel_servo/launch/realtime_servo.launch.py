import os
import yaml
from ament_index_python import get_package_share_directory

# ros2 imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
	PathJoinSubstitution,
	Command,
	FindExecutable,
	LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue


def load_yaml(package_name, file_path):
	# Load a yaml file from the specified package
	full_path = os.path.join(get_package_share_directory(package_name), file_path)
	try:
		with open(full_path, "r") as file:
			return yaml.safe_load(file)
	except EnvironmentError:
		# parent of IOError, OSError *and* WindowsError where available
		return None


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

	load_base_arg = DeclareLaunchArgument(
		name="load_base",
		default_value="false",
		description="Load the mobile robot model and tower",
		choices=["true", "false"],
	)

	robot_description_file = PathJoinSubstitution(
		[
			FindPackageShare("igus_rebel_description_ros2"),
			"urdf",
			"igus_rebel.urdf.xacro",  # baseline version of the robot
		]
	)

	robot_description_content = Command(
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

	robot_description = {
		"robot_description": ParameterValue(robot_description_content, value_type=str)
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

	kinematics_yaml = load_yaml("igus_rebel_moveit_config", "config/kinematics.yaml")
	kinematics = {"robot_description_kinematics": kinematics_yaml}
	
	joint_limits_yaml = load_yaml(
		"igus_rebel_moveit_config", "config/joint_limits.yaml"
	)
	joint_limits = {"robot_description_planning": joint_limits_yaml}


	# RViz
	rviz_config_file = PathJoinSubstitution(
		[FindPackageShare("igus_rebel_servo"), "rviz", "servo_demo.rviz"]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_config_file],
		parameters=[
			robot_description,
			robot_description_semantic,
		],
	)

	# This filter parameter should be >1. Increase it for greater smoothing but slower motion.
	low_pass_filter_coeff = {"butterworth_filter_coeff": 30.0}

	# Get parameters for the Servo node
	"""
	servo_params = {
		"moveit_servo": ParameterBuilder("igus_rebel_servo")
		.yaml("config/igus_rebel_servo.yaml")
		.to_dict()
	}
	"""
	servo_params = load_yaml("igus_rebel_servo", "config/igus_rebel_servo.yaml")
	servo_params = {"moveit_servo": servo_params}

	# Launch a standalone Servo node in the realtime_servoing.cpp file. It will try to reach the target pose statically defined.
	#
	# NOTE: moveit_servo::servo_node runs the Servo library using the ROS2 apis. It requires a pose command input topic
	# 		and outputs a velocity command topic (joint trajectory). The input topic is goal pose (can be placed using RViz).
	#		There are no significant differences in the movement with respect to the realtime_servoing.cpp example.
	servo_node = Node(
		package="igus_rebel_servo", # moveit_servo
		executable="realtime_servoing", # servo_node
		name="realtime_servoing", # servo_node
		parameters=[
			servo_params,
			low_pass_filter_coeff,
			robot_description,
			robot_description_semantic,
			kinematics,
			joint_limits
		],
		output="screen",
	)

	return LaunchDescription(
		[
			hardware_protocol_arg,
			gripper_arg,
			load_base_arg,
			rviz_node,
			servo_node,
		]
	)
