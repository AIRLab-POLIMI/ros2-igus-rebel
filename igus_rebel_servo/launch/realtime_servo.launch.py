import os

# ros2 imports
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# moveit2 imports
from moveit_configs_utils import MoveItConfigsBuilder


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

	# load igus rebel description
	moveit_config = (
		MoveItConfigsBuilder(robot_name="igus_rebel")
		.robot_description(
			file_path="config/igus_rebel.urdf.xacro",
			mappings={
				"hardware_protocol": LaunchConfiguration("hardware_protocol"),
				"gripper": LaunchConfiguration("gripper")
			},
		)
		.robot_description_semantic(file_path= "config/igus_rebel_camera.srdf")
		.robot_description_kinematics(file_path="config/kinematics.yaml")
		.trajectory_execution(file_path="config/moveit_controllers.yaml")
		.joint_limits(file_path="config/joint_limits.yaml")
		.planning_pipelines(pipelines=["ompl"])
		.planning_scene_monitor(publish_planning_scene=True,
								publish_geometry_updates=True,
								publish_state_updates=True,
								publish_transforms_updates=True,
								publish_robot_description=True,
								publish_robot_description_semantic=True)
		# .moveit_cpp(file_path="config/moveit_py.yaml")
		# .sensors_3d() # add this line when implementing Octomap integration
		.to_moveit_configs()
	)

	# Get parameters for the Servo node
	servo_params = {
		"moveit_servo": ParameterBuilder("igus_rebel_servo")
		.yaml("config/igus_rebel_servo.yaml")
		.to_dict()
	}

	# This filter parameter should be >1. Increase it for greater smoothing but slower motion.
	low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}

	# RViz
	rviz_config_file = (
		get_package_share_directory("igus_rebel_servo") + "/rviz/servo_demo.rviz"
	)
	rviz_node = launch_ros.actions.Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_config_file],
		parameters=[
			moveit_config.robot_description,
			moveit_config.robot_description_semantic,
		],
	)

	# ros2_control using FakeSystem as hardware
	ros2_controllers_file = PathJoinSubstitution([
		FindPackageShare("igus_rebel_moveit_config"),
		"config",
		"ros2_controllers.yaml",
	])

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
		parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic],
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

	# Launch a standalone Servo node.
	# As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
	servo_node = Node(
		package="igus_rebel_servo",
		executable="realtime_servoing",
		name="realtime_servoing",
		parameters=[
			servo_params,
			#low_pass_filter_coeff,
			moveit_config.robot_description,
			moveit_config.robot_description_semantic,
			moveit_config.robot_description_kinematics,
		],
		output="screen",
	)

	return launch.LaunchDescription(
		[
			hardware_protocol_arg,
			gripper_arg,
			rviz_node,
			control_node,
			joint_state_broadcaster_node,
			rebel_arm_controller_node,
			servo_node,
			robot_state_publisher_node,
		]
	)
