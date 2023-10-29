import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_yaml(package_name, file_path):
	"""Load a yaml file from the specified package"""
	full_path = os.path.join(
		get_package_share_directory(package_name), file_path)
	try:
		with open(full_path, "r") as file:
			return yaml.safe_load(file)
	except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
		return None
	

# use this script for launching everything in a single terminal window
# launches the moveit config and the commander node together with rviz2

# use demo.launch.py instead to launch the moveit config and the commander node in separate terminal windows

def generate_launch_description():

	# include launch file from igus_rebel_moveit_config
	moveit_launch_file = PathJoinSubstitution(
		[get_package_share_directory("igus_rebel_moveit_config"), "launch", "moveit_controller.launch.py"])
	igus_rebel_moveit_config_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(moveit_launch_file),
	)

	
	gripper_arg = DeclareLaunchArgument(
		name="gripper",
		default_value="none",
		choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
		description="Which gripper to attach to the flange",
	)

	hardware_protocol_arg = DeclareLaunchArgument(
		name="hardware_protocol",
		default_value="simulation",
		choices=["mock_hardware", "cri", "simulation"],
		description="Which hardware protocol or mock hardware should be used",
	)

	robot_description_file = PathJoinSubstitution([    
		get_package_share_directory("igus_rebel_description_ros2"),
		"urdf",
		"igus_rebel_mod.urdf.xacro",  # baseline version of the robot
	])
	
	robot_description = Command([
		FindExecutable(name="xacro"), " ", robot_description_file,
		" hardware_protocol:=", LaunchConfiguration("hardware_protocol"),
		" gripper:=", LaunchConfiguration("gripper"),
	])
	robot_description_semantic_file = PathJoinSubstitution([
		get_package_share_directory("igus_rebel_moveit_config"),
		"config",
		"igus_rebel.srdf",
	])
	robot_description_semantic = Command([
		FindExecutable(name="xacro"), " ", robot_description_semantic_file, 
	])
	robot_description = {"robot_description": ParameterValue(robot_description, value_type=str)}

	robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic, value_type=str)}

	kinematics_yaml = load_yaml(
		"igus_rebel_moveit_config", "config/kinematics.yaml"
	)

	planning_yaml = load_yaml(
		"igus_rebel_moveit_config", "config/ompl_planning.yaml"
	)

	planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

	commander_node = Node(
		package="igus_rebel_commander",
		executable="igus_rebel_commander_v2",
		name="igus_rebel_commander_v2",
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
		]
	)

	return LaunchDescription([
		hardware_protocol_arg,
		gripper_arg,
		igus_rebel_moveit_config_launch,
		commander_node,
		rviz2_node,
	])
