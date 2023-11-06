# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py
# and the moveit_py example
# https://github.com/peterdavidfagan/moveit2_config_tutorials/blob/moveit_py_motion_planning_python_api_tutorial/doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py


from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

from moveit_configs_utils import MoveItConfigsBuilder

# launches only the URDF version 2 robot description
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

	return LaunchDescription([
		gripper_arg,
		hardware_protocol_arg,
		OpaqueFunction(function=launch_setup)
	])


def launch_setup(context, *args, **kwargs):

	ros2_controllers_file = PathJoinSubstitution([
		FindPackageShare("igus_rebel_moveit_config"),
		"config",
		"ros2_controllers.yaml",
	])
	
	if (LaunchConfiguration("gripper").perform(context) == "camera"):
		srdf_file = "igus_rebel_camera.srdf"
	else:
		srdf_file = "igus_rebel_base.srdf"

	moveit_config = (
		MoveItConfigsBuilder(robot_name="igus_rebel")
		.robot_description(
			file_path="config/igus_rebel.urdf.xacro",
			mappings={
				"hardware_protocol": LaunchConfiguration("hardware_protocol"),
				"gripper": LaunchConfiguration("gripper")
			},
		)
		.robot_description_semantic(file_path= "config/" + srdf_file)
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

	return [
		move_group_node,
		control_node,
		robot_state_publisher_node,
		joint_state_broadcaster_node,
		rebel_arm_controller_node,
	]