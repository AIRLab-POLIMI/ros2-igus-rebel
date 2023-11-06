# python imports
import os
import yaml

# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition
from launch.actions import LogInfo

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	# A node to publish world -> panda_link0 transform
	static_tf = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_transform_publisher",
		output="log",
		arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
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

	# create node for goal pose publisher
	goal_pose_publisher_node = Node(
		package="igus_rebel_servo",
		executable="goal_pose_publisher",
		name="goal_pose_publisher_node",
		output="screen",
		parameters=[{"camera_frame": LaunchConfiguration("camera_frame")}],
	)

	rviz_file = PathJoinSubstitution(
		[FindPackageShare("igus_rebel_servo"), "rviz", "servo_demo.rviz"]
	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_file],
	)

	return LaunchDescription(
		[rviz_node, static_tf, camera_frame_arg, goal_pose_publisher_node]
	)
