from ament_index_python.packages import get_package_share_directory

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

	# include launch file from igus_rebel_moveit_config
	moveit_launch_file = PathJoinSubstitution(
		[get_package_share_directory("igus_rebel_moveit_config"), "launch", "moveit_controller.launch.py"])
	igus_rebel_moveit_config_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(moveit_launch_file),
	)

	rviz_file = PathJoinSubstitution(
		[FindPackageShare("igus_rebel_moveit_config"), "rviz", "moveit.rviz"]
	)

	rviz2_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_file]
	)

	return LaunchDescription([
		igus_rebel_moveit_config_launch,
		rviz2_node,
	])
