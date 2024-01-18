# ros2 python imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# requires launching separately the moveit_controller.launch.py file in moveit config

def generate_launch_description():
	
	rviz_file = PathJoinSubstitution(
		[FindPackageShare("igus_rebel_commander"), "rviz", "button_test.rviz"]
	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_file],
	)
	
	 # create node for goal pose publisher
	test_button_pose_computation_node = Node(
		package="igus_rebel_commander",
		executable="test_button_pose_computation",
		name="test_button_pose_computation_node",
		output="screen",
	)
	
	return LaunchDescription([
		rviz_node,
		test_button_pose_computation_node,
	])
