
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

from moveit_launch import moveit_loader

# remember to start the moveit_controllers.launch.py file from the igus_rebel_moveit_config package first
# it will start the necessary controllers and the moveit dependencies
# start it with the correct gripper and hardware protocol arguments, and leaving rviz_file:=none (default)

# launches only the URDF version 2 robot description
def generate_launch_description():

    args = moveit_loader.declare_arguments()

    commander_node = Node(
        package="igus_rebel_commander",
        executable="commander_demo",
        name="commander_demo_node",
        parameters=moveit_loader.load_moveit(with_sensors3d=False),
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
            moveit_loader.load_robot_description(),
            moveit_loader.load_robot_description_semantic()
        ],
    )

    return LaunchDescription(
        args + [commander_node, rviz2_node]
    )
