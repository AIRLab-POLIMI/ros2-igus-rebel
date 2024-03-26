
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_launch import moveit_loader


def generate_launch_description():

    args = moveit_loader.declare_arguments()

    load_rviz_arg = DeclareLaunchArgument(
        name="load_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Whether or not Rviz is used",
    )

    args.append(load_rviz_arg)

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            moveit_loader.load_robot_description(),
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=UnlessCondition(LaunchConfiguration("load_gazebo")),
    )

    # Ignition node
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(
                "igus_rebel_gazebo_ignition"), '/launch', '/ignition.launch.py'
        ]),
        launch_arguments={
            "use_sim_time": "True",
            "moveit": "false"
        }.items(),
        condition=IfCondition(LaunchConfiguration("load_gazebo"))
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare("igus_rebel_description_ros2"),
        "rviz", "rebel.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(LaunchConfiguration("load_rviz")),
    )

    return LaunchDescription(args + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        ignition_launch,
        rviz_node
    ])
