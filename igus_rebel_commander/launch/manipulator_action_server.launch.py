
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.actions import TimerAction

# load moveit controllers and parameters from yaml and robot description files
from moveit_launch import moveit_loader


# remember to start the moveit_controllers.launch.py file from the igus_rebel_moveit_config package first
# it will start the necessary controllers and the moveit dependencies
# start it with the correct arguments, and leaving rviz_file:=none (default)

# launches only the URDF version 2 robot description
def generate_launch_description():

    args = moveit_loader.declare_arguments()

    camera_frame_arg = moveit_loader.load_camera_frame_arg()
    args.append(camera_frame_arg)

    # create node for goal pose publisher
    manipulator_node = Node(
        package="igus_rebel_commander",
        executable="manipulator_action_server",
        name="manipulator_node",
        output="screen",
        parameters=moveit_loader.load_moveit(with_sensors3d=False) + [{
                "camera_frame": LaunchConfiguration("camera_frame"),
                "load_base": LaunchConfiguration("load_base"),
        }],
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_commander"), "rviz", "aruco_demo.rviz"]
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

    return LaunchDescription( args + [
        manipulator_node,
        rviz2_node,
    ])
