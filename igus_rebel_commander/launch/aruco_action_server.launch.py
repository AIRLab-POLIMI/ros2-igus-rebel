
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

    testing_arg = DeclareLaunchArgument(
        name="testing",
        default_value="false",
        description="Test: whether to launch test goal pose publisher node",
        choices=["true", "false"],
    )

    camera_frame_arg = moveit_loader.load_camera_frame_arg()

    args.append(camera_frame_arg)
    args.append(testing_arg)
    args.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(args)


def launch_setup(context, *args, **kwargs):
    
    aruco_follower_node = Node(
        package="igus_rebel_commander",
        executable="aruco_follower",
        name="aruco_follower_node",
        parameters=moveit_loader.load_moveit(with_sensors3d=False),
    )

    # create node for goal pose publisher
    aruco_action_server_node = Node(
        package="igus_rebel_commander",
        executable="aruco_action_server",
        name="aruco_action_server_node",
        output="screen",
        parameters=[
            {
                "camera_frame": LaunchConfiguration("camera_frame"),
                "testing": LaunchConfiguration("testing"),
            }
        ],
    )

    # test node for publishing a goal pose to check whether the goal pose is computed correctly
    # test_goal_pose_computation_node = Node(
    #     package="igus_rebel_commander",
    #     executable="test_goal_pose_computation",
    #     name="test_goal_pose_computation_node",
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration("testing")),
    # )

    if (LaunchConfiguration("testing").perform(context) == "true"):
        rviz_file_name = "aruco_pose_test.rviz"
    else:
        rviz_file_name = "aruco_demo.rviz"

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_commander"), "rviz", rviz_file_name]
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

    return [
        aruco_follower_node,
        aruco_action_server_node,
        TimerAction(period=1.0, actions=[test_goal_pose_computation_node]),
        rviz2_node,
    ]
