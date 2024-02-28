
# ros2 imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition

# python helper file for loading moveit parameters
from moveit_launch import moveit_loader


# launches only the URDF version 2 robot description
def generate_launch_description():

    # load launch arguments
    args = moveit_loader.declare_arguments()
    args.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(args)


def launch_setup(context, *args, **kwargs):

    # Sim time
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    # if octomap is to be loaded, then ask for sensors3d yaml config
    with_sensors_3d = LaunchConfiguration("load_octomap").perform(context) == "true"

    movegroup_parameters = moveit_loader.load_moveit(with_sensors_3d)
    movegroup_parameters.append({"use_sim_time": use_sim_time})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=movegroup_parameters,
        arguments=["--ros-args", "--log-level", "info"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_loader.load_robot_description(),
            moveit_loader.load_ros2_controllers(),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        # IRON_ONLY: listening to /robot_description to have the complete URDF
        # parameters=[{"robot_description": ""}, ros2_controllers_file],
        # remappings=[("~/robot_description", "/robot_description")],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            moveit_loader.load_robot_description(),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # cannot remap /joint_states topic
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rebel_arm_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebel_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_file")],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_loader.load_robot_description(),
            moveit_loader.load_robot_description_semantic(),
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("rviz_file"), "' != 'none' "]
            )
        ),
    )

    return [
        rviz2_node,
        joint_state_broadcaster_node,
        rebel_arm_controller_node,
        move_group_node,
        control_node,
        robot_state_publisher_node,
    ]
