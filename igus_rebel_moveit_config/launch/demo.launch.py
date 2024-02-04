from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # Gazebo specific arguments
    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Which Gazebo version to launch",
    )

    return LaunchDescription(
        [
            load_gazebo_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):

    # When Gazebo is loaded, then use the simulation time also in Moveit2
    if LaunchConfiguration("load_gazebo").perform(context) == "true":
        use_sim_time = "True"
    else:
        use_sim_time = "False"

    # launch rviz
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_moveit_config"), "rviz", "moveit.rviz"]
    )

    # include launch file from igus_rebel_moveit_config
    moveit_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "launch",
            "moveit_controller.launch.py",
        ]
    )

    igus_rebel_moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={"rviz_file": rviz_file,
                          "use_sim_time": use_sim_time}.items(),
    )

    # include launch file from igus_rebel_gazebo_ignition
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("igus_rebel_gazebo_ignition"),
                "/launch",
                "/ignition.launch.py",
            ]
        ),
        launch_arguments={
            "moveit": "true",
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("load_gazebo")),
    )

    return [
        igus_rebel_moveit_config_launch,
        ignition_launch,
    ]
