
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    


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

    return LaunchDescription([
        igus_rebel_moveit_config_launch,
        ignition_launch,
    ])
