
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
        launch_arguments={"rviz_file": rviz_file}.items(),
    )

    return LaunchDescription(
        [
            igus_rebel_moveit_config_launch,
        ]
    )
