import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='igus_rebel_hw_controller',
            executable='igus_rebel_node',
            name='igus_rebel_hw_controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='controller_manager',
            executable='controller_manager',
            name='controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[ {'use_gui': 'true'} ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        #TODO: change this with a reference to the xacro description file
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'igus_rebel_description_ros2'), 'launch/igus_rebel_robot.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
