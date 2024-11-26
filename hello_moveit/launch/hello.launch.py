from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def load_yaml(package_name, file_path):
    """Load a yaml file from the specified package"""
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except (
            EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    kinematics_yaml = load_yaml("igus_rebel_moveit_config", "config/kinematics.yaml")
    kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Define hello_moveit node
    hello_moveit_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[kinematics],
    )

    return LaunchDescription([hello_moveit_node])
