
# python packages
import os
from os import environ

# ros2 python packages
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    spawn_x_arg = DeclareLaunchArgument(
        name="spawn_x",
        default_value="-2.0",
        description="x position for the robot spawned in Gazebo Ignition",
    )

    spawn_y_arg = DeclareLaunchArgument(
        name="spawn_y",
        default_value="0.0",
        description="y position for the robot spawned in Gazebo Ignition",
    )

    spawn_z_arg = DeclareLaunchArgument(
        name="spawn_z",
        default_value="0.0",
        description="z position for the robot spawned in Gazebo Ignition",
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        name="spawn_yaw",
        default_value="-1.0",
        description="Y position for the robot spawned in Gazebo Ignition",
    )

    env_gazebo_package_arg = DeclareLaunchArgument(
        name="env_gazebo_package",
        default_value="default",
        description="Package where the gazebo world and configuration are located. Requires full name of the package, otherwise it will default to this package.",
    )

    full_world_name_arg = DeclareLaunchArgument(
        name="full_world_name",
        default_value="default.sdf",
        description="Name of the world to be loaded in Gazebo Ignition of the type: name.sdf",
    )

    return LaunchDescription(
        [
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            env_gazebo_package_arg,
            full_world_name_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):

    # Ignition env variables
    env = {  # IGN GAZEBO FORTRESS env variables
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ":".join(
            [
                environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
            ]
        ),
    }

    # Ignition world package
    env_gazebo_package = LaunchConfiguration(
        "env_gazebo_package").perform(context)
    full_world_name = LaunchConfiguration("full_world_name").perform(context)

    if env_gazebo_package != 'default':
        ignition_models_path = os.path.join(
            get_package_share_directory(env_gazebo_package), "models",
        )
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ignition_models_path

    # Additional bridge for joint state if Moveit is not used (only for visualization of the description)
    # and related gui with or without the joint position controller gui
    if (LaunchConfiguration("moveit").perform(context) == 'false'):
        bridge_config_filename = "bridge_description.yaml"
        gazebo_config_gui_filename = "gazebo_gui_description.config"
    else:
        bridge_config_filename = "bridge_moveit.yaml"
        gazebo_config_gui_filename = "gazebo_gui_moveit.config"

    if env_gazebo_package == 'default':
        package_name = "igus_rebel_gazebo_ignition"
    else:
        package_name = env_gazebo_package

    # World SDF path
    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        full_world_name,
    )

    # Bridge config
    bridge_config_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        bridge_config_filename,
    )

    # Gazebo gui config
    gazebo_config_gui_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        gazebo_config_gui_filename,
    )

    # Ignition processes
    ign_sim = ExecuteProcess(
        cmd=[
            "ign gazebo",
            "--verbose 1 -r --gui-config " + gazebo_config_gui_path,
            world_path,
        ],
        output="log",
        additional_env=env,
        shell=True,
    )

    ign_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "igus_rebel",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw")
        ],
        parameters=[{'use_sim_time': True},],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output="screen",
    )

    return [
        ign_sim,
        ign_spawn_entity,
        ign_bridge,
    ]
