
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from ament_index_python.packages import get_package_share_directory, FindPackageShareDirectory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # TODO: remove this piece once the URDF is updated
    # TODO: add camera holder urdf specification and the static tf transforms to camera link
    # TODO: change SRDF considering the camera holder to avoid self collisions
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(parameter_namespace="igus_rebel_servo", file_path="config/igus_rebel_servo.yaml")
        .to_dict()
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="igus_rebel_servo",
        executable="realtime_servoing",
        name="realtime_servoing_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # RViz
    rviz_config_file = FindPackageShareDirectory(
        ["igus_rebel_servo", "rviz", "servo_demo.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Load controllers, robot state publisher, joint state publisher from moveit config launch file
    moveit_config_launch_file = FindPackageShareDirectory(
        ["igus_rebel_moveit_config", "launch", "moveit_controller.launch.py"])
    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_launch_file),
    )

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, moveit_config_launch]
    )
