
from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    load_base_arg = DeclareLaunchArgument(
        name="load_base",
        default_value="false",
        description="Load the mobile robot model and tower",
        choices=["true", "false"],
    )

    mount_arg = DeclareLaunchArgument(
        name="mount",
        default_value="mount_v1",
        choices=["none", "mount_v1"],
        description="Which mount to attach to the flange",
    )

    camera_arg = DeclareLaunchArgument(
        name="camera",
        default_value="realsense",
        choices=["realsense", "oakd", "none"],
        description="Which camera to attach to the mount",
    )

    end_effector_arg = DeclareLaunchArgument(
        name="end_effector",
        default_value="toucher_v1",
        choices=["toucher_v1", "none"],
        description="Which end_effector to attach to the mount",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="simulation",
        choices=["mock_hardware", "cri", "simulation", "ignition"],
        description="Which hardware protocol or mock hardware should be used",
    )

    load_gazebo_arg = DeclareLaunchArgument(
        name="load_gazebo",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not Gazebo Ignition is used",
    )

    jsp_gui_arg = DeclareLaunchArgument(
        name="jsp_gui",
        default_value="true",
        choices=["true", "false"],
        description="load joint state publisher gui to send joint states in ROS",
    )

    return LaunchDescription(
        [
            load_base_arg,
            mount_arg,
            camera_arg,
            end_effector_arg,
            hardware_protocol_arg,
            load_gazebo_arg,
            jsp_gui_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):

    # Sim time
    if LaunchConfiguration("load_gazebo").perform(context) == 'true':
        use_sim_time = True
    else:
        use_sim_time = False

    robot_description_filename = "robot.urdf.xacro"

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf",
            robot_description_filename,
        ]
    )

    # load robot description xacro
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " load_base:=",
            LaunchConfiguration("load_base"),
            " mount:=",
            LaunchConfiguration("mount"),
            " camera:=",
            LaunchConfiguration("camera"),
            " end_effector:=",
            LaunchConfiguration("end_effector"),
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
            " load_gazebo:=",
            LaunchConfiguration("load_gazebo"),
            " moveit:=",
            "false",
        ]
    )

    # remap input and ouytput topics for robot state publisher and joint state publisher
    remappings = [
        ("/joint_states", "/rebel/joint_states"),
        ("/robot_description", "/rebel/robot_description"),
    ]

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': use_sim_time},
        ],
        # subscribes to /rebel/joint_states and publishes to /rebel/robot_description
        # remappings=remappings
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(LaunchConfiguration("load_gazebo")),
        # publishes to /rebel/joint_states
        # remappings=remappings
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}],
        # publishes to /rebel/joint_states and receives URDF from /rebel/robot_description
        # remappings=remappings
    )

    # Ignition node
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(
                "igus_rebel_gazebo_ignition"), '/launch', '/ignition.launch.py'
        ]),
        launch_arguments={
            "use_sim_time": "True",
            "moveit": "false"
        }.items(),
        condition=IfCondition(LaunchConfiguration("load_gazebo"))
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare("igus_rebel_description_ros2"),
        "rviz", "rebel.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        ignition_launch,
        rviz_node,
    ]
