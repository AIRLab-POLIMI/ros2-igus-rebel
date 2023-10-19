# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py
# and the moveit_py example
# https://github.com/peterdavidfagan/moveit2_config_tutorials/blob/moveit_py_motion_planning_python_api_tutorial/doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import load_yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from launch_ros.parameter_descriptions import ParameterValue


def opaque_test(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz")
    gripper = LaunchConfiguration("gripper")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("igus_rebel_moveit2_config"), "rviz", "moveit.rviz"]
    )

    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )


    joint_limits_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
            "joint_limits.yaml",
        ]
    )


    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description_ros2"),
            "urdf",
            "igus_rebel_mod.urdf.xacro", # baseline version of the robot
        ]
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " hardware_protocol:=",
            hardware_protocol,
            " gripper:=",
            gripper,
        ]
    )
    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
            "igus_rebel_mod.srdf.xacro",
        ]
    )
    robot_description_semantic = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
        ]
    )


    controllers = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
        	"controllers.yaml",
		]
	)
            
    controllers_dict = load_yaml(Path(controllers.perform(context)))

    ompl_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
            "ompl.yaml",
        ]
    )
    ompl = {"ompl": load_yaml(Path(ompl_file.perform(context)))}

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_dict,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    robot_description_kinematics_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit2_config"),
            "config",
            "kinematics.yaml",
        ]
    )

    planning_pipeline = {
        "move_group": {
            # NOTE: Copied from UR ROS2 for testing purposes, update configuration for the rebel
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
        # "move_group": {
        #     "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        #     "request_adapters": "default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
        #     "default_planner_config": "PTP",
        #     "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
        # },
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_args_not_concatenated = [
        {"robot_description": robot_description.perform(context)},
        {"robot_description_semantic": robot_description_semantic.perform(context)},
        load_yaml(Path(robot_description_kinematics_file.perform(context))),
        load_yaml(Path(joint_limits_file.perform(context))),
        moveit_controllers,
        planning_scene_monitor_parameters,
        planning_pipeline,
        {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        ompl,
        # {"planning_pipeline": {"planning_plugin": "ompl_rrt_star"}},
    ]

    # Concatenate all dictionaries together, else moveitpy won't read all parameters
    moveit_args = dict()
    for d in moveit_args_not_concatenated:
        moveit_args.update(d)

	# prints the entire configuration code at launch startup
	# commented to avoid cluttering the terminal
	# print(moveit_args) 

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        #namespace=namespace,
        parameters=[
            moveit_args,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #namespace=namespace,
        parameters=[
            moveit_args,
            ros2_controllers_file,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        #namespace=namespace,
        parameters=[
            moveit_args,
        ],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rebel_arm_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        arguments=["rebel_arm_controller", "-c", controller_manager_name],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            # Passing the entire dict to rviz results in an error with the joint limits
            {"robot_description": ParameterValue(robot_description, value_type=str)},
        ],
        condition=IfCondition(use_rviz),
    )

    return [
        move_group_node,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_node,
        rebel_arm_controller_node,
        rviz_node,
    ]


def generate_launch_description():
    #namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        #default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
		default_value=["controller_manager"],
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start rviz with the launch file",
    )
    gripper_arg = DeclareLaunchArgument(
        name="gripper",
        default_value="none", 
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )
    
    hardware_protocol_arg = DeclareLaunchArgument(
        name="hardware_protocol",
        default_value="mock_hardware",
        choices=["mock_hardware", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )
    
    ld = LaunchDescription()

    #ld.add_action(namespace_arg)
    ld.add_action(controller_manager_name_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(gripper_arg)
    ld.add_action(hardware_protocol_arg)


    ld.add_action(OpaqueFunction(function=opaque_test))

    return ld
