cmake_minimum_required(VERSION 3.8)
project(igus_rebel_commander)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

# commander demo --> move group interface tutorial with predefined goals: cartesian space and joint space goals
add_executable(commander_demo
	src/commander.cpp
)

ament_target_dependencies(commander_demo
	pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface)

# button presser demo --> move the robot to press 3 buttons in a row for a demo
add_executable(button_presser_demo
	src/button_presser.cpp
)

ament_target_dependencies(button_presser_demo
	pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface
	std_msgs geometry_msgs tf2_ros tf2_geometry_msgs ros2_aruco_interfaces)

# goal pose publisher --> given aruco pose, publishes feasible goal pose for robot
add_executable(goal_pose_publisher src/goal_pose_publisher.cpp)
ament_target_dependencies(goal_pose_publisher
	rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
target_include_directories(goal_pose_publisher PRIVATE include)

# test goal pose computation --> publishes predefined aruco pose for testing and debugging
add_executable(test_goal_pose_computation test/test_goal_pose_computation.cpp)
ament_target_dependencies(test_goal_pose_computation
	rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# aruco follower --> given aruco pose, the robot "follows" it
add_executable(aruco_follower
	src/aruco_follower.cpp)
ament_target_dependencies(aruco_follower
	pluginlib moveit_core moveit_kinematics moveit_visual_tools moveit_ros_planning_interface)

install(TARGETS commander_demo button_presser_demo goal_pose_publisher test_goal_pose_computation aruco_follower
	DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch rviz
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
