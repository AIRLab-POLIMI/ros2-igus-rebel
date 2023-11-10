cmake_minimum_required(VERSION 3.8)
project(igus_rebel_servo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(realtime_servoing src/realtime_servoing.cpp)
ament_target_dependencies(realtime_servoing
	rclcpp moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_servo)

add_executable(goal_pose_publisher src/goal_pose_publisher.cpp)
ament_target_dependencies(goal_pose_publisher
	rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

install(
	TARGETS realtime_servoing
	DESTINATION lib/${PROJECT_NAME}
)

install(
	TARGETS goal_pose_publisher
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config include
	DESTINATION share/${PROJECT_NAME}
)

ament_package()