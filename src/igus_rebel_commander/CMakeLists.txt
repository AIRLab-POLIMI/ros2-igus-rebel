cmake_minimum_required(VERSION 3.8)
project(igus_rebel_commander)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_kdl REQUIRED)

add_executable(${PROJECT_NAME}
	src/commander_main.cpp
)

add_executable(${PROJECT_NAME}_v2
	src/commander_v2.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC include)

ament_target_dependencies(${PROJECT_NAME} pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface)

ament_target_dependencies(${PROJECT_NAME}_v2 pluginlib moveit_core moveit_kinematics moveit_visual_tools tf2_kdl moveit_ros_planning_interface)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_v2
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
	DESTINATION include/
)

ament_package()
