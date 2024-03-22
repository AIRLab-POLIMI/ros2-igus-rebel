cmake_minimum_required(VERSION 3.8)
project(igus_rebel_gripper_controller LANGUAGES CXX)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(INCLUDE_DEPENDS
	hardware_interface
	control_msgs
	std_msgs
	pluginlib
	rclcpp
	rclcpp_lifecycle
	controller_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)

# # COMPILE
add_library(
	igus_rebel_gripper_controller
	SHARED
	src/gripper_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(
	igus_rebel_gripper_controller
	PUBLIC
	${INCLUDE_DEPENDS}
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface gripper_controller.xml)

include_directories(include)

# -------------- ROS2 declaration
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${INCLUDE_DEPENDS})
ament_export_dependencies(${INCLUDE_DEPENDS})
ament_export_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()


ament_package()