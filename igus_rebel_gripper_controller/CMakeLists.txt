cmake_minimum_required(VERSION 3.8)
project(igus_rebel_gripper_controller)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
set(INCLUDE_DEPENDS
	hardware_interface
	control_msgs
	std_msgs
	builtin_interfaces
	pluginlib
	rclcpp
	rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
	"srv/GripperActuation.srv"
	DEPENDENCIES builtin_interfaces std_msgs
	LIBRARY_NAME ${PROJECT_NAME}
)

# # COMPILE
add_library(
	igus_rebel_gripper_controller
	SHARED
	src/gripper_controller.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")
target_link_libraries(igus_rebel_gripper_controller PUBLIC "${cpp_typesupport_target}")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface gripper_controller.xml)

install(TARGETS igus_rebel_gripper_controller
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_target_dependencies(
	igus_rebel_gripper_controller
	PUBLIC
	${INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${INCLUDE_DEPENDS} rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(igus_rebel_gripper_controller)

ament_package()