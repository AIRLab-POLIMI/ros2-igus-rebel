cmake_minimum_required(VERSION 3.8) # higher version of CMake
project(igus_rebel_hw_controller LANGUAGES CXX)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# -------------- ROS2 declaration
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED) # added according to wiki
find_package(rclcpp_lifecycle REQUIRED) # added according to wiki
find_package(rcpputils REQUIRED)
find_package(ros2_control REQUIRED)

set(INCLUDE_DEPENDS
	ament_cmake
	rclcpp
	rcpputils
	ros2_control
	std_msgs
	sensor_msgs
	control_msgs
	rviz2
	controller_manager
	controller_manager_msgs
	hardware_interface
	controller_interface
	pluginlib # added according to wiki
	rclcpp_lifecycle # added according to wiki
)

# ##################################
# # ament specific configuration ##
# ##################################

# -------------- ROS2 declaration

add_library(${PROJECT_NAME} SHARED
	src/simulation_controller.cpp
	src/rebel_controller.cpp
	src/cri_messages.cpp
	src/cri_socket.cpp
)

# Export hardware plugins library: definitions of the library hardware_interface::SystemInterface in an xml file
pluginlib_export_plugin_description_file(hardware_interface igus_rebel_hw_controller.xml)

# Export hardware plugins library: definitions of the library hardware_interface::SystemInterface in an xml file
pluginlib_export_plugin_description_file(hardware_interface igus_rebel_simulation_controller.xml)

include_directories(include)

install(DIRECTORY include DESTINATION include)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# -------------- ROS2 declaration
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${INCLUDE_DEPENDS})

ament_export_dependencies(${INCLUDE_DEPENDS})

ament_export_libraries(${PROJECT_NAME})

# this must happen before the invocation of ament_package()
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# -------------- ROS2 declaration (to be placed after all the declarations)
ament_package()
