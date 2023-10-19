cmake_minimum_required(VERSION 3.8) # higher version of CMake
project(igus_rebel_hw_controller)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)



# -------------- ROS2 declaration
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_interface REQUIRED) # probably not required
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED) #added according to wiki
find_package(rclcpp_lifecycle REQUIRED) # added according to wiki
# find_package(irc_ros_msgs REQUIRED) messages to be ported yet


set(INCLUDE_DEPENDS
	ament_cmake
	rclcpp
	std_msgs
	sensor_msgs
	control_msgs
	rviz2 # added
	controller_manager
	controller_manager_msgs
	hardware_interface
	# irc_ros_msgs # messages definitions not ported yet
	pluginlib # added according to wiki
	rclcpp_lifecycle # added according to wiki

)

# ##################################
# # ament specific configuration ##
# ##################################


# -------------- ROS2 declaration
include_directories(include)

add_library(${PROJECT_NAME} SHARED
	#src/igus_rebel_hw_controller.cpp
	src/fake_controller.cpp
	src/rebel_controller.cpp
	src/cri_messages.cpp
	src/cri_socket.cpp
)

target_include_directories( ${PROJECT_NAME} PRIVATE
  include
)

# Export hardware plugins library: definitions of the library hardware_interface::SystemInterface in an xml file
pluginlib_export_plugin_description_file(hardware_interface igus_rebel_hw_controller.xml)

pluginlib_export_plugin_description_file(hardware_interface igus_rebel_fake_controller.xml)


# -------------- ROS2 declaration
install(TARGETS ${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME} # probably not needed
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)


# -------------- ROS2 declaration
ament_target_dependencies(${PROJECT_NAME} ${INCLUDE_DEPENDS} )

ament_export_dependencies(${INCLUDE_DEPENDS})

ament_export_include_directories( include )


ament_export_libraries(${PROJECT_NAME})


# this must happen before the invocation of ament_package()
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# -------------- ROS2 declaration (to be placed after all the declarations)
ament_package()
