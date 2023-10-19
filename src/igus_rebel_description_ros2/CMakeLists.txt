cmake_minimum_required(VERSION 3.8) # 3.8 is required for C++17

project(igus_rebel_description_ros2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# -------------- ROS2 cmake standard declarations
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------- ROS1 declaration
#find_package(catkin REQUIRED)
#catkin_package()
find_package(ament_cmake REQUIRED)

# -------------- ROS1 declaration
#if(CATKIN_ENABLE_TESTING)
  #find_package(roslaunch REQUIRED)
  #roslaunch_add_file_check(launch)
#endif()
# -------------- ROS2 declaration
if (BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# -------------- ROS1 declaration
#install(DIRECTORY images meshes launch rviz urdf
        #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# -------------- ROS2 declaration
install(DIRECTORY meshes launch urdf rviz
		DESTINATION share/${PROJECT_NAME}
)

# -------------- ROS2 declaration
ament_package()

