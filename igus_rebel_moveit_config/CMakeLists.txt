cmake_minimum_required(VERSION 3.8)
project(igus_rebel_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ign_ros2_control REQUIRED)

ament_python_install_package(moveit_launch)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()



