#!/bin/bash

# Function to execute a ROS 2 launch file in a new Konsole terminal
run_ros2_launch_in_konsole() {
    konsole --hold -e bash -c "ros2 launch $1"
}

run_ros2_launch_in_konsole "aruco_pose_estimation aruco_pose_estimation.launch.py" & sleep 1 

run_ros2_launch_in_konsole "igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri load_base:=false" &
sleep 3

run_ros2_launch_in_konsole "igus_rebel_commander aruco_follower_demo.launch.py testing:=false load_base:=false" 


# You can add more launch files as needed
exit 0
