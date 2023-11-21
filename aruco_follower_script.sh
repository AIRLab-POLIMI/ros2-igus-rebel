#!/bin/bash

# Function to execute a ROS 2 launch file in a new Konsole terminal
run_ros2_launch_in_konsole() {
    konsole --hold -e bash -c "source ~/robotics/ros2_igus_rebel/install/setup.bash  && source ~/robotics/ros2_aruco/install/setup.bash && ros2 launch $1"
}

run_ros2_launch_in_konsole "realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true publish_tf:=true tf_publish_rate:=1.0" &

sleep 2 &

run_ros2_launch_in_konsole "ros2_aruco aruco_recognition.launch.py" &

sleep 5 &

# Spawn Konsole terminals for each ROS 2 launch file
run_ros2_launch_in_konsole "igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri gripper:=camera load_base:=false" &

sleep 10 &

run_ros2_launch_in_konsole "igus_rebel_commander aruco_follower_demo.launch.py testing:=false load_base:=false gripper:=camera hardware_protocol:=cri" 


# You can add more launch files as needed
exit 0
