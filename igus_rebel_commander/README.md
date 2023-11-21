## Movement of the robot via MoveIt2 with a C++ ROS2 node

### Commander Node:
This package contains a node that sends a goal position in cartesian space, which is then reached by the robot from its current position.
Then sends a new goal position specified in joint space, which is reached after the first goal is achieved.

This serves as a demo of the moveit2 functionalities integrated with the Igus Rebel robot. The demo is interactive and works with rviz visual tools gui.
The rviz file is already configured with the necessary tools.

### Aruco Follower demo:

In order to start the demo, type in separate terminal windows:

insert bash code here:

``` bash
$ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true publish_tf:=true tf_publish_rate:=1.0

$ ros2 launch ros2_aruco aruco_recognition.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py gripper:=camera hardware_protocol:=cri load_base:=false

$ ros2 launch igus_rebel_commander aruco_follower_demo.launch.py gripper:=camera hardware_protocol:=cri testing:=false load_base:=false
```

Parameters:
- `gripper`: 'camera' or 'none' --> whether to use the camera gripper or not
- `hardware_protocol`: 'cri' or 'simulation' --> whether to use the real robot or the simulation
- `testing`: 'true' or 'false' --> if false the aruco pose will be transformed with respect to the camera reference frame; if true the aruco pose will be transformed with respect to the base_link frame
- `load_base`: 'true' or 'false' --> whether to load the robot scout URDF or not
