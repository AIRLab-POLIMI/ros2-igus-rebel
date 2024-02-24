# Movement and autonomous control of Igus Rebel via MoveIt2 C++ APIs in ROS2

This package offers autonomous command and control software using MoveIt2 APIs in ROS2. The code makes use of the `move_group_interface` 
and `planning_scene_interface` to command the robot in a simulated or real environment, allowing interactions with objects and
avoiding obstacles in the environment. The motion planning is collision-free thanks to the collision definitions in the SRDF configuration file.

This package presents several demos, and the main functions needed to control the robot in a real environment with real sensor feedback data.
The functions are collected in an API source code file, adn the APIs are used by the demo programs to command and control the robot.

There are 2 main demos in this package:
1. Commander Demo: sample functions to demonstrate the use of MoveIt2 APIs and interfaces.
2. Aruco Follower Demo: a demo that uses the camera to detect Aruco markers and follow them with the robot's last joint.

## 1. Commander Node:

The commander demo is an implementation of the tutorial provided by MoveIt2 for `move_group_interface` and `planning_scene_interface`.
This package contains a node that sends a goal position in cartesian space, which is then reached by the robot from its current position.
Then sends a new goal position specified in joint space, which is reached after the first goal is achieved.

This serves as a demo of the moveit2 functionalities, integrated with the Igus Rebel robot. The demo is interactive and works
with RViz2 visual tools gui elements, for clear and nice visualization of the target objectives.

To launch the demo, type in separate terminal windows:

```bash
$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=simulation

$ ros2 launch igus_rebel_commander commander_demo.launch.py

$ ros2 launch ros2_aruco_pose_estimation aruco_recognition.launch.py
```

## 2. Aruco Follower demo:

The Aruco Follower demo is a demo that uses the camera to detect Aruco markers and follow them with the robot last joint.
It uses movement APIs to command the robot to follow the detected Aruco marker with respect to the camera frame. When the marker is detected
and within the camera field of view, the robot moves to a defined position and orientation with respect to the marker.
The position is defined as a point in which the last joint points towards the marker, and the orientation is defined as the orientation 
of the normal of the marker plane. The robot then moves autonomously to "follow" the marker, in such a way that the last joint is always pointing
towards the marker. In the case the marker is within reach of the robot, the robot will move to the marker to "touch" it with the end effector,
orthogonally to the marker plane.

To start the demo, type in separate terminal windows:

``` bash
$ ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri

$ ros2 launch igus_rebel_commander aruco_follower_demo.launch.py hardware_protocol:=cri testing:=false
```

Parameters:
- `load_base`: 'true' or 'false' --> whether the robot arm is mounted on top of a mobile robot base
- `hardware_protocol`: 'cri' or 'simulation' --> whether to use the real robot or the simulation
- `testing`: 'true' or 'false' --> whether to use the testing mode, which uses a fixed marker pose for testing purposes

