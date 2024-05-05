# Movement and autonomous control of Igus Rebel via MoveIt2 C++ APIs in ROS2

This package offers autonomous command and control software using MoveIt2 APIs in ROS2. The code makes use of the `move_group_interface` 
and `planning_scene_interface` to command the robot in a simulated or real environment, allowing interactions with objects and
avoiding obstacles in the environment. The motion planning is collision-free thanks to the collision definitions in the SRDF configuration file.

It also provides simple ROS2 action servers that can be used to command the robot and create high-level behaviors for the robot.

#### Contributor: Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- **Politecnico di Milano**, Academic Year 2023/2024.

Developed at **AIRLAB: Artificial Intelligence and Robotics Laboratory**, [website](https://airlab.deib.polimi.it/)

## Description

This package presents several demos, and the main functions needed to control the robot in a real environment, for simple use-case scenarios.
The functions are collected in a source code file, and the MoveIt2 APIs are used by the demo programs to command and control the robot.

The demos have also action servers, which can be used to command the robot in a more high-level way, by sending goals to the robot
and receiving feedback and results from the robot.

## Demos

There are 3 main demos in this package:
1. Commander Demo: sample functions to demonstrate the use of MoveIt2 APIs and interfaces with the planning scene and environment.
2. Aruco Follower Demo: a demo that uses the camera to detect Aruco markers and follow them with the robot's end effector.
3. Toucher Demo: given a detected Aruco marker, the robot end effector moves to touch the marker orthogonally to the marker plane, in its center.

### 1. Commander Node:

The commander demo is an implementation of the tutorial provided by MoveIt2 for `move_group_interface` and `planning_scene_interface`.
This package contains a node that sends a goal position in cartesian space, which is then reached by the robot from its current position.
Then sends a new goal position specified in joint space, which is reached after the first goal is achieved.

This serves as a demo of the moveit2 functionalities, integrated with the Igus Rebel robot. The demo is interactive and works
with RViz2 visual tools GUI elements, for clear and nice visualization of the target objectives.

This demo works well only in simulation. To launch the demo, type in separate terminal windows:

```bash
$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=simulation

$ ros2 launch igus_rebel_commander commander_demo.launch.py

$ ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

### 2. Aruco Follower demo:

The Aruco Follower demo is a demo that uses the camera to detect Aruco markers and follow them with the robot's end effector'.
It uses movement APIs to command the robot to follow the detected Aruco marker with respect to the camera frame. When the marker is detected
and within the camera field of view, the robot moves to a defined position and orientation with respect to the marker.
The position is defined as a point in which the last joint points towards the marker, and the orientation is defined as the orientation 
of the normal of the marker plane. The robot then moves autonomously to "follow" the marker, in such a way that the last joint is always pointing
towards the marker. In the case the marker is within reach of the robot, the robot will move to the marker to "touch" it with the end effector,
orthogonally to the marker plane.

To start the demo, type in separate terminal windows:

``` bash
$ ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py

$ ros2 launch igus_rebel_commander aruco_follower_demo.launch.py testing:=false
```

Parameters:
- `testing`: 'true' or 'false' --> whether to use the testing mode, which uses a fixed marker pose for testing purposes

### 3. Aruco Toucher demo:

The Aruco Toucher demo is a demo that uses the camera to detect Aruco markers and touch them with the robot's end effector.
It uses movement APIs to command the robot to touch the detected Aruco marker with respect to the camera frame. When the marker is detected
and within the camera field of view, the robot moves to a defined position and orientation with respect to the marker.
The position is defined as a point in which the last joint points towards the marker, and the orientation is defined as the orientation
of the normal of the marker plane. The robot then moves autonomously to "touch" the marker, in such a way that the end effector is always
orthogonal to the marker plane, and the end effector touches the marker in its center.

To start the demo, type in separate terminal windows:

``` bash
$ ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py

$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py

$ ros2 launch igus_rebel_commander aruco_toucher.launch.py
```

## Package Structure

The package is structured as follows:

```yaml
igus_rebel_commander
│   README.md
│   package.xml
│   CMakeLists.txt
├── action
│   ├── MoveManipulator.action # action definition for the robot arm manipulator command action server
│   ├── FollowAruco.action # action definition for the aruco follower demo action server
├── launch
│   ├── aruco_action_server.launch.py # aruco follower demo with action server
│   ├── aruco_follower_demo.launch.py # aruco follower demo (stand-alone)
│   ├── commander_demo.launch.py # commander demo
│   ├── aruco_toucher.launch.py # aruco toucher demo (stand-alone)
│   ├── manipulator_action_server.launch.py # robot arm manipulator command action server
│   ├── test_goal_pose.launch.py # test goal pose for the aruco follower demo
├── include
│   ├── aruco_follower.h # aruco follower demo header file
│   ├── aruco_toucher.hpp # aruco toucher demo header file
│   ├── commander.hpp # commander demo header file
│   ├── manipulator_action_server.hpp # robot arm manipulator command action server header file
│   ├── goal_pose_publisher.h # publisher for end effector goal pose for the aruco follower demo
│   ├── aruco_action_server.hpp # aruco follower demo action server header file
├── src
│   ├── aruco_follower.cpp # aruco follower demo source code
│   ├── aruco_toucher.cpp # aruco toucher demo source code
│   ├── commander.cpp # commander demo source code
│   ├── manipulator_action_server.cpp # robot arm manipulator command action server source code
│   ├── goal_pose_publisher.cpp # publisher for end effector goal pose for the aruco follower demo
│   ├── aruco_action_server.cpp # aruco follower demo action server source code
├── rviz
│   ├── aruco_demo.rviz # rviz configuration file for the aruco follower demo
│   ├── aruco_pose_test.rviz # rviz configuration file for the goal pose test
│   ├── cmd.rviz # rviz configuration file for the commander demo
├── test
│   ├── test_goal_pose_computation.cpp # test goal pose computation for the aruco follower demo
├── aruco_follower_script.sh # bash script to launch the aruco follower demo
├── CMakeLists.txt
├── package.xml
├── README.md
```