# Igus Rebel ROS2 Autonomous Control with MoveIt2 and Gazebo Ignition

This repository contains the ROS2 packages for controlling the Igus Rebel robot arm with MoveIt2 and Gazebo Ignition Fortress.
The packages contain the ros2 control hardware interfaces for the robot arm control, the MoveIt2 interfaces for the motion planners,
and the Gazebo simulation environment for testing the robot control and motion planning capabilities.
This repository provides a set of quick and simple demos to command the robot via MoveIt2 MoveGroup C++ APIs, and to control the robot
in a completely autonomous way, with the planned joint trajectories and the end effector control.

This repository provides a working and tested environment for controlling the robot via **CRI protocol**, using the **ethernet connection**.
While the official repository for the robot control provides a working implementation for the ROS2 control interface for CAN protocol,
their implementation is not yet working correctly for the CRI (ethernet) protocol. This repository, instead, provides support for the CRI protocol
(ethernet) for controlling the robot arm via position and velocity control. The code is tested and working correctly for the real robot control,
using the latest version of the motor control firmware.

#### Contributor: Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- **Politecnico di Milano**, Academic Year 2023/2024.

Developed at **AIRLAB: Artificial Intelligence and Robotics Laboratory**, [website](https://airlab.deib.polimi.it/)


## Installation

### Software Compatibility

Tested on a Linux computer running with:
- Ubuntu 22.04
- ROS2 Humble & Iron
- MoveIt2 Main branch

Currently, the package can only be used by building it from source.

* Create a colcon workspace (or use an existing one)
* Clone (or download) this repository into the `src` folder of the ros2 workspace
* Build with `colcon build --symlink-install`

### Required ROS2 dependencies 

- `ros2_control`: install with `sudo apt install ros-<ros2-distro>-ros2-control`
- `ros2_controllers`: install with `sudo apt install ros-<ros2-distro>-ros2-controllers`
- `tf_transformations`: install with `sudo apt install ros-<ros2-distro>-tf-transformations`
- `ros-gz`: install with `sudo apt install ros-<ros2-distro>-ros-gz`
- `moveit2`: install from source, following the instructions at [this link](https://moveit.ros.org/install-moveit2/source/)
- `moveit_visual_tools`: install from source from the MoveIt2 repository


### Required Gazebo dependencies

- `ros-gzfortress`: Ignition Gazebo Fortress (v6) simulation environment

## Usage with CRI protocol

This repository provides the necessary packages for controlling the Igus Rebel robot arm using the CRI protocol, via ethernet connection.
The robot control is done via position and velocity control, using the joint trajectory controller provided by the `ros2_control` package.
The current configuration uses velocity control but can be easily extended to use position control. The end effector hardware interface
is designed for the specific end effector mounted on the robot arm flange, but it can be used as a starting point for implementing
other interfaces for different end effectors.

### Connection to the robot

The ros2 hardware interface node expects to reach the robot at the static IP address and port `192.168.1.102:3920` 
via ethernet connection (netmask `255.255.255.0`), using the CRI protocol. To connect to the robot, the computer with the
ROS2 node must be connected to the same network as the robot, using an ethernet connection. To change the IP address used, it can be
changed easily in the `igus_rebel_hw_controller` package, in the `rebel_controller.cpp` file.
The computer running the ROS2 interface node must use the static IPv4 address `192.168.1.101`.

## Main launch file

Load URDF models of the robot depending on the desired configuration. To start up the robot, use the following command:
```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py
```

To correctly use the ROS2 control interface, the parameters to be set are:
- `load_gazebo`: (default `false`) if true, loads the gazebo simulation environment (requires the `ros-gz` package)
- `load_base`: (default `true`) if true, loads the mobile base supporting the robotic arm, as in a mobile manipulator configuration
- `mount`: (default `mount_v2`) choose the URDF model for the end effector and camera mount on the robot arm flange
- `camera`: (default `realsense`) choose the URDF model for the stereo camera mounted on the robot
- `end_effector`: (default `soft_gripper`) choose the URDF model for the end effector mounted on the robot arm flange
- `hardware_protocol` (default `simulation`) choose the protocol to be used for the robot control interface [`cri`, `mock_hardware`, `simulation`]
- `load_octomap`: (default `false`) if true, loads the octomap server and produces a volumetric occupancy map from the stereo camera 
  sensor and uses it for motion planning around static obstacles.

Further instructions for the usage are provided in the `igus_rebel_moveit_config` package, in the README file.

## Packages

This repository contains the following ROS2 packages:
1. `igus_rebel_description`: XACRO macros for URDF configurations
2. `igus_rebel_moveit_config`: MoveIt2 planner and controller
3. `igus_rebel_gazebo_ignition`: Gazebo simulation environment for Igus Rebel and multiple sensors
4. `igus_rebel_hw_controller`: ROS2 control and hardware interfaces for the Igus Rebel robot
5. `igus_rebel_commander`: Command and control demo programs using MoveIt2 MoveGroup C++ API
6. `igus_rebel_gripper_controller`: Soft Gripper Pneumatic Pump control interface with ROS2
7. `igus_rebel_servo`: Servo control of the robot end effector

### 1. `igus_rebel_description`: XACRO macros for URDF configurations

The package contains the XACRO macros for the URDF configurations of the Igus Rebel robot arm, the mobile base, the end effector, 
the stereo camera and the flange 3D printed mount. The package contains also the URDF description and ros2-control specification files
for the robot arm, with the joint limits and optimized collision meshes. The package contains also the URDF description for the
"castle", which represents the collision boundaries that the robot arm must avoid when moving on top of the mobile base.
The mounts are the URDF exported models from the CAD files of the 3D printed parts, used for mounting the stereo camera and the end effector
on the robot arm flange.

### 2. `igus_rebel_moveit_config` MoveIt2 planner and controller

Run Rviz2 and MoveIt2 interfaces with `ros2 launch igus_rebel_moveit_config demo.launch.py`, using the same parameters specified above.

Run the MoveIt2 controller only, without RVIZ2 GUI with `ros2 launch igus_rebel_moveit_config moveit_controller.launch.py`.

To perform static obstacle avoidance during motion planning, it is required to use the pointcloud data from the stereo camera sensor
and produce an Octomap of the environment. This can be done with the `load_octomap` parameter:
- `load_octomap`: (default `false`) if true, loads the octomap server and produces a volumetric occupancy map from the stereo camera 
  sensor and uses it for motion planning around static obstacles.

The **hardware interfaces** to be used is set up in the package `igus_rebel_moveit_config`:
- mock hardware interface, for computer test purposes only, commands the robot via position control
- simulation interface, for simulating the robot control interface with Rviz2 and the joint trajectory controller
- CRI interface, for controlling the real robot with the CRI protocol (ethernet connection)

### 3. `igus_rebel_gazebo_ignition`: Gazebo simulation environment for Igus Rebel and multiple sensors

The launch file `ignition.launch.py` loads the gazebo simulation environment with the robot and the sensors and the visual control
interfaces integrated directly into the simulation environment. The package allows also to start-up the default simulation environment 
for testing purposes. The package contains also the bridge configuration files for bridging the topics across Gazebo and ROS2.

### 4. `igus_rebel_hw_controller`: ROS2 control and hardware interfaces for the Igus Rebel robot

The package contains the hardware interface for the robot control, using the CRI ethernet protocol. 
The package contains also the simulation hardware interfaces, provided for controlling the robot using velocity and position control interfaces. 

- The **simulation** hardware interface is used for testing the robot control with Rviz2 and MoveIt2, without the need of using the real robot.
It allows for controlling via position and velocity of the joints using a joint trajectory controller. This interface provides virtual 
feedback data for the control loop, where the measured positions of the robot correspond exactly to the given commands from the
joint trajectory controller.
- The **CRI** hardware interface is used for controlling the real robot with the CRI protocol, using the ethernet connection. The
hardware interface is used for controlling the robot via position or velocity control. This interface reads the positions of the joints in time
to provide feedback data for the control loop (when using velocity inputs control). The position feedbacks are derivated in time to provide
velocity feedback data.

### 5. `igus_rebel_commander`: Command and control demo programs using MoveIt2 MoveGroup C++ API: 

The package contains a set of demo programs for controlling the robot using the MoveIt2 MoveGroup C++ API. The programs are used for
testing the robot control and the motion planning capabilities of the robot in a variety of scenarios. The programs are used to show the
capabilities of the robot while performing autonomous control tasks.

- **Aruco Follower demo**: given an aruco marker and a stereo camera mounted on the robot, the robot can follow the aruco marker, 
  keeping the end effector in a position in which the last joint points toward the marker. 
- **Button Pressing demo**: given a box setup with 3 buttons to press, the robot searches for the buttons in the nearby environment. Once the buttons 
  box is found, the end effector of the robot will press the buttons in a sequence.
- **Commander Demo**: a set of functions replicating the MoveIt2 C++ API tutorials using the igus rebel robot.

### 6. `igus_rebel_gripper_controller`: Soft Gripper Pneumatic Pump control interface with ROS2

This package contains the control interface for the pneumatic pump actuator for the soft gripper end effector, 
mounted on the robot arm flange. The gripper is controlled via an arduino, which sends the commands to the pneumatic pump
to open and close the gripper. The electronic connections are made on a soldered PCB board, which is connected to the Arduino
and the pneumatic pump via relays. The relays act as switches for the pneumatic pump, which is powered by a 24V power supply.
The relays are controlled by the Arduino digital pins, which receive the commands from the ROS2 control interface via serial communication.
The package contains the ROS2 control interface for the gripper control, which acts as a bridge between the ROS2 control interface and the Arduino.

### 7. `igus_rebel_servo` Servo control of the robot end effector: 

Using MoveIt2 Servo library, the package _will_ allow the control of the robot end effector using servoing techniques. 
The package is currently under development and is not working correctly at the moment.
Currently, this package will compile only with ROS2 Iron. If using ROS2 Humble, the package will not compile correctly, so it is advised to
delete the package from the workspace or selectively build the other packages with `colcon build --packages-select <package_name>`.