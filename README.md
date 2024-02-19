# Igus Rebel ROS2 autonomous control packages

#### Developed by Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence specialization
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- Politecnico di Milano, Academic Year 2023/2024.

Developed in: **AIRLAB** Artificial Intelligence and Robotics Laboratory, [website](https://airlab.deib.polimi.it/)

## Summary

* Connects to an igus ReBeL over an Ethernet connection
* Controls the Igus ReBel with MoveIt2 interface via ROS2 control structure
* Defines the configuration files for the robot itself
* Supports the addition of grippers and object manipulators to be installed on the robot arm

## Installation

### Software Compatibility

Tested on a Linux computer running with:
- Ubuntu 22.04
- ROS2 Humble & Iron
- X64 processor

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
- `moveit_visual_tools`


### Required Gazebo dependencies

- Ignition Gazebo Fortress (v6)

## Usage

### Connection to the robot

The ros2 hardware interface node expects to reach the robot at the static IP address and port `192.168.1.102:3920` 
via ethernet connection (netmask `255.255.255.0`), using the CRI protocol. In order to connect to the robot, the computer with the
ROS2 node must be connected to the same network as the robot, using an ethernet connection
The computer running the node must use the static IPv4 address `192.168.1.101`

## Packages

### XACRO macros for URDF configurations: `igus_rebel_description`

Load URDF models of the robot depending on the desidered configuration. The parameters to be set are:
- `load_gazebo`: (default `false`) if true, loads the gazebo simulation environment (requires the `ros-gz` package)
- `load_base`: (default `false`) if true, loads the mobile base supporting the robotic arm, as in a mobile manipulator configuration
- `mount`: (default `mount_v1`) choose the URDF model for the end effector and camera mount on the robot arm flange
- `camera`: (default `realsense`) choose the URDF model for the stereo camera mounted on the robot
- `end_effector`: (default `toucher_v1`) choose the URDF model for the end effector mounted on the robot arm flange
- `hardware_protocol` (default `simulation`) choose the protocol to be used for the robot control interface [`cri`, `mock_hardware`, `simulation`]


### MoveIt2 planner and controller `igus_rebel_moveit_config`

Run Rviz2 and MoveIt2 interfaces with `ros2 launch igus_rebel_moveit_config demo.launch.py`, using the same parameters specified above.

Run the MoveIt2 controller only, without GUI with `ros2 launch igus_rebel_moveit_config moveit_controller.launch.py`.

To perform dynamic obstacle avoidance during motion planning, it is required to use the pointcloud data from the stereo camera sensor
and produce an octomap of the environment. This can be done with the `load_octomap` parameter:
- `load_octomap`: (default `false`) if true, loads the octomap data from the stereo camera sensor and uses it for motion planning

The **hardware interfaces** to be used can be set up in the package `igus_rebel_moveit_config`:
- mock hardware interface, for computer test purposes only, commands the robot via position control
- simulation interface, for simulating the robot control interface with Rviz2 and the joint trajectory controller
- CRI interface, for controlling the real robot with the CRI protocol (ethernet connection)

### Gazebo simulation environment for Igus Rebel and multiple sensors: `igus_rebel_gazebo_ignition`:

The launch file `ignition.launch.py` loads the gazebo simulation environment with the robot and the sensors and the visual control
interfaces integrated directly in the simulation environment. The package allows to also start up the default simulation environment for testing purposes.
The package contains also the bridge configuration files for bridgin topics across Gazebo and ROS2.

### ROS2 control and hardware interfaces for the Igus Rebel robot: `igus_rebel_hw_controller`

The package contains the hardware interface for the robot control, using the CRI ethernet protocol. 
The packages contains also the simulation hardware interfaces, provided for controlling the robot using velocity and position control interfaces. 

- The **simulation** hardware interface is used for testing the robot control with Rviz2 and MoveIt2, without the need of the real robot.
It allows for controlling via position and velocity of the joints using a joint trajectory controller. This interfaces provides virtual 
feedback data for the control loop, where the measured positions of the robot correspond exactly to the given commands from the
joint trajectory controller.
- The **CRI** hardware interface is used for controlling the real robot with the CRI protocol, using the ethernet connection. The
hardware interface is used for controlling the robot via position or velocity control. This interface reads the positions of the joints in time
to provide feedback data for the control loop (when using velocity inputs control). The position feedbacks are derivated in time to provide
velocity feedback data.

### Command and control demo programs using MoveIt2 MoveGroup C++ API: `igus_rebel_commander`

The package contains a set of demo programs for controlling the robot using the MoveIt2 MoveGroup C++ API. The programs are used for
testing the robot control and the motion planning capabilities of the robot in a variety of scenarios. The programs are used for showing the
capabilities of the robot while performing autonomous control tasks.

- Aruco Follower demo: given an aruco markers and a stereo camera mounted on the robot, the robot can follow the aruco marker, 
  keeping a position in which the last joint points towards the marker. 
- Button Pressing demo: given a box setup with 3 buttons to press, the robot searches for the buttons in the nearby environment. Once the buttons 
  box is found, the end effector of the robot will press the buttons in a sequence.
- Commander Demo: a set of functions replicating the moveit2 cpp api tutorials using the igus rebel robot.

### Servo control of the robot end effector: `igus_rebel_servo`

Using MoveIt2 Servo library, the package _will_ allow to control the robot end effector using servoing techniques. The package is currently
under development and is not working correctly at the moment.