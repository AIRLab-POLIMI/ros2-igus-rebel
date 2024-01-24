# Igus Rebel ROS2 control packages

Project developed as part of a master's degree thesis project at Politecnico di Milano, Academic Year 2023/2024.

Authors: 
- Simone Giampà -- [contact mail](simone.giampa@mail.polimi.it)

Developed in: AIRLAB -- Artificial Intelligence and Robotics Laboratory -- [website](https://airlab.deib.polimi.it/)

## Summary

* Connects to an igus ReBeL over an Ethernet connection
* Controls the Igus ReBel with MoveIt2 interface via ROS2 control structure
* Defines the configuration files for the robot itself
* Supports the addition of grippers and object manipulators to be installed on the robot arm

## Compatibility

Tested on a Linux computer running with:
- Ubuntu 22.04
- ROS2 Humble & Iron
- X64 processor

## Installation

Currently, the package can only be used by building it from source.

* Create a colcon workspace (or use an existing one)
* Clone (or download) this repository into the `src` folder of the ros2 workspace
* Build with `colcon build`

### Required ROS2 dependencies 

- `ros2_control`: install with `sudo apt install ros-<ros2-distro>-ros2-control`
- `ros2_controllers`: install with `sudo apt install ros-<ros2-distro>-ros2-controllers`
- `tf_transformations`: install with `sudo apt install ros-<ros2-distro>-tf-transformations`

### Required MoveIt2 dependencies

It is currently suggested to install MoveIt2 from source, following the instructions at [this link](https://moveit.ros.org/install-moveit2/source/).

Additional MoveIt2 dependencies are required:
- `moveit_visual_tools`

### Required Gazebo dependencies

- `Ignition Gazebo Fortress`

## Usage

The ros2 node expects to reach the robot at the IP and port `192.168.1.102:3920` via ethernet connection (netmask `255.255.255.0`), using the CRI protocol. 
The computer running the node must be connected to the same network as the robot, at static IP address `192.168.1.101`

It is recommended to run the ROS2 node with `ros2 launch igus_rebel_moveit_config demo.launch.py`.

The hardware interface to be used can be set up in the package `igus_rebel_moveit_config`:
- mock hardware interface, for computer test purposes only, commands the robot via position control
- simulation interface, for simulating the robot control interface with Rviz2 and the joint trajectory controller
- CRI interface, for controlling the real robot with the CRI protocol (ethernet connection)

