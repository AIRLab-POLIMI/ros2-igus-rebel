# Igus Rebel ROS2 control packages

Project developed as part of a master's degree thesis project at Politecnico di Milano, Academic Year 2023/2024.

Author: Simone Giampà -- [contact mail](simone.giampa@mail.polimi.it)
Developed in: AIRLAB -- Artificial Intelligence and Robotics Laboratory

## Summary

* Connects to an igus ReBeL over an Ethernet connection
* Controls the Igus ReBel with MoveIt2 interface via ROS2 control structure
* Defines the configuration files for the robot itself
* Supports the addition of grippers and object manipulators to be installed on the robot arm

## Compatibility

Tested on a Linux computer running with:
- Ubuntu 22.04
- ROS2 Humble
- X64 processor

## Installation

Currently, the package can only be used by building it from source.

* Create a colcon workspace (or use an existing one)
* Clone (or download) this repository into the `src` folder of the ros2 workspace
* Build with `colcon build`

## Usage

The ros2 node expects to reach the robot at the IP and port `192.168.3.11:3920` via ethernet connection. using the CRI protocol.

It is recommended to run the ROS2 node with `ros2 launch igus_rebel_moveit_config demo.launch.py`.

This will automatically start a `joint_state_controller/JointStateController` controller and a `joint_trajectory_controller/JointTrajectoryController` controller together with the ROS2 node for the robot hardware interface. After this, the current joint angles are published on the topic `/rebel/joint_states` with message type `sensor_msgs/JointState`.

The hardware interface to be used can be set up in the package `igus_rebel_moveit_config`:
- mock hardware interface, for pc test purposes only, commands the robot via position control
- simulation interface, for simulating the robot control interface with Rviz2 and the joint trajectory controller
- cri interface, for controlling the real robot with the cri protocol (ethernet connection needed)

