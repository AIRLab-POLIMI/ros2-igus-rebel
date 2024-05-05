# Robot Hardware Interfaces with ROS2 control and CRI protocol

This package provides a hardware interface for the Igus Rebel robot manipulator.
It is based on the `ros2_control` framework and can be used with the Moveit controller manager, which orchestrates the control
of the robot manipulator using different controllers.

This package provides a working and tested hardware interface using the CRI protocol, via ethernet interface, 
for the Igus Rebel robot manipulator, which can be used to control the robot effectively and easily, without resorting to the
CAN bus interface (which requires a proprietary connection cable).

#### Contributor: Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- **Politecnico di Milano**, Academic Year 2023/2024.

Developed at **AIRLAB: Artificial Intelligence and Robotics Laboratory**, [website](https://airlab.deib.polimi.it/)

## Description

This package contains a hardware interface for the Igus Rebel robot manipulator. 
It is based on the `ros2_control` framework and can be used with the `ros2_control` controller manager and controller spawner. 
The hardware interface is implemented in the file `rebel_controller.cpp`. 
The hardware interface is based on the ros2_control `joint_trajectory_controller` interface. 
The controller manager and controller spawner are defined in the `moveit_controller.launch.py`.

The hardware interface is used to control the robot manipulator using the CRI protocol, via ethernet interface.
It supports the latest version of the CRI protocol and the latest version of the internal robot firmware and motor controllers.
The implementation is stable but may be subject to changes in the future, depending on the development
and calibration of the robot firmware and motor controllers.

## Usage

The hardware interface is linked directly from the `igus_rebel_description_ros2` package. To use the hardware interface,
first build the `igus_rebel_description_ros2` package. Then, source the workspace and launch the `moveit_controller.launch.py` file

To launch the CRI hardware interface, using an ethernet connection, use the following command:

```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=cri
```

To simulate the robot hardware interface with velocity or position control, use the following command:

```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=simulation
```
This will launch the robot with a customized simulation interface, which supports both velocity and position control.
The default implementation of the simulation interface supports only position control.

To simulate the robot with Gazebo Ignition, use the following command:

```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=ignition
```

## CRI Hardware Interface

It defines the CRI hardware protocol for communicating with the robot. The protocol is used to:
- control the robot via the `ALIVEJOG` command with velocity input
- control the robot via the `Move Joint` command with position input
- read the robot joint positions via the `STATUS` message type, along with other feedback information
- receive other detailed information about the robot current state and kinematics limits
- receive error messages when the kinematic limits are reached or when the emergency button is pressed, or when there is no sufficient current
  to power the robot motors.

## Package Structure

The package is structured as follows:

```yaml
igus_rebel_hw_controller
│
├── include
│   ├── cri_keywords.h # The keywords for the CRI protocol
│   ├── cri_socket.h # The socket interface for CRI protocol
│   ├── cri_messages.h # The message types for CRI protocol
│   ├── rebel_controller.h # The hardware interface for CRI protocol
│   ├── simulation_controller.h # The simulation controller interface
├── src
│   ├── rebel_controller.cpp # The hardware interface implementation for CRI protocol
│   ├── cri_socket.cpp # The socket implementation for CRI protocol
│   ├── cri_messages.cpp # The message types for CRI protocol
│   ├── simulation_controller.cpp # The simulation controller implementation
├── CMakeLists.txt
├── package.xml
├── igus_rebel_hw_controller.xml # The hardware interface plugin description for CRI protocol
├── igus_rebel_simulation_controller.xml # The hardware interface plugin description for simulation environment
├── README.md
```