## Robot Hardware Interface with ROS2 controller

This package contains a hardware interface for the igus rebel robot manipulator. It is based on the `ros2_control` framework and can be used with the `ros2_control` controller manager and controller spawner. The hardware interface is implemented in the file `rebel_controller.cpp`. The hardware interface is based on the ros2_control `joint_trajectory_controller` interface. The controller manager and controller spawner are defined in the `moveit_controller.launch.py`.

### Usage

The hardware interface is linked directly from the `igus_rebel_description_ros2` package. To use the hardware interface, first build the `igus_rebel_description_ros2` package. Then, source the workspace and launch the `moveit_controller.launch.py` file

To launch the CRI hardware interface, use the following command:

```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=cri
```

To simulate the robot hardware interface without commanding the robot, use the following command:

```bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=simulation
```

So this package is a library referenced with `pluginlib` from the robot description package.

### Hardware Interface

It defines the CRI hardware protocol for communicating with the robot. The protocol is used to:
- control the robot via the `ALIVEJOG` command with velocity input
- control the robot via the `Move Joint` command with position input
- read the robot joint positions via the `STATUS` message type
- receive other detailed information about the robot current state and kinematics

