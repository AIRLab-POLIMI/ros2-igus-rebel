# MoveIt2 package

MoveIt2 configuration files for controlling the robot graphically with Rviz2. It contains the definitions for everything that is needed to control the robot.

It contains definitions for:
- OMPL planner and MoveitSimpleController Manager connections
- Kinematic constraints
- Velocity and acceleration constraints
- ROS2 control with joint trajectory controller
- SRDF configurations and XACRO macros for gripper controls (not yet implemented)
- Joint positions for standard positions

## Usage
Start MoveIt with rviz with the following command:

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py
```

