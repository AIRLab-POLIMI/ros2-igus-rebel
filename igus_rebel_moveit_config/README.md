# MoveIt2 package

MoveIt2 configuration files for controlling the robot graphically with Rviz2. It contains the definitions for everything that is needed to control the robot.


It contains definitions for:
- OMPL planner and MoveitSimpleController Manager connections
- Kinematic constraints
- Velocity and acceleration constraints
- ROS2 control with joint trajectory controller
- SRDF configurations and XACRO macros for controls and collisions
- Joint positions for standard positions

## Usage

### Rviz

To start MoveIt2 controlling the Igus ReBeL on Rviz:

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none
```

### Gazebo Ignition and Rviz

To start MoveIt2 controlling the Igus ReBeL on Rviz and transfer the control also on the Gazebo Ignition spawned arm:
``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=false mount:=none camera:=none end_effector:=none load_gazebo:=true hardware_protocol:=ignition
```

The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to change the spawned position of the robot in Gazebo Ignition.

The additional arguments `env_gazebo_package` is used to define the package that contain specific Gazebo Ignition world file and bridges. This argument is set to `default` by default, and in this case the world file loaded in Gazebo Ignition will be `default.sdf` contained in this current folder.

The additional arguments `full_world_name` is used to define the world name to load in Gazebo Ignition. If the argument `env_gazebo_package` is set to `default`, this argument will non be used.

