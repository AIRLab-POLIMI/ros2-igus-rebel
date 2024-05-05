# MoveIt2 planning and control for Igus Rebel

MoveIt2 configuration files for controlling the robot autonomously, and planning join trajectories in a collision-free environment.
It contains the definitions for everything needed to control the robot using MoveIt2 libraries and interfaces.

#### Contributor: Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- **Politecnico di Milano**, Academic Year 2023/2024.

Developed at **AIRLAB: Artificial Intelligence and Robotics Laboratory**, [website](https://airlab.deib.polimi.it/)

## Package structure

This package contains the MoveIt2 configuration files for the Igus Rebel robot arm. The package is structured as follows:

```yaml
igus_rebel_moveit_config
├── config # Configuration files for MoveIt2, explained below
│   ├── igus_rebel.srdf.xacro # SRDF collisions mapping with XACRO macros for all possible robot configurations
│   ├── moveit_controllers.yaml
│   ├── ompl_planning.yaml
│   ├── stomp_planning.yaml
│   ├── pilz_industrial_planner.yaml
│   ├── pilz_cartesian_limits.yaml
│   ├── multiple_planning_pipelines.yaml
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── ros2_controllers.yaml
│   ├── initial_positions.yaml
├── launch
│   ├── demo.launch.py # main launch file for MoveIt2 control with RVIZ2 visualization
│   ├── moveit_controller.launch.py # main launch file for MoveIt2 control without RVIZ2 visualization
├── moveit_launch
│   ├── moveit_loader.py # Python script to load the MoveIt2 configuration files, the robot descriptions and the collision definitions
├── rviz
│   ├── moveit.rviz
├── CMakeLists.txt
├── package.xml
├── README.md
```

The script `moveit_loader.py` is used to load the MoveIt2 configuration files, the robot descriptions, and the collision definitions.
It provides useful functions to load the robot model, the planning scene, and the MoveIt2 control manager, which can be accessed
from other packages that need to access the configuration files. Refer to the script for further details.
The functions can be accessed by including the script in the Python code and calling the functions to load the configuration files.

```python
from moveit_launch import moveit_loader
```

## Configuration

The **config** folder contains several configuration files for:
- `moveit_controllers.yaml`: MoveIt2 control manager with follow joint trajectory controller
- `ompl_planning.yaml`: OMPL planner configuration
- `stomp_planning.yaml`: STOMP planning configuration
- `pilz_industrial_planner.yaml`: Pilz industrial motion planner configuration
- `pilz_cartesian_limits.yaml`: Pilz industrial motion planner cartesian limits
- `multiple_planning_pipelines.yaml`: Multiple planning pipelines: OMPL, STOMP, Pilz Industrial
- `kinematics.yaml`: Inverse Kinematic solvers configuration
- `joint_limits.yaml`: Velocity and acceleration constraints
- `ros2_controllers.yaml`: ROS2 control for the robot arm with joint trajectory controller and Moveit controller manager
- `igus_rebel.srdf.xacro`: SRDF collisions mapping with XACRO macros for all possible robot configurations
- `initial_positions.yaml`: Joint positions for standard positions
- `sensors_3d.yaml`: 3D sensors configuration for the Octomap server

This package contains also the file `.setup_assistant` that is used to configure the robot for MoveIt2.

## Usage

The robot can be used in simulation or with the real robot. The robot can be controlled with MoveIt2 APIs and interfaces,
and the robot can be visualized in Rviz and Gazebo Ignition. Rviz2 is used for visualizing the robot, the planning scene, and the
collision objects created with the Octomap server.

Gazebo Ignition is used for visualizing the robot in a simulated environment, and to test the robot with simulated sensor feedback data.

### Visual control with Rviz: Parameters required

To start MoveIt2 controlling the Igus ReBeL on Rviz:

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py
```

To use the robot properly, it is necessary to specify the hardware protocol used. The available options for the parameter `hardware_protocol` are:
- `simulation`: to use the simulation robot model with velocity control
- `cri`: to use the real robot with the ethernet interface (CRI) protocol
- `ignition`: to use the Gazebo Ignition simulation
- `mock_hardware`: to use the default simulation model for position control only

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py hardware_protocol:=<protocol>
```

To load the Octomap server using pointcloud data from the camera, the parameter `load_octomap` can be set to `true`. Default is `false`:

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_octomap:=true
```

To use the robot arm when mounted on the AgileX Scout mobile robot base, the parameter `load_base` must be set to `true`. 
This is mandatory to use the robot arm with the mobile robot base, to prevent collisions between the robot arm and the base
when planning trajectories.

``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_base:=true
```

For further details on the rest of the parameters, please refer to the launch file `demo.launch.py`, or the package `igus_rebel_description_ros2`
for the parameters related to the URDF description of the robot.

## Usage with Gazebo Ignition and Rviz

To start MoveIt2 controlling the Igus ReBeL on Rviz and transfer the control also on the Gazebo Ignition spawned arm:
``` bash
$ ros2 launch igus_rebel_moveit_config demo.launch.py load_gazebo:=true hardware_protocol:=ignition
```

The additional parameters `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to change the spawned position
of the robot in Gazebo Ignition.

The additional parameter `env_gazebo_package` is used to define the package that contains specific Gazebo Ignition world files and bridges.
This argument is set to `default` by default, and in this case, the world file loaded in Gazebo Ignition will be `default.sdf` 
contained in this current folder.

The additional parameter `full_world_name` is used to define the world name to load in Gazebo Ignition. 
If the argument `env_gazebo_package` is set to `default`, this argument will not be used.

