# Igus Rebel Description: URDF and XACRO macros files

This package contains URDF and XACRO macro description configuration files for the Igus Rebel 6DOF robot.
The launch file allows the visualization of the robot in Rviz and Gazebo Ignition, without any control or simulation.
This package is used as a dependency of all packages that require the Igus Rebel robot description, for planning and control purposes.

#### Contributor: Simone Giampà

Project developed as part of a master's degree thesis project.

- Master's Thesis in Computer Science Engineering - Robotics and Artificial Intelligence
- Author: __Simone Giampà__ -- [contact email](simone.giampa@mail.polimi.it)
- **Politecnico di Milano**, Academic Year 2023/2024.

Developed at **AIRLAB: Artificial Intelligence and Robotics Laboratory**, [website](https://airlab.deib.polimi.it/)

## Description

This package includes also a launch file to visualize the Robot as it is. There are 2 implemented versions of the robot:
- `igus_rebel_mod.urdf.xacro`: This is the original version of the robot, with the 6DOF arm.
  The original version has been slightly modified to account for the 3D mount of the gripper and corrected to have more precise joint positions.
  Note that even though this configuration is not exact at a millimeter accuracy, it shows a good robot visualization thanks to the textures provided
- `igus_rebel.urdf.xacro`: this is the new version of the robot released by Commonplace Robotics. It is sub-millimeter accurate 
  but does not have any texture. This is the version that should be used for any tests with the real robot.
  It also has simplified meshes, meaning that it is faster to compute the collision checks for the robot.

## Package structure 

```yaml
igus_rebel_description_ros2
├── launch
│   ├── visualize.launch.py # Launch file to visualize the robot in Rviz and Gazebo Ignition
├── meshes
│   ├── igus_rebel # Folder containing the meshes for the robot
│   ├── igus_rebel_mod # Folder containing the meshes for the old robot
│   ├── cameras # Folder containing the meshes for the cameras
│   ├── end_effectors # Folder containing the meshes for the end effectors
│   ├── mounts # Folder containing the meshes for the mounts
├── urdf
│   ├── robot.urdf.xacro # Main XACRO file for the robot complete configuration, where each parameter defines which macros to include
│   ├── mounts # Folder containing the XACRO macros for the robot
│   │   ├── mount_v1.xacro # First mount version
│   │   ├── mount_v2.xacro # Second mount version
│   ├── end_effectors # Folder containing the XACRO macros for the robot
│   │   ├── toucher_v1.xacro # First end effector version
│   │   ├── soft_gripper.description.xacro # Second end effector version: soft gripper
│   │   ├── soft_gripper.control.xacro # ROS2 control macro for the soft gripper
│   ├── cameras # Folder containing the XACRO macros for the robot
│   │   ├── cameras.urdf.xacro # Camera definitions for the robot: Realsense and OAK-D stereo cameras
│   │   ├── virtual_camera_frames.xacro # virtual camera definitions for gazebo
│   ├── igus_rebel # Folder containing the XACRO macros for the robot
│   │   ├── igus_rebel.urdf.xacro # URDF macro of the robot arm with includes for the mount, end effector and camera
│   │   ├── igus_rebel.description.xacro # URDF macro with the description of the robot arm model and joints
│   │   ├── igus_rebel.control.xacro # XACRO macro with the control interfaces definition for the robot arm
│   │   ├── igus_rebel.gazebo.xacro # XACRO macro supporting definition for the gazebo simulation sensors and plugins
│   │   ├── igus_rebel_mod.urdf.xacro # URDF macro of the old and modified version of the robot arm with other additional components
│   │   ├── igus_rebel_mod.description.xacro # URDF description of the old and modified version of the robot arm
│   ├── mobile_robot # Folder containing the mobile robot relative exclusion areas
│   │   ├── castle.urdf # URDF for exclusion areas of the sensors and electronics of the mobile robot
├── rviz # Folder containing the RVIZ configuration files for visualization
│   ├── rebel.rviz # RVIZ configuration file for the robot visualization
├── CMakeLists.txt
├── package.xml
```

## Usage

This package is mainly for visualization only and does not contain any control or simulation code. 
The robot can be visualized in Rviz and Gazebo Ignition.

### Rviz

To visualize the description of the Igus ReBeL only on Rviz, set the `load_gazebo` argument to `false` (default value):

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_gazebo:=false
```

### Gazebo Ignition and Rviz

To visualize the description of the Igus ReBeL also in Gazebo Ignition:

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_gazebo:=true
```
The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to change the spawned position
of the robot in Gazebo Ignition.

The additional argument `env_gazebo_package` is used to define the package that contains specific Gazebo Ignition world files and bridges.
This argument is set to `default` by default, and in this case, the world file loaded in Gazebo Ignition will be `default.sdf` 
contained in this current folder.

The additional argument `full_world_name` is used to define the world name to load in Gazebo Ignition. 
If the argument `env_gazebo_package` is set to `default`, this argument will not be used.

When the Igus ReBeL is loaded in Gazebo Ignition, it is possible to control and move it by using the joint position controller GUI, 
located on the top left side of Gazebo Ignition. By clicking the Igus ReBeL inside the simulation, the joints will appear inside the
joint position controller GUI, and it is possible to change their values. The movement in Gazebo Ignition will be also transferred to Rviz.