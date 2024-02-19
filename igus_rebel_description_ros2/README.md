# Igus Rebel Description: URDF and XACRO macros files

This package contains URDF and XACRO macro description configuration files for the Igus Rebel 6DOF robot.

This package includes also a launch file to visualize the Robot as it is. There are 2 implemented versions of the robot:
- `igus_rebel_mod.urdf.xacro`: This is the original version of the robot, with the 6DOF arm and the 2DOF gripper. The original version has been slightly modified to account for the mount of the gripper and corrected to have more precise joints positions. Note that even though this configuration is not exact at a millimiter accuracy, it shows a good robot visualization thanks to the textures provided
- `igus_rebel.urdf.xacro`: this is the new version of the robot released by Commonplace Robotics. It is millimiter accurate, but does not have any texture. This is the version that should be used for any tests with the real robot. It also has simplified meshes, meaning that it is easier to compute the collision checks for the robot.

## Directory structure of the URDF and XACRO files

```
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
│   ├── robot.urdf.xacro # XACRO file for the robot complete configuration, where each parameter defines which macros to include
│   ├── mounts # Folder containing the XACRO macros for the robot
│   │   ├── mount_v1.xacro # First mount version
│   ├── end_effectors # Folder containing the XACRO macros for the robot
│   │   ├── toucher_v1.xacro # First end effector version
│   ├── cameras # Folder containing the XACRO macros for the robot
│   │   ├── oak_d_pro_camera.xacro # oak-d-pro camera simplified model
│   │   ├── virtual_camera_frames.xacro # virtual camera definitions for gazebo
│   ├── igus_rebel # Folder containing the XACRO macros for the robot
│   │   ├── igus_rebel.urdf.xacro # URDF macro of the robot arm with includes for the mount, end effector and camera
│   │   ├── igus_rebel_mod.urdf.xacro # URDF macro of the old and modified version of the robot arm
│   │   ├── igus_rebel.description.xacro # URDF macro with the description of the robot arm model and joints
│   │   ├── igus_rebel.control.xacro # XACRO macro with the control interfaces definition for the robot arm
│   │   ├── igus_rebel.gazebo.xacro # XACRO macro supporting definition for the gazebo simulation sensors and plugins
│   ├── mobile_robot # Folder containing the mobile robot relative exclusion areas
│   │   ├── castle.urdf # URDF for exclusion areas of the sensors and electronics of the mobile robot
├── rviz # Folder containing the RVIZ configuration files for visualization
│   ├── rebel.rviz # RVIZ configuration file for the robot visualization
├── CMakeLists.txt
├── package.xml
```

## Usage

This package is mainly for visualization only and does not contain any control or simulation code. The robot can be visualized in Rviz and Gazebo Ignition.

#### Rviz

To visualize the description of the Igus ReBeL only on Rviz:

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none end_effector:=none camera:=none
```

#### Gazebo Ignition and Rviz

To visualize the description of the Igus ReBeL also in Gazebo Ignition:

```bash
$ ros2 launch igus_rebel_description_ros2 visualize.launch.py load_base:=false mount:=none end_effector:=none camera:=none load_gazebo:=true
```
The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to change the spawned position of the robot in Gazebo Ignition.

The additional arguments `env_gazebo_package` is used to define the package that contain specific Gazebo Ignition world file and bridges. This argument is set to `default` by default, and in this case the world file loaded in Gazebo Ignition will be `default.sdf` contained in this current folder.

The additional arguments `full_world_name` is used to define the world name to load in Gazebo Ignition. If the argument `env_gazebo_package` is set to `default`, this argument will non be used.


When the Igus ReBeL is loaded in Gazebo Ignition, it is possible to control and move it by using the joint position controller gui, located on the top left side of Gazebo Ignition. By clicking the Igus ReBeL inside the simulation, the joints will appear inside the joint position controller gui, and it is possible to change their values. The movement in Gazebo Ignition will be also transferred to Rviz.