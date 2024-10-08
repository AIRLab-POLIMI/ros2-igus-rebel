# Igus ReBeL Gazebo Ignition simulation package

It contains the configurations and the launch file to run Gazebo Ignition.

## Config

In this folder, there are the bridge configurations and the gui configurations.

### Bridge

- `bridge_description.yaml` contains the bridge between Ros2 and Gazebo Ignition topics that are used when the Igus ReBeL is 
run in description mode and connected also to Gazebo Ignition.

- `bridge_moveit.yaml` contains the bridge between Ros2 and Gazebo Ignition topics that are used when the Igus ReBeL is run 
with MoveIt2 controller and connected also to Gazebo Ignition.


### Gazebo GUI

`gazebo_gui_moveit.config` contains the Gazebo Ignition GUI configuration when it is supposed to be run in Moveit2 mode. 

`gazebo_gui_description.config` contains the Gazebo Ignition GUI configuration when it is supposed to be run in Moveit2 mode. 
The only difference with the configuration of Moveit2 is that now there is the joint position controller GUI.

## Launch

In this folder, there is the launch file `ignition.launch.py` which is used whenever Gazebo Ignition is needed to be run.

The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to the command lines to change 
the spawned position of the robot in Gazebo Ignition.

The additional argument `env_gazebo_package` is used to define the package that contains specific Gazebo Ignition world files and bridges.
This argument is set to `default` by default, and in this case, the world file loaded in Gazebo Ignition will be `default.sdf` contained 
in this current folder.

The additional argument `full_world_name` is used to define the world name to load in Gazebo Ignition. 
If the argument `env_gazebo_package` is set to `default`, this argument will noy be used.

Finally, the Gazebo Ignition main node, bridge and spawn entity are executed.