## Igus ReBeL Gazebo Ignition

It contains the configurations and the launch file to run Gazebo Ignition.

### Config

In this folder there are the bridge configurations and the gui configurations.

#### Bridge

`bridge_description.yaml` contains the bridge between Ros2 and Gazebo Ignition topics that are used when the Igus ReBeL is run in description mode and connected also to Gazebo Ignition.

`bridge_moveit.yaml` contains the bridge between Ros2 and Gazebo Ignition topics that are used when the Igus ReBeL is run in MoveIt2 mode and connected also to Gazebo Ignition.


#### Gazebo GUI

`gazebo_gui_moveit.config` contains the Gazebo Ignition GUI configuration when it is supposed to be run in Moveit2 mode. 

`gazebo_gui_description.config` contains the Gazebo Ignition GUI configuration when it is supposed to be run in Moveit2 mode. The only difference with the configuration of Moveit2 is that now there is the joint position controller gui.



### Launch

In this folder there is the launch file `ignition.launch.py` that is used whenever Gazebo Ignition need to be run.

The additional arguments `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` can be optionally added to the command lines to change the spawned position of the robot in Gazebo Ignition.

The additional arguments `env_gazebo_package` is used to define the package that contain specific Gazebo Ignition world file and bridges. This argument is set to `default` by default, and in this case the world file loaded in Gazebo Ignition will be `default.sdf` contained in this current folder.

The additional arguments `full_world_name` is used to define the world name to load in Gazebo Ignition. If the argument `env_gazebo_package` is set to `default`, this argument will non be used.


Finally, the Gazebo Ignition main node, bridge and spawn entity are executed.