# MoveIt2 package

**This is very much WIP. While it appears to run fine the current state is insufficiently tested to let it run without being very cautios**

## Usage
Start MoveIt with rviz with the following command:

``` bash
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_moveit_config rebel.launch.py
```

### Namespaces and prefixes
Most of the package supports the parameters, only the controllers.yaml does not. For using th erviz plugin successfully change the topic names to match the namespaces and add the moveit namespace parameter. Save the config and restart rviz without rebuilding.