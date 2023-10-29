## Movement of the robot via MoveIt2 with a C++ ROS2 node

This package contains a node that sends a goal position in cartesian space, which is then reached by the robot from its current position.
Then sends a new goal position specified in joint space, which is reached after the first goal is achieved.

This serves as a demo of the moveit2 functionalities integrated with the Igus Rebel robot. The demo is interactive and works with rviz visual tools gui.
The rviz file is already configured with the necessary tools.

In order to start the demo, type in 2 separate terminal windows:

insert bash code here:

``` bash
$ ros2 launch igus_rebel_moveit_config moveit_controller.launch.py

$ ros2 launch igus_rebel_commander demo.launch.py
```

or launch everything in a single terminal window:

```console
ros2 launch igus_rebel_commander demo_complete.launch.py
```