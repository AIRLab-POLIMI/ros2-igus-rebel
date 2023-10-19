# igus ReBeL ROS node #

## Summary ##

* Connects to an igus ReBeL over an Ethernet connection
* Is set up to work with ROS controllers to get the current joint angles and set joint velocities

## Compatibility ##

This was only tested on ROS noetic on Ubuntu 20, with kernel 5.4.0. But it is likely that it will also work on older ROS/Ubuntu versions, or other Linux distributions and kernel versions.

## Installation ##

Currently, the package can only be used by building it from source.

* Create a catkin workspace (or use an existing one)
* Clone (or download) this repository into the `src` folder of the catkin workspace
* Build with `catkin_make` or `catkin build`

## Usage ##

The ros node expects to reach the robot at the IP and port `192.168.3.11:3920`.

It is recommended to run the ROS node with the provided launch file, using
`roslaunch igus_rebel rebel.launch`
This will automatically start a `joint_state_controller/JointStateController` controller and a `velocity_controllers/JointGroupVelocityController` controller together with the ROS node for the robot.
After this, the current joint angles are published on the topic `/rebel/joint_states` with message type `sensor_msgs/JointState`. The joint velocity can be commanded by publishing a message on the topic `/rebel/joint_velocity_controller/command` with the message type `std_msgs/Float64MultiArray`.

If a ROS master is running, the ros node can also be started with `rosrun igus_rebel igus_rebel_node`. In this case, controllers have to be started manually.