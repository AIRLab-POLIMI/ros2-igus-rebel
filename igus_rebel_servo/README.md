## Realtime servoing with MoveIt2 and ROS2-control

### Code repository and tutorials

Code inspired by [this tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)

Pose tracking code demo found in the official repo: [moveit_servo/demos/demo_pose.cpp](https://github.com/ros-planning/moveit2/blob/iron/moveit_ros/moveit_servo/demos/cpp_interface/demo_pose.cpp)


### Current status of the development

Current implementation: the servo library doesn't work as expected. The robot moves slightly only for 1 or 2 seconds, then the velocity commands are scaled down so much that they become negligible. 

What I tested so far:
- velocity controller in open loop --> divergent behavior
- velocity controller in closed loop --> works but velocity is scaled down too much after a few seconds
- position controller --> same as velocoty controller in closed loop
- pid gains --> affect only the stability and oscillations in the motion
- servo parameters --> only the butterworth pass down filter affects the motion, since it changes how many times the warning (velocity scaling) is triggered. Other parameters do not seem to have a relevant effect in the movement.
- goal pose --> moving it close or far from the robot changes the robot movement but the overall behavior stays the same
- servo_params.publish_period --> changing it to 0.02 makes the robot not move, and the commands are sent only after the servo node is shutdown for some absurd reason.

No useful information found online, servo becomes stale at the moment (2023-11-14).

