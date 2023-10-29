## Igus_Rebel_Description_ROS2

URDF and XACRO macro description configuration files for the Igus Rebel 6DOF robot.

This package includes also a launch file to visualize the Robot as it is. There are 2 implemented versions of the robot:
- `igus_rebel_mod.urdf.xacro`: This is the original version of the robot, with the 6DOF arm and the 2DOF gripper. The original version has been slightly modified to account for the mount of the gripper and corrected to have more precise joints positions. Note that even though this configuration is not exact at a millimiter accuracy, it shows a good robot visualization thanks to the textures provided
- `igus_rebel.urdf.xacro`: this is the new version of the robot released by Commonplace Robotics. It is millimiter accurate, but does not have any texture. This is the version that should be used for any tests with the real robot. It also has simplified meshes, meaning that it is easier to compute the collision checks for the robot.

Further possible updates: create a collision mesh of the robot with the minimum number of triangles, to speed up the collision checks. A possible implementation could be to use the approximate convex decomposition library to create a convex hull of the robot, and then use the convex hull as the collision mesh.