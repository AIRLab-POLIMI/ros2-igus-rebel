
// ROS
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <cmath>


class GoalPosePublisher : public rclcpp::Node {

public:
	std::string camera_frame_name;
	GoalPosePublisher();

private:
	// callback when aruco_pose_array is received
	void aruco_pose_callback(const geometry_msgs::msg::PoseArray aruco_pose_array);
	// processes one pose from the aruco_pose_array, requires cartesian distance to the aruco marker from base_link
	void processPoseArray(const geometry_msgs::msg::Pose aruco_pose, float distance);

	void test_pub_thread(void );

	// goal pose publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
	// aruco poses array subscriber
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_pose_sub_;


};
