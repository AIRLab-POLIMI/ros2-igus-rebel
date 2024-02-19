
// ROS2 includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// tf2 includes
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// C++ libraries includes
#include <cmath>
#include <thread>

class GoalPosePublisher : public rclcpp::Node {

public:
	// camera frame name
	std::string camera_frame_name;

	// constructor
	GoalPosePublisher();

private:
	/**
	 * @brief callback function: aruco_pose_array is received
	 * @param aruco_pose_array: array of poses of the aruco markers in the camera frame
	 */
	void aruco_pose_callback(const geometry_msgs::msg::PoseArray aruco_pose_array);

	/**
	 * @brief processes one pose: computing the goal pose for pointing / touching the aruco marker
	 * @param aruco_pose: pose of the aruco marker in the camera frame
	 * @param distance cartesian distance to the aruco marker from base_link
	 */
	void processPose(const geometry_msgs::msg::Pose aruco_pose, float distance);

	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::goal_pose_pub");

	// goal pose publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
	// aruco poses array subscriber
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_pose_sub_;

	std::shared_ptr<tf2_ros::TransformListener> tf2; // tf listener
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;	 // tf buffer

	// The target frame (desired link's frame)
	const std::string target_frame = "igus_rebel_base_link"; // The target frame (within the robot arm frame)
	std::string source_frame;								 // The source frame (aruco's base frame)
	const std::string link_ref_frame = "link_2";			 // the reference link from which the goal pose alignment is computed

	// position of the link_ref frame with respect to base link
	float link_ref_z;

	// parameters for the goal pose computation
	const float goal_radius = 0.8;		// radius in meters of a reachable goal
	const float reachable_radius = 0.7; // radius in meters of the position that the robot will reach
};
