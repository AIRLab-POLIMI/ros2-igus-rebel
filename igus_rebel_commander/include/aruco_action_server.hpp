// ROS2 includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp>
#include "aruco_action_interfaces/action/follow_aruco.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
#include <set>
#include <cstdint>

class ArucoActionServer : public rclcpp::Node {

public:
    using Aruco = aruco_action_interfaces::action::FollowAruco;
    using GoalHandleAruco = rclcpp_action::ServerGoalHandle<Aruco>;

	// camera frame name
	std::string camera_frame_name;

	// constructor
	ArucoActionServer(const rclcpp::NodeOptions & options);

private:
    rclcpp_action::Server<Aruco>::SharedPtr action_server_;

	/**
	 * @brief callback function: aruco_marker_array is received
	 * @param aruco_marker_array: array of aruco markers in the camera frame
	 */
	void aruco_marker_callback(const aruco_interfaces::msg::ArucoMarkers aruco_marker_array);

	/**
	 * @brief processes one pose: computing the goal pose for pointing / touching the aruco marker
	 * @param aruco_pose: pose of the aruco marker in the camera frame
	 * @param distance cartesian distance to the aruco marker from base_link
	 */
	void processPose(const geometry_msgs::msg::Pose aruco_pose, float distance);

	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::goal_pose_pub");

	// goal pose publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
	// aruco markers subscriber
	rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;

	// map with aruco id and the corresponding pose
	std::map<int64_t, geometry_msgs::msg::Pose> arucos;
	// aruco target id for the action server
	int64_t goal_id;

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

	
	// callback function for handling new goals
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Aruco::Goal> goal);
	// callback function for dealing with cancellation
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle);
	// callback function that accepts a new goal and starts processing it
    void handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle);
	// function that execute the given goal
    void execute(const std::shared_ptr<GoalHandleAruco> goal_handle);
};
