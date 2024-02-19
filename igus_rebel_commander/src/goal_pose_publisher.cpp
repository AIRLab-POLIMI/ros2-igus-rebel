// Objective of this node:
// 1. reads pose array topic /aruco_poses
// 2. reads first pose in array of detected arucos
// 3. compute the line coordinates from link_1 to aruco pose and estimate distance
// 4. compute the coordinate pose that the end effector will reach as a goal
// the goal pose should be in a position that the end effector can reach
// reduce the distance from link1 to aruco pose enough so that the distance from link_1 to goal pose is less than a meter
// if the aruco pose is reachable, then the goal pose is the aruco pose
// 5. publish the goal pose to /goal_pose
// 6. program the servoing node to reach the goal pose at any time
// if the aruco pose is not detected anymore, follow the last goal pose until a new goal pose is published

#include "goal_pose_publisher.h"

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher") {
	// Create a publisher for the goal pose
	goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
	// Create a subscriber for the aruco pose array
	aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
		"/aruco/poses", 10, std::bind(&GoalPosePublisher::aruco_pose_callback, this, std::placeholders::_1));

	// Retrieve the camera frame parameter from the config YAML file of the aruco detector

	this->declare_parameter("testing", rclcpp::PARAMETER_BOOL);
	bool testing = this->get_parameter("testing").as_bool(); // whether to load test unit or not
	if (testing) {
		camera_frame_name = "base_link"; // only for testing without aruco detector
	} else {
		// loads the camera frame name from the aruco detector config file
		this->declare_parameter("camera_frame", rclcpp::PARAMETER_STRING);
		camera_frame_name = this->get_parameter("camera_frame").as_string();
		if (camera_frame_name != std::string()) {
			RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
		} else {
			RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
		}
	}

	// Initialize the TF2 transform listener
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// The source frame (aruco's base frame)
	source_frame = camera_frame_name;

	// wait until the tf buffer is filled with the transform from igus_rebel_base_link to link_ref
	while (!tf_buffer_->canTransform(target_frame, link_ref_frame, tf2::TimePointZero)) {
		// RCLCPP_INFO(LOGGER, "Waiting for transform from igus_rebel_base_link to link_ref");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// get the position of the link_ref frame with respect to base link
	geometry_msgs::msg::TransformStamped link_ref_transform;
	try {
		// consider starting point = (x, y, z) = (0, 0, link_ref.z)
		link_ref_transform = tf_buffer_->lookupTransform(target_frame, link_ref_frame, tf2::TimePointZero);
	} catch (tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "Failed to lookup transform: %s", ex.what());
		return;
	}

	RCLCPP_INFO(LOGGER, "link_ref transform: %f, %f, %f", link_ref_transform.transform.translation.x,
				link_ref_transform.transform.translation.y, link_ref_transform.transform.translation.z);

	link_ref_z = link_ref_transform.transform.translation.z;
}

void GoalPosePublisher::aruco_pose_callback(const geometry_msgs::msg::PoseArray aruco_pose_array) {
	if (aruco_pose_array.poses.size() == 0) {
		return; // do not process callback if pose array is empty
	}
	// Read the first pose in the array
	geometry_msgs::msg::Pose aruco_pose = aruco_pose_array.poses[0];

	geometry_msgs::msg::TransformStamped transform;
	try {
		transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
	} catch (tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "Failed to lookup transform: %s", ex.what());
		return;
	}

	// apply the TF transform to the aruco pose to get the position in the link_ref frame
	tf2::doTransform(aruco_pose, aruco_pose, transform); // in, out, transform

	// Calculate the Cartesian distance
	float distance = std::sqrt(
		std::pow(aruco_pose.position.x, 2) +
		std::pow(aruco_pose.position.y, 2) +
		std::pow(aruco_pose.position.z - link_ref_z, 2));

	// log the distance
	RCLCPP_INFO(LOGGER, "Distance from (0, 0, %.3f) to aruco: %.4f", link_ref_z, distance);

	processPose(aruco_pose, distance);
}

void GoalPosePublisher::processPose(const geometry_msgs::msg::Pose aruco_pose, float distance) {
	// compute the closest reachable pose of the end effector to the aruco pose

	geometry_msgs::msg::PoseStamped goal_pose;
	// if the aruco pose is reachable, then the goal pose is the aruco pose
	goal_pose.header.frame_id = target_frame;

	if (distance < goal_radius) {
		// goal pose corresponds to the aruco pose
		goal_pose.pose.position.x = aruco_pose.position.x;
		goal_pose.pose.position.y = aruco_pose.position.y;
		goal_pose.pose.position.z = aruco_pose.position.z;

		// compute new orientation such that the end effector is facing the aruco pose

		// convert aruco orientation to rotation quaternion
		tf2::Quaternion q_aruco_rotation, q_aruco;
		tf2::fromMsg(aruco_pose.orientation, q_aruco_rotation);

		// this quaternion describes a rotation of -pi/2 radians around the y axis.
		// can also be described as the orientation of the y axis rotated by -pi/2 radians around itself
		tf2::Quaternion q_y_orientation(std::cos(-M_PI_2 / 2.0), 0.0, std::sin(-M_PI_2 / 2.0), 0.0);

		// this formula applies the rotation quaternion described by the aruco pose orientation
		// 		to the y axis rotated by -pi/2 radians. This results in the y axis of the original orientation
		// 		being rotated by -pi/2 radians around the aruco pose orientation (used as rotation quaternion)
		q_aruco = q_aruco_rotation * q_y_orientation;

		// apply rotation and save it into the goal pose
		goal_pose.pose.orientation = tf2::toMsg(q_aruco);

	} else {
		// if the aruco pose is not reachable, then the goal pose is at a distance less than 1 meter from link_1
		// compute new x, y, z coordinates such that the new distance from link_1 to goal_pose is at a fixed distance of 0.7m
		// the new goal pose should be in the same direction as the vector pointing the aruco pose from link_1

		// compute the unit vector of the aruco pose
		float aruco_pose_unit_vector_x = aruco_pose.position.x / distance;
		float aruco_pose_unit_vector_y = aruco_pose.position.y / distance;
		float aruco_pose_unit_vector_z = (aruco_pose.position.z - link_ref_z) / distance;

		// compute the new goal pose coordinates
		float goal_pose_x = aruco_pose_unit_vector_x * reachable_radius;
		float goal_pose_y = aruco_pose_unit_vector_y * reachable_radius;
		float goal_pose_z = aruco_pose_unit_vector_z * reachable_radius;

		// set the new goal pose coordinates
		goal_pose.pose.position.x = goal_pose_x;
		goal_pose.pose.position.y = goal_pose_y;
		goal_pose.pose.position.z = goal_pose_z + link_ref_z;

		// compute the new orientation of the goal pose to be equal to the orientation of the direction vector
		float roll = 0.0f;
		float pitch = std::atan2(-goal_pose_z, std::sqrt(goal_pose_x * goal_pose_x + goal_pose_y * goal_pose_y));
		float yaw = std::atan2(goal_pose_y, goal_pose_x);
		tf2::Quaternion rotation;
		rotation.setRPY(roll, pitch, yaw);

		// apply rotation and save it into the goal pose
		goal_pose.pose.orientation = tf2::toMsg(rotation);
	}

	// log the new goal pose
	RCLCPP_INFO(LOGGER, "New goal pose position: %f, %f, %f", goal_pose.pose.position.x, goal_pose.pose.position.y,
				goal_pose.pose.position.z);
	tf2::Quaternion new_orientation(goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
									goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

	// Publish the computed PoseArray to the /goal_pose topic
	goal_pose.header.stamp = this->now();
	goal_pose_pub_->publish(goal_pose);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// Create a node
	auto node = std::make_shared<GoalPosePublisher>();

	rclcpp::spin(node);

	// Clean up node after thermination
	rclcpp::shutdown();
	return 0;
}
