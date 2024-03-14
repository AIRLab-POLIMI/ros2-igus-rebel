#include "aruco_action_server.hpp"

ArucoActionServer::ArucoActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("goal_pose_publisher", options) {
	using namespace std::placeholders;

	// Initialize action server
	this->action_server_ = rclcpp_action::create_server<Aruco>(
		this,
		"aruco",
		std::bind(&ArucoActionServer::handle_goal, this, _1, _2),
		std::bind(&ArucoActionServer::handle_cancel, this, _1),
		std::bind(&ArucoActionServer::handle_accepted, this, _1));

	// Create a publisher for the goal pose
	goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pointer", 10);

	// Create a subscriber for the aruco marker array
	aruco_marker_sub_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers", 10, std::bind(&ArucoActionServer::aruco_marker_callback, this, std::placeholders::_1));

	// Create a subscriber for the goal done message
	goal_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/goal/done", 10, std::bind(&ArucoActionServer::goal_done_callback, this, std::placeholders::_1));

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

	int64_t test_id = 10;
	geometry_msgs::msg::Pose test_pose;
	test_pose.position.x = 1.0;
	test_pose.position.x = 1.0;
	arucos[test_id] = test_pose;

	test_id = 1;
	test_pose.position.x = -1.0;
	test_pose.position.x = -1.0;
	arucos[test_id] = test_pose;

	test_id = 7;
	test_pose.position.x = 1.4;
	test_pose.position.x = -1.6;
	arucos[test_id] = test_pose;
}

void ArucoActionServer::aruco_marker_callback(const aruco_interfaces::msg::ArucoMarkers aruco_marker_array) {
	// Update the map with received marker IDs and poses
	for (size_t i = 0; i < aruco_marker_array.marker_ids.size(); ++i) {
		int64_t id = aruco_marker_array.marker_ids[i];
		geometry_msgs::msg::Pose pose = aruco_marker_array.poses[i];
		arucos[id] = pose; // Using the subscript operator
	}
}

void ArucoActionServer::goal_done_callback(const std_msgs::msg::Bool goal_done_msg) {
	goal_done = goal_done_msg.data;
}

void ArucoActionServer::processPose(const geometry_msgs::msg::Pose aruco_pose, float distance) {
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

rclcpp_action::GoalResponse ArucoActionServer::handle_goal(
	const rclcpp_action::GoalUUID &uuid,
	std::shared_ptr<const Aruco::Goal> goal) {
	goal_id = goal->aruco_id;
	RCLCPP_INFO(this->get_logger(), "Check if aruco with id %ld exists...", goal_id);
	(void)uuid;

	auto it = arucos.find(goal->aruco_id);
	if (it != arucos.end()) {
		// ID exists in the map
		// You can access the corresponding pose using it->second
		RCLCPP_INFO(this->get_logger(), "Aruco with id %ld found, execute goal.", goal_id);

		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	} else {
		// ID doesn't exist in the map
		RCLCPP_INFO(this->get_logger(), "Aruco with id %ld not found, cancel Goal.", goal_id);
		return rclcpp_action::GoalResponse::REJECT;
	}
}

rclcpp_action::CancelResponse ArucoActionServer::handle_cancel(
	const std::shared_ptr<GoalHandleAruco> goal_handle) {
	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}

void ArucoActionServer::handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle) {
	using namespace std::placeholders;
	std::thread{std::bind(&ArucoActionServer::execute, this, _1), goal_handle}.detach();
}

void ArucoActionServer::execute(const std::shared_ptr<GoalHandleAruco> goal_handle) {
	auto result = std::make_shared<Aruco::Result>();
	(void)goal_handle;
	RCLCPP_INFO(this->get_logger(), "Executing goal");

	geometry_msgs::msg::Pose aruco_pose = arucos[goal_id];

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

	while (!goal_done) {
		sleep(1);
	}
	goal_done = false;
	goal_handle->succeed(result);
	RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto action_server = std::make_shared<ArucoActionServer>();

	rclcpp::spin(action_server);

	rclcpp::shutdown();
	return 0;
}
