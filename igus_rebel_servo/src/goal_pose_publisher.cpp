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

#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::goal_pose_pub");

//std::thread test_pub_thread_;
//rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr test_pub;

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher") {
    // Create a publisher for the goal pose
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    // Create a subscriber for the aruco pose array
    aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&GoalPosePublisher::aruco_pose_callback, this, std::placeholders::_1));

    // create a publisher on topic aruco poses
    //test_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_poses", 10);
    //test_pub_thread_ = std::thread(&GoalPosePublisher::test_pub_thread, this);

    // Retrieve the camera frame parameter from the config YAML file of the aruco detector
	this->declare_parameter("camera_frame", rclcpp::PARAMETER_STRING);
	camera_frame_name = this->get_parameter("camera_frame").as_string();
    if (camera_frame_name != std::string()) {
        RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to get my_string_parameter");
    }

    //camera_frame_name = "base_link";  // NOTE: only for testing without aruco detector

	// Initialize the TF2 transform listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // The source frame (aruco's base frame)
    source_frame = camera_frame_name;

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
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(LOGGER, "Failed to lookup transform: %s", ex.what());
        return;
    }

	// apply the TF transform to the aruco pose to get the position in the link_1 frame
	tf2::doTransform(aruco_pose, aruco_pose, transform); // in, out, transform

	// apply a 90 degree rotation so that the aruco pose x axis is aligned with the computed z axis
	tf2::Quaternion alignment, current_alignment;
	alignment.setRPY(0.0, M_PI/2.0, 0.0);
	tf2::fromMsg(aruco_pose.orientation, current_alignment);
	alignment = alignment * current_alignment;
	alignment.normalize();
	aruco_pose.orientation = tf2::toMsg(alignment);

	// get the position of the link_1 frame with respect to base link
	geometry_msgs::msg::TransformStamped link_1_transform;
	try {
		link_1_transform = tf_buffer_->lookupTransform("base_link", "link_1", tf2::TimePointZero);
	} catch (tf2::TransformException& ex) {
		RCLCPP_ERROR(LOGGER, "Failed to lookup transform: %s", ex.what());
		return;
	}

    // Calculate the Cartesian distance
    float distance = std::sqrt(
        std::pow(aruco_pose.position.x + link_1_transform.transform.translation.x, 2) +
        std::pow(aruco_pose.position.y + link_1_transform.transform.translation.y, 2) +
        std::pow(aruco_pose.position.z + link_1_transform.transform.translation.z, 2));

    // log the distance
    RCLCPP_INFO(LOGGER, "Distance from link_1 to aruco: %f", distance);

    processPoseArray(aruco_pose, distance);
}

void GoalPosePublisher::processPoseArray(const geometry_msgs::msg::Pose aruco_pose, float distance) {
    // compute the closest reachable pose of the end effector to the aruco pose

    geometry_msgs::msg::PoseStamped goal_pose;
	// if the aruco pose is reachable, then the goal pose is the aruco pose
    goal_pose.header.frame_id = "link_1";

    if (distance < goal_radius) {

		// goal pose corresponds to the aruco pose
		goal_pose.pose.position.x = aruco_pose.position.x;
        goal_pose.pose.position.y = aruco_pose.position.y;
        goal_pose.pose.position.z = aruco_pose.position.z;

        // compute new orientation such that the end effector is facing the aruco pose

        // Extract the reference orientation as a quaternion
        tf2::Quaternion reference_quaternion;
        tf2::fromMsg(aruco_pose.orientation, reference_quaternion);

        // Invert the x-component of the quaternion to reverse the x-axis direction
        tf2::Quaternion new_quaternion = reference_quaternion * tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI);
        
		// Set the new orientation of the pose
        goal_pose.pose.orientation = tf2::toMsg(new_quaternion);

    } else {
        // if the aruco pose is not reachable, then the goal pose is at a distance less than 1 meter from link_1
        // compute new x, y, z coordinates such that the new distance from link_1 to goal_pose is at a fixed distance of 0.7m
        // the new goal pose should be in the same direction as the vector pointing the aruco pose from link_1

        // compute the unit vector of the aruco pose
        float aruco_pose_unit_vector_x = aruco_pose.position.x / distance;
        float aruco_pose_unit_vector_y = aruco_pose.position.y / distance;
        float aruco_pose_unit_vector_z = aruco_pose.position.z / distance;

        // compute the new goal pose coordinates
        float goal_pose_x = aruco_pose_unit_vector_x * 0.7;
        float goal_pose_y = aruco_pose_unit_vector_y * 0.7;
        float goal_pose_z = aruco_pose_unit_vector_z * 0.7;

        // set the new goal pose coordinates
        goal_pose.pose.position.x = goal_pose_x;
        goal_pose.pose.position.y = goal_pose_y;
        goal_pose.pose.position.z = goal_pose_z;

        // compute new orientation such that the end effector is facing the aruco pose
        tf2::Vector3 direction_vector(
            aruco_pose.position.x, // should be link_1.position.x - aruco_pose.position.x
            aruco_pose.position.y, // should be link_1.position.y - aruco_pose.position.y
           	aruco_pose.position.z); // should be link_1.position.z - aruco_pose.position.z

		// compute the new orientation of the goal pose to be equal to the orientation of the direction vector
		float roll = 0;
		float pitch = std::atan2(- direction_vector.z(), std::sqrt(direction_vector.x() * direction_vector.x() + direction_vector.y() * direction_vector.y()));
		float yaw = std::atan2(direction_vector.y(), direction_vector.x());
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
    RCLCPP_INFO(LOGGER, "New goal pose orientation in rpy: %f, %f, %f",
                new_orientation.x(), new_orientation.y(), new_orientation.z());

    // Publish the computed PoseArray to the "/goal_pose" topic
    goal_pose.header.stamp = this->now();
    goal_pose_pub_->publish(goal_pose);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<GoalPosePublisher>();

    rclcpp::spin(node);

    // Clean up node after thermination
    rclcpp::shutdown();
    return 0;
}

// creates a sample pose, then publishes it to the topic /aruco_poses
// it is used to test whether the goal pose publisher is working correctly
void GoalPosePublisher::test_pub_thread() {
    // create a test for a pose array
    geometry_msgs::msg::PoseArray test_pose_array;
    geometry_msgs::msg::Pose test_pose;
    test_pose_array.header.frame_id = "base_link";
    test_pose.position.x = 0.7;
    test_pose.position.y = 0.7;
    test_pose.position.z = 0.7;
    tf2::Quaternion orientation;
	orientation.setRPY(-2.132123, -0.143221, 0.443123);
	test_pose.orientation = tf2::toMsg(orientation);
    test_pose_array.poses.push_back(test_pose);

	tf2::Quaternion base_orientation(test_pose.orientation.x, test_pose.orientation.y,
                                     test_pose.orientation.z, test_pose.orientation.w);
    RCLCPP_INFO(LOGGER, "Base pose orientation in rpy: %f, %f, %f",
                base_orientation.x(), base_orientation.y(), base_orientation.z());


    rclcpp::WallRate loop_rate(1);  // Publish at 1 Hz

    int count = 0;
    // Publish a the test pose every second
    while (count < 100) {
        test_pose_array.header.stamp = this->now();
        //test_pub->publish(test_pose_array);

        loop_rate.sleep();
        count++;
    }
}
