// ROS2 imports
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ imports
#include <thread>

std::thread test_thread_;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test::test_button_pose_computation");

// publisher for poses computed
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr test_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr test_button_pub;

geometry_msgs::msg::Pose compute_button_pose(geometry_msgs::msg::Pose aruco, float delta_x, float delta_y, float delta_z, tf2::Quaternion rotation);

// creates a sample pose, then publishes it to the topic /aruco_poses
// it is used to test whether the goal pose publisher is working correctly
void test_thread() {
	// create a test for a pose array
	geometry_msgs::msg::PoseStamped test_pose;
	test_pose.header.frame_id = "base_link";
	test_pose.pose.position.x = 0.4f;
	test_pose.pose.position.y = -0.4f;
	test_pose.pose.position.z = 0.4f;

	tf2::Quaternion orientation;
	orientation.setRPY(-2.132123, -0.143221, 0.443123);
	orientation.normalize();

	RCLCPP_INFO(LOGGER, "Base pose orientation in rpy: %f, %f, %f",
				orientation.x(), orientation.y(), orientation.z());

	// alternative quaternion definition
	// tf2::Quaternion orientation(0.47927573323249817, -0.021937279030680656, 0.8773083090782166, 0.011984390206634998);

	test_pose.pose.orientation = tf2::toMsg(orientation);

	// this quaternion describes a rotation of -pi/2 radians around the y axis.
	// can also be described as the orientation of the y axis rotated by -pi/2 radians around itself
	tf2::Quaternion q_y_orientation(std::cos(-M_PI_2 / 2.0), 0.0, std::sin(-M_PI_2 / 2.0), 0.0);

	geometry_msgs::msg::Pose button_pose = compute_button_pose(test_pose.pose, 0.2, 0.2, 0.0, q_y_orientation);
	geometry_msgs::msg::PoseStamped button_pose_stamped;
	button_pose_stamped.header.frame_id = "base_link";
	button_pose_stamped.pose = button_pose;

	RCLCPP_INFO(LOGGER, "Button pose orientation in rpy: %f, %f, %f",
				button_pose.orientation.x, button_pose.orientation.y, button_pose.orientation.z);
	RCLCPP_INFO(LOGGER, "Button pose position: %f, %f, %f",
				button_pose.position.x, button_pose.position.y, button_pose.position.z);

	// now publish both poses computed at 1hz for 100 seconds
	rclcpp::WallRate loop_rate(1); // Publish at 1 Hz

	int count = 0;
	// Publish a the test pose every second
	while (count < 100) {
		test_pose.header.stamp = rclcpp::Clock().now();
		test_pub->publish(test_pose);
		loop_rate.sleep();
		button_pose_stamped.header.stamp = rclcpp::Clock().now();
		test_button_pub->publish(button_pose_stamped);

		loop_rate.sleep();
		count++;
	}
}

geometry_msgs::msg::Pose compute_button_pose(geometry_msgs::msg::Pose aruco, float delta_x, float delta_y, float delta_z, tf2::Quaternion rotation) {
	// create tf2 transform from aruco pose
	tf2::Transform aruco_tf2;
	tf2::fromMsg(aruco, aruco_tf2);

	// create tf2 transform from delta pose
	tf2::Transform delta_tf2;
	delta_tf2.setOrigin(tf2::Vector3(delta_x, delta_y, delta_z));
	delta_tf2.setRotation(rotation);

	// compute button pose as the composition of the aruco pose and the delta pose
	tf2::Transform button_tf2 = aruco_tf2 * delta_tf2;

	// convert tf2 transform to geometry_msgs::msg::Transform
	geometry_msgs::msg::Transform button_transformed;
	button_transformed = tf2::toMsg(button_tf2);

	// convert geometry_msgs::msg::Transform to geometry_msgs::msg::Pose
	geometry_msgs::msg::Pose button_pose;
	button_pose.position.x = button_transformed.translation.x;
	button_pose.position.y = button_transformed.translation.y;
	button_pose.position.z = button_transformed.translation.z;
	button_pose.orientation = button_transformed.rotation;

	return button_pose;
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// Create a local node
	auto node = std::make_shared<rclcpp::Node>("test_button_pose_computation");

	test_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);
	test_button_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/button_pose", 10);

	// Create a thread to publish the test pose
	test_thread_ = std::thread(test_thread);

	if (test_thread_.joinable()) {
		test_thread_.join();
	}

	rclcpp::spin(node);

	// Clean up node after thermination
	rclcpp::shutdown();
	return 0;
}