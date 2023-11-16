// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// C++ imports
#include <thread>

std::thread test_pub_thread_;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr test_pub;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test::test_goal_pose_computation");

// creates a sample pose, then publishes it to the topic /aruco_poses
// it is used to test whether the goal pose publisher is working correctly
void test_pub_thread() {
    // create a test for a pose array
    geometry_msgs::msg::PoseArray test_pose_array;
    geometry_msgs::msg::Pose test_pose;
    test_pose_array.header.frame_id = "base_link";
    test_pose.position.x = 0.8f;
    test_pose.position.y = -0.8f;
    test_pose.position.z = 0.8f;

    tf2::Quaternion orientation;
	orientation.setRPY(-2.132123, -0.143221, 0.443123);
	//orientation.normalize();

	//tf2::Quaternion orientation(0.47927573323249817, -0.021937279030680656, 0.8773083090782166, 0.011984390206634998);

	test_pose.orientation = tf2::toMsg(orientation);
    test_pose_array.poses.push_back(test_pose);

    RCLCPP_INFO(LOGGER, "Base pose orientation in rpy: %f, %f, %f",
                orientation.x(), orientation.y(), orientation.z());

    rclcpp::WallRate loop_rate(1);  // Publish at 1 Hz

    int count = 0;
    // Publish a the test pose every second
    while (count < 100) {
        test_pose_array.header.stamp = rclcpp::Clock().now();
        test_pub->publish(test_pose_array);

        loop_rate.sleep();
        count++;
    }
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	// Create a local node
	auto node = std::make_shared<rclcpp::Node>("test_goal_pose_computation");

	// Create a test publisher
	test_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_poses", 10);

	// Create a thread to publish the test pose
	test_pub_thread_ = std::thread(test_pub_thread);

	rclcpp::spin(node);

	// Clean up node after thermination
	rclcpp::shutdown();
	return 0;
}