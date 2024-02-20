// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the button presser node
	auto node = std::make_shared<ButtonPresser>(node_options);

	rclcpp::executors::MultiThreadedExecutor executor;

	auto main_thread = std::make_unique<std::thread>([&executor, &node]() {
		executor.add_node(node->get_node_base_interface());
		executor.spin();
	});

	// initialize planner, move group, planning scene and get general info
	node->initPlanner();

	// initialize visual tools for drawing on rviz
	node->initRvizVisualTools();

	// NOTE: change the following function to switch between static search and dynamic search

	// move to the predefined static searching pose
	// node->moveToSearchingPose();

	// alternatively start waving the robot arm to find the buttons setup
	node->lookAroundForArucoMarkers(true);

	// start the demo thread once the robot is in the searching pose
	std::thread button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, node);
	button_presser_demo_thread.detach();

	main_thread->join();
	button_presser_demo_thread.join();

	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
