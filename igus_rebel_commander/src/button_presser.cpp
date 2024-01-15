// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.h"

ButtonPresser::ButtonPresser(const rclcpp::NodeOptions& node_options) : Node("button_presser_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting button presser demo");

	// aruco goal pose subscriber with callback thread
	aruco_markers_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco_markers", 10, std::bind(&ButtonPresser::arucoMarkersCallback, this, std::placeholders::_1));

	ready = false;

	// track the goal pose asynchrounously
	//button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, this);
	//button_presser_demo_thread.detach();
}

void ButtonPresser::initPlanner() {
	button_presser_node = shared_from_this();

	// Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
	// a RobotModel and a PlanningScene.
	//
	// We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
	// parameter server and construct a RobotModel for us to use.

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
		new robot_model_loader::RobotModelLoader(button_presser_node, "robot_description"));

	move_group = new moveit::planning_interface::MoveGroupInterface(button_presser_node, PLANNING_GROUP);

	// We can print the name of the reference frame for this robot.
	RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	// "flange" for simple robot movement, "toucher" for robot arm + camera
	end_effector_link = move_group->getEndEffectorLink();
	RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

	// We can get a list of all the groups in the robot:
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
			  std::ostream_iterator<std::string>(std::cout, ", "));

	// We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information
	const moveit::core::RobotModelPtr& robot_model = robot_model_loader->getModel();

	// Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

	move_group->setStartState(*robot_state);

	// Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	// group is useful for dealing with one set of joints at a time such as a left arm or a end effector
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	// Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
	planning_scene = new planning_scene::PlanningScene(robot_model);

	RCLCPP_INFO(LOGGER, "Planner and utilities initialized");

	loadPlanner(robot_model);
}

void ButtonPresser::loadPlanner(const moveit::core::RobotModelPtr& robot_model) {
	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

	std::string planner_plugin_name;

	// We will get the name of planning plugin we want to load
	// from the ROS parameter server, and then load the planner
	// making sure to catch all exceptions.
	if (!button_presser_node->get_parameter("planning_plugin", planner_plugin_name))
		RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
	try {
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException& ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, button_presser_node, button_presser_node->get_namespace())) {
			RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
		}
		RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
	} catch (pluginlib::PluginlibException& ex) {
		const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (const auto& cls : classes)
			ss << cls << " ";
		RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
					 ex.what(), ss.str().c_str());
	}

	RCLCPP_INFO(LOGGER, "Loaded planner and planning plugin");
}

void ButtonPresser::initRvizVisualTools() {
	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	namespace rvt = rviz_visual_tools;
	// @param node - base frame - markers topic - robot model
	visual_tools = new moveit_visual_tools::MoveItVisualTools(button_presser_node, "base_link", "/rviz_visual_tools", move_group->getRobotModel());

	// extra options
	visual_tools->setPlanningSceneTopic("/move_group/monitored_planning_scene");
	visual_tools->loadPlanningSceneMonitor();
	visual_tools->enableBatchPublishing();

	visual_tools->deleteAllMarkers();

	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools->loadRemoteControl();

	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools->publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XXLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools->trigger();

	RCLCPP_INFO(LOGGER, "Loaded rviz visual tools");
}

/**
 * @brief Move the robot to the static predefined searching pose
*/
void ButtonPresser::moveToSearchingPose() {
	// define a pre-defined pose for the robot to look at the buttons setup: simplified version of the demo
	// define pose as an array of target joints positions, then use moveit planning in joint space to move the robot

	RCLCPP_INFO(LOGGER, "Moving the robot to searching pose");

	// Setup a joint space goal
	moveit::core::RobotState goal_state(*move_group->getCurrentState());
	goal_state.setJointGroupPositions(joint_model_group, search_joints_positions);

	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(0.002);    // 1 mm
	move_group->setGoalOrientationTolerance(0.01);  // 0.01 rad

	bool valid_motion = move_group->setJointValueTarget(goal_state);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Target joints outside their phyisical limits");
		return;
	}

	// Get the pose of a specific link (e.g., the end-effector link).
	const Eigen::Isometry3d goal_pose = goal_state.getGlobalLinkTransform(end_effector_link);

	// use rviz visual tools to publish a coordinate axis corresponding to the goal pose defined
	visual_tools->publishAxisLabeled(goal_pose, "search_pose");
	visual_tools->trigger();

	moveit::planning_interface::MoveGroupInterface::Plan static_search_plan;
	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(0.2);
	move_group->setMaxAccelerationScalingFactor(0.2);
	moveit::core::MoveItErrorCode response = move_group->plan(static_search_plan);

	// visualizing the trajectory
	RCLCPP_INFO(LOGGER, "Result of plannning to the searching position = %s", moveit::core::error_code_to_string(response).c_str());

	visual_tools->publishTrajectoryPath(static_search_plan.trajectory, static_search_plan.start_state);
	visual_tools->trigger();

	if (bool(response)) {
		RCLCPP_INFO(LOGGER, "moving the robot to searching pose with joint space goal");
		move_group->execute(static_search_plan);
	} else {
		RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
	}
}

void ButtonPresser::arucoMarkersCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array) {
	// to be run until the aruco markers are detected and sorted -> demo ready
	if (!ready) {
		// first check if the markers have all been detected
		if (aruco_markers_array->poses.size() != n_btns) {
			return;  // skip and wait until all markers are detected
		}

		// then check whether the markers have the correct ids
		for (int i = 0; i < n_btns; i++) {
			if (aruco_markers_array->marker_ids[i] != btn_1 &&
				aruco_markers_array->marker_ids[i] != btn_2 &&
				aruco_markers_array->marker_ids[i] != btn_3) {
				return;  // skip and wait until all markers are detected
			}
		}

		// assign the markers to the correct array position based on their id
		for (int i = 0; i < n_btns; i++) {
			for (int j = 0; j < n_btns; j++) {
				if (aruco_markers_array->marker_ids[j] == btn_ids[i]) {
					aruco_markers[i] = aruco_markers_array->poses[j];
					break;
				}
			}
		}

		// set the ready flag after successful button setup detection
		ready = true;
		RCLCPP_INFO(LOGGER, "Button setup detected, demo can start");
	}
}

void ButtonPresser::buttonPresserDemoThread() {
	RCLCPP_INFO(LOGGER, "Starting button presser demo thread");
	while (rclcpp::ok()) {
		if (ready) {
			break;
		} else {
			
			// wait until it's ready
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			continue;
		}
	}
	RCLCPP_INFO(LOGGER, "Ending button presser demo thread");
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the PoseStampedSubscriberNode
	auto node = std::make_shared<ButtonPresser>(node_options);

	rclcpp::executors::SingleThreadedExecutor executor;
    
    auto main_thread = std::make_unique<std::thread>([&executor, &node]() { 
		executor.add_node(node->get_node_base_interface());
		executor.spin();
	});


	// initialize planner, move group, planning scene and get general info
	node->initPlanner();

	// initialize visual tools for drawing on rviz
	node->initRvizVisualTools();

	// wait 5 seconds so that the robot does not move instantly
	//std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	node->moveToSearchingPose();

	// start the demo thread
	std::thread button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, node);
	button_presser_demo_thread.detach();

	main_thread->join();
	button_presser_demo_thread.join();

	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
