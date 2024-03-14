
// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// Code from the move group interface tutorial
// https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// other useful links:
// https://moveit.picknik.ai/main/doc/examples/motion_planning_pipeline/motion_planning_pipeline_tutorial.html
// https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html

#include "aruco_follower.h"

ArucoFollower::ArucoFollower(const rclcpp::NodeOptions &node_options) : Node("follower_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting Aruco follower with moveit cpp interface library");

	// aruco goal pose subscriber with callback thread
	goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/aruco_pointer", 10, std::bind(&ArucoFollower::goalPoseCallback, this, std::placeholders::_1));

	goal_done_pub = this->create_publisher<std_msgs::msg::Bool>("/goal/done", 10);

	goal_pose.header.frame_id = ""; // initialize the goal pose for the fisrt assignment

	// track the goal pose asynchrounously
	track_goal_pose_thread = std::thread(&ArucoFollower::trackGoalPose, this);
	track_goal_pose_thread.detach();
}

void ArucoFollower::initPlanner() {
	follower_node = shared_from_this();

	// Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
	// a RobotModel and a PlanningScene.
	//
	// We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
	// parameter server and construct a RobotModel for us to use.

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
		new robot_model_loader::RobotModelLoader(follower_node, "robot_description"));

	move_group = new moveit::planning_interface::MoveGroupInterface(follower_node, PLANNING_GROUP);

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
	const moveit::core::RobotModelPtr &robot_model = robot_model_loader->getModel();

	// Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

	move_group->setNumPlanningAttempts(5);
	move_group->setStartState(*robot_state);

	// Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	// group is useful for dealing with one set of joints at a time such as a left arm or a end effector
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	// Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
	planning_scene = new planning_scene::PlanningScene(robot_model);

	RCLCPP_INFO(LOGGER, "Planner and utilities initialized");

	loadPlanner(robot_model);
}

void ArucoFollower::loadPlanner(const moveit::core::RobotModelPtr &robot_model) {
	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

	std::string planner_plugin_name;

	// We will get the name of planning plugin we want to load
	// from the ROS parameter server, and then load the planner
	// making sure to catch all exceptions.
	if (!follower_node->get_parameter("planning_plugin", planner_plugin_name))
		RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
	try {
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, follower_node, follower_node->get_namespace())) {
			RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
		}
		RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
	} catch (pluginlib::PluginlibException &ex) {
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (const auto &cls : classes)
			ss << cls << " ";
		RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
					 ex.what(), ss.str().c_str());
	}

	RCLCPP_INFO(LOGGER, "Loaded planner and planning plugin");
}

void ArucoFollower::initRvizVisualTools() {
	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	namespace rvt = rviz_visual_tools;
	// @param node - base frame - markers topic - robot model
	visual_tools = new moveit_visual_tools::MoveItVisualTools(follower_node, fixed_base_frame, "/rviz_visual_tools", move_group->getRobotModel());

	// extra options
	visual_tools->setPlanningSceneTopic("/move_group/monitored_planning_scene");
	visual_tools->loadPlanningSceneMonitor();
	visual_tools->enableBatchPublishing();
	visual_tools->setBaseFrame(fixed_base_frame);
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

void ArucoFollower::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
	// Process the received PoseStamped message
	RCLCPP_INFO(get_logger(), "Received PoseStamped message: (%.3f, %.3f, %.3f)",
				msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	{ // get mutex lock on the goal pose to change it when it is received
		std::lock_guard<std::mutex> lock(goal_pose_mutex);
		// set the goal pose
		goal_pose = *msg;
	}
}

void ArucoFollower::trackGoalPose() {
	// Pose goal from the last goal pose received
	bool skip_pose = false;

	while (rclcpp::ok()) {
		{ // acquire lock on the goal pose to read the last pose available
			std::lock_guard<std::mutex> lock(goal_pose_mutex);
			// check if the current goal pose has been initialized

			if (goal_pose.header.frame_id != "") { // if the goal pose has been initialized
												   // if the newly recorded goal pose is different from the last one
				if (goal_pose.header.stamp != current_goal_pose.header.stamp) {
					current_goal_pose = goal_pose; // copy the last goal pose received
					skip_pose = false;
				} else {
					skip_pose = true;
				}
			} else {
				skip_pose = true;
			}
		}

		// skip this iteration if the pose is the same as the last one
		if (skip_pose)
			continue;

		RCLCPP_INFO(get_logger(), "new goal: (%.3f, %.3f, %.3f)",
					current_goal_pose.pose.position.x, current_goal_pose.pose.position.y, current_goal_pose.pose.position.z);

		// publish a coordinate axis corresponding to the pose with rviz visual tools
		visual_tools->publishAxisLabeled(current_goal_pose.pose, "target");
		visual_tools->trigger();

		move_group->setStartState(*move_group->getCurrentState());
		move_group->setGoalPositionTolerance(0.005);   // meters
		move_group->setGoalOrientationTolerance(0.1); // radians
		move_group->setPoseTarget(current_goal_pose, end_effector_link);
		move_group->setPlannerId("RRTConnectkConfigDefault");
		// optionally limit accelerations and velocity scaling
		move_group->setMaxVelocityScalingFactor(1.0);
		move_group->setMaxAccelerationScalingFactor(0.5);
		// move_group->setPlanningPipelineId("ompl");

		// create plan for reaching the goal pose
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		moveit::core::MoveItErrorCode response = move_group->plan(my_plan);

		// show output of planned movement
		RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal): result = %s", moveit::core::error_code_to_string(response).c_str());

		// retrieve trajectory computed by the planner
		moveit_msgs::msg::RobotTrajectory trajectory = my_plan.trajectory;
		// Visualize the trajectory
		RCLCPP_INFO(LOGGER, "Visualizing the trajectory");

		visual_tools->publishTrajectoryLine(trajectory, joint_model_group);
		visual_tools->trigger();

		if (bool(response)) { // if the plan was successful
			RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
			moveit::core::MoveItErrorCode plan_response = move_group->execute(my_plan);
			std_msgs::msg::Bool msg;
			msg.data = (bool)plan_response;
			goal_done_pub->publish(msg);
		} else {
			RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
		}
	}
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the PoseStampedSubscriberNode
	auto node = std::make_shared<ArucoFollower>(node_options);

	// initialize planner, move group, planning scene and get general info
	node->initPlanner();

	// initialize visual tools for drawing on rviz
	node->initRvizVisualTools();

	// Spin the node in the main thread
	rclcpp::spin(node);

	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
