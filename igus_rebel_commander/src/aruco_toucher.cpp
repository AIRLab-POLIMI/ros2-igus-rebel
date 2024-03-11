
// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// Code from the move group interface tutorial
// https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// other useful links:
// https://moveit.picknik.ai/main/doc/examples/motion_planning_pipeline/motion_planning_pipeline_tutorial.html
// https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html

#include "aruco_toucher.hpp"

ArucoToucher::ArucoToucher(const rclcpp::NodeOptions &node_options) : Node("toucher_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting Aruco follower with moveit cpp interface library");

	// aruco goal pose subscriber with callback thread
	aruco_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
		aruco_topic, 10, std::bind(&ArucoToucher::aruco_callback, this, std::placeholders::_1));

	// loads the camera frame name from the aruco detector config file
	// this->declare_parameter("camera_frame", rclcpp::PARAMETER_STRING);
	camera_frame_name = this->get_parameter("camera_frame").as_string();
	if (camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// track the goal pose asynchrounously
	aruco_toucher_thread_ = std::thread(&ArucoToucher::aruco_toucher_thread, this);
	aruco_toucher_thread_.detach();
}

void ArucoToucher::initPlanner() {
	aruco_toucher_node = shared_from_this();

	// Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
	// a RobotModel and a PlanningScene.
	//
	// We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
	// parameter server and construct a RobotModel for us to use.

	robot_model_loader::RobotModelLoaderPtr robot_model_loader(
		new robot_model_loader::RobotModelLoader(aruco_toucher_node, "robot_description"));

	move_group = new moveit::planning_interface::MoveGroupInterface(aruco_toucher_node, PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// planning_scene_interface_ = &planning_scene_interface;

	// We can print the name of the reference frame for this robot.
	root_base_frame = move_group->getPlanningFrame();
	RCLCPP_INFO(LOGGER, "Planning frame: %s", root_base_frame.c_str());
	move_group->setPoseReferenceFrame(fixed_base_frame);
	RCLCPP_INFO(LOGGER, "Pose reference frame: %s", move_group->getPoseReferenceFrame().c_str());

	// print the name of the end-effector link for this group.
	// "flange" for simple robot movement, "toucher" for robot arm + camera
	end_effector_link = move_group->getEndEffectorLink();
	RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

	// We can get a list of all the groups in the robot:
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::vector<std::string> group_names = robot_model_loader->getModel()->getJointModelGroupNames();
	for (const auto &group_name : group_names) {
		RCLCPP_INFO(LOGGER, "  %s", group_name.c_str());
	}

	// We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information
	const moveit::core::RobotModelPtr &robot_model = robot_model_loader->getModel();

	// Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

	move_group->setStartState(*robot_state);

	// Set the number of planning attempts to a very high number to guarantee maximum
	// probability of finding a valid plan to be executed
	move_group->setNumPlanningAttempts(150);

	// Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	// group is useful for dealing with one set of joints at a time such as a left arm or a end effector
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	std::vector<const moveit::core::LinkModel *> tips;
	bool check_tips = joint_model_group->getEndEffectorTips(tips);
	if (check_tips) {
		RCLCPP_INFO(LOGGER, "End effector tips:");
		for (const auto &tip : tips) {
			RCLCPP_INFO(LOGGER, "Tip: %s", tip->getName().c_str());
		}
	} else {
		RCLCPP_ERROR(LOGGER, "Could not get end effector tips");
	}

	// get the default parked group state joints values
	std::map<std::string, double> group_state_map = std::map<std::string, double>();
	joint_model_group->getVariableDefaultPositions("parked", group_state_map);
	parked_joint_positions = std::vector<double>(group_state_map.size());
	std::string joint_names_values = "Parked joint positions: ";
	for (const auto &pair : group_state_map) {
		// assumes that the joints are ordered in the same way as the move_group interface does
		parked_joint_positions[joint_model_group->getVariableGroupIndex(pair.first)] = pair.second;
		joint_names_values += pair.first + ": " + std::to_string(pair.second) + ", ";
	}
	RCLCPP_INFO(LOGGER, "%s", joint_names_values.c_str());

	// Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
	planning_scene = new planning_scene::PlanningScene(robot_model);

	RCLCPP_INFO(LOGGER, "Planner and utilities initialized");

	loadPlanner(robot_model);
}

void ArucoToucher::loadPlanner(const moveit::core::RobotModelPtr &robot_model) {
	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

	std::string planner_plugin_name;

	// We will get the name of planning plugin we want to load
	// from the ROS parameter server, and then load the planner
	// making sure to catch all exceptions.
	if (!aruco_toucher_node->get_parameter("planning_plugin", planner_plugin_name))
		RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
	try {
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, aruco_toucher_node, aruco_toucher_node->get_namespace())) {
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

void ArucoToucher::initRvizVisualTools() {
	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	namespace rvt = rviz_visual_tools;
	// @param node - base frame - markers topic - robot model
	visual_tools = new moveit_visual_tools::MoveItVisualTools(aruco_toucher_node, fixed_base_frame, "/rviz_visual_tools", move_group->getRobotModel());

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
	visual_tools->publishText(text_pose, "Button Presser Demo", rvt::WHITE, rvt::XXLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools->trigger();

	RCLCPP_INFO(LOGGER, "Loaded rviz visual tools");
}

void ArucoToucher::aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
	// first check if the markers have all been detected
	if (msg->poses.size() != 1) {
		return; // skip and wait until a single marker is detected
	}

	// transform the pose of the aruco marker with respect to the fixed base frame of reference
	geometry_msgs::msg::TransformStamped tf_camera_base_msg;
	try {
		tf_camera_base_msg = tf_buffer_->lookupTransform(fixed_base_frame, camera_frame_name, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return;
	}

	geometry_msgs::msg::Pose aruco_marker_tf_pose;
	// assign the markers to the correct array position based on their id

	// transform the aruco poses to the fixed base frame of reference
	tf2::doTransform(msg->poses[0], aruco_marker_tf_pose, tf_camera_base_msg);

	// create a PoseStamped message to return with the aruco marker pose estimated
	geometry_msgs::msg::PoseStamped::SharedPtr aruco_marker_tf_pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
	aruco_marker_tf_pose_stamped->pose = aruco_marker_tf_pose;
	aruco_marker_tf_pose_stamped->header.frame_id = fixed_base_frame;
	aruco_marker_tf_pose_stamped->header.stamp = this->now();

	// convert pose quaternion into rotation matrix
	tf2::Quaternion pose_quaternion;
	tf2::fromMsg(aruco_marker_tf_pose_stamped->pose.orientation, pose_quaternion);

	// these quaternions describe the rotations required to get from the aruco poses to
	// the end effector pose in such a way that the end effector (last joint) doesn't rotate if not necessary
	const tf2::Quaternion flip_rotation = tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), M_PI_2);
	const tf2::Quaternion extra_rotation = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -M_PI_2);

	// apply the rotations to the pose quaternion and save the result
	tf2::Quaternion flip_quat = flip_rotation * extra_rotation;

	flip_quat = pose_quaternion * flip_quat;
	flip_quat = flip_quat.normalized();

	// save the quaternion into the transformed pose
	aruco_marker_tf_pose_stamped->pose.orientation = tf2::toMsg(flip_quat);

	{
		// acquire reference aruco marker mutex lock
		std::lock_guard<std::mutex> lock(goal_pose_mutex);
		// save the reference aruco marker
		this->goal_pose = aruco_marker_tf_pose_stamped;
	}

	ready = true;
}

void ArucoToucher::aruco_toucher_thread() {
	// sleep for a while before starting the test
	std::this_thread::sleep_for(std::chrono::seconds(5));

	this->robotPlanAndMove(search_joint_positions);

	while (rclcpp::ok()) {
		if (ready) {
			RCLCPP_INFO(LOGGER, "Goal available");
			// acquire lock on the goal pose to read the last pose available
			{
				std::lock_guard<std::mutex> lock(goal_pose_mutex);
				// check if the current goal pose has been initialized
				if (goal_pose != nullptr) {
					// get the current goal pose
					current_goal_pose = goal_pose;
					// reset the ready flag
					ready = false;
				}
			}

			// plan and move the robot to the current goal pose
			this->robotPlanAndMove(current_goal_pose);

			// sleep for a while
			std::this_thread::sleep_for(std::chrono::seconds(10));

			this->robotPlanAndMove(search_joint_positions);

		} else {
			// sleep for a while
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}

/**
 * @param target_pose the cartesian pose target with reference frame associated
 * @return result of the movement
 * @brief Plan and move the robot to the target pose
 */
bool ArucoToucher::robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
	RCLCPP_INFO(LOGGER, "Planning and moving to target cartesian pose");

	// publish a coordinate axis corresponding to the pose with rviz visual tools
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishAxisLabeled(target_pose->pose, "target");
	visual_tools->trigger();

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(0.001);
	move_group->setGoalOrientationTolerance(0.05);
	move_group->setPoseTarget(*target_pose, end_effector_link);
	move_group->setPlannerId("RRTConnectkConfigDefault");
	move_group->setPlanningTime(5.0);

	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(0.3);
	move_group->setMaxAccelerationScalingFactor(0.3);

	move_group->clearPathConstraints();

	// create plan for reaching the goal pose
	// make several attempts at planning until a valid motion is found or the maximum number of retries is reached
	int attempt = 0;
	bool valid_motion = false;
	moveit::planning_interface::MoveGroupInterface::Plan plan_motion;
	moveit::core::MoveItErrorCode response;
	while (attempt < 3 && !valid_motion) {
		// attempt at planning and moving to the joint space goal
		response = move_group->plan(plan_motion);
		valid_motion = bool(response);
		attempt++;
	}

	// show output of planned movement
	RCLCPP_INFO(LOGGER, "Plan result = %s", moveit::core::error_code_to_string(response).c_str());

	// visualizing the trajectory
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	auto link_eef = move_group->getCurrentState()->getLinkModel(end_effector_link);
	// TODO: this trajectory line is not displayed properly in rviz
	visual_tools->setBaseFrame(plan_motion.trajectory.multi_dof_joint_trajectory.header.frame_id);
	visual_tools->publishTrajectoryLine(plan_motion.trajectory, link_eef, joint_model_group);
	visual_tools->trigger();

	if (bool(response)) { // if the plan was successful
		RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
		move_group->execute(plan_motion);
	} else {
		RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
	}
	return bool(response);
}

/**
 * @brief Plan and move the robot to the joint space goal
 * @param joint_space_goal the joint space goal, sequence of 6 joint values expressed in radians
 * @return true if plan and movement were successful, false otherwise
 */
bool ArucoToucher::robotPlanAndMove(std::vector<double> joint_space_goal) {
	// Setup a joint space goal
	moveit::core::RobotState goal_state(*move_group->getCurrentState());
	goal_state.setJointGroupPositions(joint_model_group, joint_space_goal);

	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(0.001);   // meters ~ 5 mm
	move_group->setGoalOrientationTolerance(0.05); // radians ~ 5 degrees
	move_group->setPlanningTime(5.0);
	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(0.5);
	move_group->setMaxAccelerationScalingFactor(0.3);

	bool valid_motion = move_group->setJointValueTarget(goal_state);
	if (!valid_motion) {
		RCLCPP_ERROR(LOGGER, "Target joints outside their physical limits");
		return false;
	}

	// gets end effector link with respect to global frame of reference (root frame)
	const Eigen::Isometry3d goal_pose = goal_state.getGlobalLinkTransform(end_effector_link);

	// lookup transform from global frame of reference (root frame) to fixed base frame
	geometry_msgs::msg::TransformStamped tf_base_footprint_msg;
	try {
		// lookup transform from root base frame (base_footprint when load_base = true) to fixed base frame (igus rebel base link)
		tf_base_footprint_msg = tf_buffer_->lookupTransform(fixed_base_frame, root_base_frame, tf2::TimePointZero);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_ERROR(LOGGER, "%s", ex.what());
		return false;
	}

	// convert global link transform obtained to the fixed base frame of reference
	Eigen::Isometry3d goal_pose_tf2;
	tf2::doTransform(goal_pose, goal_pose_tf2, tf_base_footprint_msg); // in, out, transform

	// use rviz visual tools to publish a coordinate axis corresponding to the goal pose defined
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	visual_tools->setBaseFrame(fixed_base_frame);
	visual_tools->publishAxisLabeled(goal_pose_tf2, "search_pose");
	visual_tools->trigger();

	// make several attempts at planning until a valid motion is found or the maximum number of retries is reached
	int attempt = 0;
	valid_motion = false;
	moveit::planning_interface::MoveGroupInterface::Plan search_plan;
	moveit::core::MoveItErrorCode response;
	while (attempt < 3 && !valid_motion) {
		// attempt at planning and moving to the joint space goal
		response = move_group->plan(search_plan);
		valid_motion = bool(response);
		attempt++;
	}

	// visualizing the trajectory
	RCLCPP_INFO(LOGGER, "Planning to the searching position = %s", moveit::core::error_code_to_string(response).c_str());

	visual_tools->setBaseFrame(root_base_frame);
	visual_tools->publishTrajectoryLine(search_plan.trajectory, goal_state.getLinkModel(end_effector_link), joint_model_group);
	visual_tools->trigger();

	if (bool(response)) {
		RCLCPP_INFO(LOGGER, "Moving the robot to searching pose with joint space goal");
		move_group->execute(search_plan);
	} else {
		RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
	}

	return bool(response);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the PoseStampedSubscriberNode
	auto node = std::make_shared<ArucoToucher>(node_options);

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
