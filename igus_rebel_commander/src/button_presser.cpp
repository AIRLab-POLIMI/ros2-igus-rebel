// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

#include "button_presser.h"

ButtonPresser::ButtonPresser(const rclcpp::NodeOptions &node_options) : Node("button_presser_node", node_options) {
	RCLCPP_INFO(LOGGER, "Starting button presser demo");

	// aruco goal pose subscriber with callback thread
	aruco_markers_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco_markers", 10, std::bind(&ButtonPresser::arucoMarkersCallback, this, std::placeholders::_1));

	ready = false;

	aruco_markers = std::vector<geometry_msgs::msg::Pose::SharedPtr>(n_btns);

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
	fixed_base_frame = "igus_rebel_base_link";

	// track the goal pose asynchrounously
	// button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, this);
	// button_presser_demo_thread.detach();
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
	const moveit::core::RobotModelPtr &robot_model = robot_model_loader->getModel();

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

void ButtonPresser::loadPlanner(const moveit::core::RobotModelPtr &robot_model) {
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
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
	}
	try {
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, button_presser_node, button_presser_node->get_namespace())) {
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
	move_group->setGoalPositionTolerance(0.002);   // 1 mm
	move_group->setGoalOrientationTolerance(0.01); // 0.01 rad

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

/**
 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
 * @brief Callback function for the aruco markers subscriber
 */
void ButtonPresser::arucoMarkersCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array) {
	// to be run until the aruco markers are detected and sorted -> demo ready
	if (!ready) {
		// first check if the markers have all been detected
		if (aruco_markers_array->poses.size() != n_btns) {
			return; // skip and wait until all markers are detected
		}

		// then check whether the markers have the correct ids
		for (int i = 0; i < n_btns; i++) {
			if (aruco_markers_array->marker_ids[i] != btn_1 &&
				aruco_markers_array->marker_ids[i] != btn_2 &&
				aruco_markers_array->marker_ids[i] != btn_3) {
				return; // skip and wait until all markers are detected
			}
		}

		// transform the poses of the aruco markers with respect to the fixed base frame of reference
		geometry_msgs::msg::TransformStamped tf_camera_base_msg;
		try {
			tf_camera_base_msg = tf_buffer_->lookupTransform(fixed_base_frame, camera_frame_name, tf2::TimePointZero);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_ERROR(LOGGER, "%s", ex.what());
			return;
		}

		geometry_msgs::msg::Pose aruco_marker_tf_pose;
		// assign the markers to the correct array position based on their id
		for (int i = 0; i < n_btns; i++) {
			for (int j = 0; j < n_btns; j++) {
				if (aruco_markers_array->marker_ids[j] == btn_ids[i]) {
					// transform the aruco poses to the fixed base frame of reference
					tf2::doTransform(aruco_markers_array->poses[j], aruco_marker_tf_pose, tf_camera_base_msg);
					aruco_markers[i] = std::make_shared<geometry_msgs::msg::Pose>(aruco_marker_tf_pose);
					break;
				}
			}
		}

		// set the ready flag after successful button setup detection
		ready = true;
		RCLCPP_INFO(LOGGER, "Button setup detected, demo can start");
	}
}

/**
 * @brief Main thread function for the button presser demo
 */
void ButtonPresser::buttonPresserDemoThread() {
	RCLCPP_INFO(LOGGER, "Starting button presser demo thread");
	while (rclcpp::ok()) {
		if (ready) { // starts the demo
			RCLCPP_INFO(LOGGER, "Moving to looking position");
			// first move to the predefined looking pose
			geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = this->computeLookingPose();
			this->robotPlanAndMove(looking_pose, false);

			// then press the buttons in order
			RCLCPP_INFO(LOGGER, "Pressing button 1 ...");
			// button 1
			geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_1 = this->getPoseAboveButton(1);
			this->robotPlanAndMove(pose_above_button_1, false);

			// descent to press the button
			geometry_msgs::msg::PoseStamped::SharedPtr pose_pressing_button_1 = this->getPosePressingButton(
				std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_1->pose), 1);
			this->robotPlanAndMove(pose_pressing_button_1, true);

			// ascent to release the button
			this->robotPlanAndMove(pose_above_button_1, true);

			// move back to the looking pose
			//RCLCPP_INFO(LOGGER, "Returning to looking pose");
			//this->robotPlanAndMove(looking_pose, false);

			// button 2
			RCLCPP_INFO(LOGGER, "Pressing button 2 ...");
			geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_2 = this->getPoseAboveButton(2);
			this->robotPlanAndMove(pose_above_button_2, false);

			// descent to press the button
			geometry_msgs::msg::PoseStamped::SharedPtr pose_pressing_button_2 = this->getPosePressingButton(
				std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_2->pose), 2);
			this->robotPlanAndMove(pose_pressing_button_2, true);

			// ascent to release the button
			this->robotPlanAndMove(pose_above_button_2, true);

			// move back to the looking pose
			//RCLCPP_INFO(LOGGER, "Returning to looking pose");
			//this->robotPlanAndMove(looking_pose, false);

			// button 3
			RCLCPP_INFO(LOGGER, "Pressing button 3 ...");
			geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button_3 = this->getPoseAboveButton(3);
			this->robotPlanAndMove(pose_above_button_3, false);

			// descent to press the button
			geometry_msgs::msg::PoseStamped::SharedPtr pose_pressing_button_3 = this->getPosePressingButton(
				std::make_shared<geometry_msgs::msg::Pose>(pose_above_button_3->pose), 3);
			this->robotPlanAndMove(pose_pressing_button_3, true);

			// ascent to release the button
			this->robotPlanAndMove(pose_above_button_3, true);

			// move back to the looking pose
			RCLCPP_INFO(LOGGER, "Returning to looking pose and ending demo");
			this->robotPlanAndMove(looking_pose, false);

			// end of demo

			break;
		} else {
			// wait until it's ready
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			continue;
		}
	}
	RCLCPP_INFO(LOGGER, "Ending button presser demo thread");
}

/**
 * @brief the looking pose: positioning along the z-axis such that the robot faces the buttons setup from a distance
 * @return the looking pose
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::computeLookingPose() {
	// apply transform on the aruco marker in the middle, with a flip rotation to point towards it
	geometry_msgs::msg::PoseStamped::SharedPtr looking_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
	looking_pose->pose = *this->apply_transform(aruco_markers[1], delta_x[0], delta_y[0], delta_z[0], true);
	looking_pose->header.frame_id = fixed_base_frame;
	looking_pose->header.stamp = this->now();
	return looking_pose;
}

/**
 * @brief compute the pose just above the button before pressing it
 * @param button_id the number of the button to press - 1, 2, 3
 * @return the pose just above the button before pressing it
 */
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::getPoseAboveButton(const int button_id) {
	// apply transform from the aruco marker to the button, with a flip rotation to point towards it
	geometry_msgs::msg::PoseStamped::SharedPtr pose_above_button = std::make_shared<geometry_msgs::msg::PoseStamped>();
	pose_above_button->pose = *this->apply_transform(
		aruco_markers[button_id - 1], delta_x[button_id], delta_y[button_id], delta_z[button_id], true);
	pose_above_button->header.frame_id = fixed_base_frame;
	pose_above_button->header.stamp = this->now();
	return pose_above_button;
}

/**
 * @brief compute the pose to reach when pressing the button (button pressed)
 * @param pose_above_button the pose just above the button before pressing it
 * @param button_id the number of the button to press - 1, 2, 3
 * @return the pose to reach when pressing the button (button pressed)
*/
geometry_msgs::msg::PoseStamped::SharedPtr ButtonPresser::getPosePressingButton(
	const geometry_msgs::msg::Pose::SharedPtr pose_above_button, const int button_id) {
	// apply transform from the pose above the button to the pose pressing the button
	// changes only the x coordinate, without any rotation changes, straight movement
	geometry_msgs::msg::PoseStamped::SharedPtr pose_pressing_button = std::make_shared<geometry_msgs::msg::PoseStamped>();
	pose_pressing_button->pose = *this->apply_transform(pose_above_button, delta_pressing[button_id - 1], 0.0, 0.0, false);
	pose_pressing_button->header.frame_id = fixed_base_frame;
	pose_pressing_button->header.stamp = this->now();
	return pose_pressing_button;
}

/**
 * @param pose the pose of the aruco marker or a button
 * @param delta_x the delta x to apply to the pose
 * @param delta_y the delta y to apply to the pose
 * @param delta_z the delta z to apply to the pose
 * @param flip whether to apply a rotation of 180 degrees around the y axis or not
 * @brief Apply a transform to the pose of the aruco marker or a button
 */
geometry_msgs::msg::Pose::UniquePtr ButtonPresser::apply_transform(geometry_msgs::msg::Pose::SharedPtr pose,
																   float delta_x, float delta_y, float delta_z,
																   bool flip) {

	// create tf2 transform from given pose
	tf2::Transform pose_tf2;
	pose_tf2.setOrigin(tf2::Vector3(pose->position.x, pose->position.y, pose->position.z));
	pose_tf2.setRotation(tf2::Quaternion(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w));

	// create tf2 transform from delta pose
	tf2::Transform delta_tf2;
	delta_tf2.setOrigin(tf2::Vector3(delta_x, delta_y, delta_z));

	// compute button pose as the composition of the aruco pose and the delta pose
	tf2::Transform computed_tf2;

	if (flip) {
		// apply a rotation of 180 degrees around the y axis
		delta_tf2.setRotation(flip_rotation);
		computed_tf2 = pose_tf2 * delta_tf2;
		computed_tf2.setRotation(computed_tf2.getRotation() * extra_rotation);
	} else {
		// apply identity rotation quaternion
		tf2::Quaternion identity_rotation(0.0, 0.0, 0.0, 1.0);
		delta_tf2.setRotation(identity_rotation);
		computed_tf2 = pose_tf2 * delta_tf2;
	}

	// convert geometry_msgs::msg::Transform to geometry_msgs::msg::Pose
	geometry_msgs::msg::Pose::UniquePtr transformed_pose = std::make_unique<geometry_msgs::msg::Pose>();
	transformed_pose->position.x = computed_tf2.getOrigin().getX();
	transformed_pose->position.y = computed_tf2.getOrigin().getY();
	transformed_pose->position.z = computed_tf2.getOrigin().getZ();
	transformed_pose->orientation.x = computed_tf2.getRotation().getX();
	transformed_pose->orientation.y = computed_tf2.getRotation().getY();
	transformed_pose->orientation.z = computed_tf2.getRotation().getZ();
	transformed_pose->orientation.w = computed_tf2.getRotation().getW();

	return transformed_pose;
}

/**
 * @param target_pose the cartesian pose target with reference frame associated
 * @return result of the movement
 * @brief Plan and move the robot to the target pose
 */
bool ButtonPresser::robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose, bool planar_movement) {
	RCLCPP_INFO(LOGGER, "Planning and moving to target pose");

	// print out the target coordinates and orientation
	RCLCPP_INFO(LOGGER, "Target pose coordinates: x = %f, y = %f, z = %f", target_pose->pose.position.x, target_pose->pose.position.y, target_pose->pose.position.z);
	RCLCPP_INFO(LOGGER, "Target pose orientation: x = %f, y = %f, z = %f, w = %f", target_pose->pose.orientation.x, target_pose->pose.orientation.y, target_pose->pose.orientation.z, target_pose->pose.orientation.w);

	// publish a coordinate axis corresponding to the pose with rviz visual tools
	visual_tools->publishAxisLabeled(target_pose->pose, "target");
	visual_tools->trigger();

	// set the target pose
	move_group->setStartState(*move_group->getCurrentState());
	move_group->setGoalPositionTolerance(0.002);   // 2 mm
	move_group->setGoalOrientationTolerance(0.05); // 0.05 rad
	move_group->setPoseTarget(*target_pose, end_effector_link);
	move_group->setPlannerId("RRTConnectkConfigDefault");

	// optionally limit accelerations and velocity scaling
	move_group->setMaxVelocityScalingFactor(0.2);
	move_group->setMaxAccelerationScalingFactor(0.2);
	// move_group->setPlanningPipelineId("ompl");

    // set orientation constraint during movement to make fixed end effector orientation
	if (planar_movement) { 
		moveit_msgs::msg::OrientationConstraint constrained_orientation;
		constrained_orientation.link_name = end_effector_link;
		constrained_orientation.header.frame_id = fixed_base_frame;
		// maintain same orientation as the target pose
		constrained_orientation.orientation = target_pose->pose.orientation;
		constrained_orientation.absolute_x_axis_tolerance = 0.5;
		constrained_orientation.absolute_y_axis_tolerance = 0.5;
		constrained_orientation.absolute_z_axis_tolerance = 0.5;
		constrained_orientation.weight = 1.0;
		moveit_msgs::msg::Constraints constrained_endeffector;
		constrained_endeffector.orientation_constraints.push_back(constrained_orientation);
		move_group->setPathConstraints(constrained_endeffector);
	} else {
        move_group->clearPathConstraints();
    }

	// create plan for reaching the goal pose
	moveit::planning_interface::MoveGroupInterface::Plan plan_motion;
	moveit::core::MoveItErrorCode response = move_group->plan(plan_motion);

	// show output of planned movement
	RCLCPP_INFO(LOGGER, "Plan result = %s", moveit::core::error_code_to_string(response).c_str());

	// retrieve trajectory computed by the planner
	moveit_msgs::msg::RobotTrajectory trajectory = plan_motion.trajectory;

	// visualizing the trajectory
	visual_tools->publishTrajectoryLine(trajectory, joint_model_group);
	visual_tools->trigger();

	if (bool(response)) { // if the plan was successful
		RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
		move_group->execute(plan_motion);
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

	// wait 5 seconds so that the robot does not move instantly
	// std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	// move to the predefined static searching pose
	// alternatively start waving the robot arm to find the buttons setup
	node->moveToSearchingPose();

	// start the demo thread once the robot is in the searching pose
	std::thread button_presser_demo_thread = std::thread(&ButtonPresser::buttonPresserDemoThread, node);
	button_presser_demo_thread.detach();

	main_thread->join();
	button_presser_demo_thread.join();

	// Cleanup and shutdown
	rclcpp::shutdown();
	return 0;
}
