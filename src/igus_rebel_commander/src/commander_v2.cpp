/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Mike Lautman*/

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::commander_v2");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto commander_node = rclcpp::Node::make_shared("commander_v2_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(commander_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    RCLCPP_INFO(LOGGER, "Starting Commander V2");

    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
    // a RobotModel and a PlanningScene.
    //
    // We will start by instantiating a
    // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
    // object, which will look up the robot description on the ROS
    // parameter server and construct a
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
    // for us to use.
    const std::string PLANNING_GROUP = "rebel_arm";
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader(commander_node, "robot_description"));

    // Using the RobotModelLoader, we can construct a planning scene monitor that
    // will create a planning scene, monitors planning scene diffs, and apply the diffs to it's
    // internal planning scene. We then call startSceneMonitor, startWorldGeometryMonitor and
    // startStateMonitor to fully initialize the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(commander_node, robot_model_loader));

    /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                         the internal planning scene accordingly */
    psm->startSceneMonitor();
    /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                                  world geometry, collision objects and optionally octomaps */
    psm->startWorldGeometryMonitor();
    /* listen to joint state updates as well as changes in attached collision objects
                          and update the internal planning scene accordingly*/
    psm->startStateMonitor();

    /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
       for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
       RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
    moveit::core::RobotStatePtr robot_state(
        new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
       group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    /* // instantiating a planning pipeline doesn't work this way --> it selects CHOMP instead of OMPL
    // We can now setup the PlanningPipeline object, which will use the ROS parameter server
    // to determine the set of request adapters and the planning plugin to use
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
            new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
    */

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!commander_node->get_parameter("planning_plugin", planner_plugin_name))
        RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, commander_node, commander_node->get_namespace())) {
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

    moveit::planning_interface::MoveGroupInterface move_group(commander_node, PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    // @param node - base frame - markers topic - robot model
    moveit_visual_tools::MoveItVisualTools visual_tools(commander_node, "base_link", "commander_v2", psm);

    // extra options
    visual_tools.setPlanningSceneTopic("/move_group/monitored_planning_scene");
    visual_tools.loadPlanningSceneMonitor();
    visual_tools.enableBatchPublishing();

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanRequest req;
    req.pipeline_id = "ompl";
    req.planner_id = "RRTConnectkConfigDefault";
    req.allowed_planning_time = 1.0;
    req.group_name = PLANNING_GROUP;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.2229;
    pose.pose.position.y = 0.3659;
    pose.pose.position.z = 0.4955;
    pose.pose.orientation.x = 0.9593;
    pose.pose.orientation.y = -0.0239;
    pose.pose.orientation.z = 0.2815;
    pose.pose.orientation.w = -0.007;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.1);
    std::vector<double> tolerance_angle(3, 0.1);

    // We will create the request as a constraint using a helper
    // function available from the
    // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
    // package.

    moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    /*{
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        // Now, call the pipeline and check whether planning was successful.
        // Check that the planning was successful
        if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
            rclcpp::shutdown();
            return -1;
        }
    }*/

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    {  // read-lock acquisition on the planning scene
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(lscene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
            rclcpp::shutdown();
            return -1;
        } else {
            RCLCPP_INFO(LOGGER, "Plan successfully computed");
        }
    }

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
        commander_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher->publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

	

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

    // Now, setup a joint space goal
    moveit::core::RobotState goal_state(*robot_state);
    std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0}; // radians
    goal_state.setJointGroupPositions(joint_model_group, joint_values); 
    moveit_msgs::msg::Constraints joint_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    { // read-lock acquisition on the planning scene
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);

        // Now, call the pipeline and check whether planning was successful.
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(lscene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
            rclcpp::shutdown();
            return -1;
        } else {
            RCLCPP_INFO(LOGGER, "Plan successfully computed");
        }
    }

    /* Visualize the trajectory */
    RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    // Now you should see two planned trajectories in series
    display_publisher->publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Using a Planning Request Adapter
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // A planning request adapter allows us to specify a series of operations that
    // should happen either before planning takes place or after the planning
    // has been done on the resultant path

    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

    // Now, set one of the joints slightly outside its upper limit
    const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("joint3");
    const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
    robot_state->setJointPositions(joint_model, tmp_values);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        // Now, call the pipeline and check whether planning was successful.
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(lscene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
            rclcpp::shutdown();
            return -1;
        } else {
            RCLCPP_INFO(LOGGER, "Plan successfully computed");
        }
    }


    /* Visualize the trajectory */
    RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    /* Now you should see three planned trajectories in series*/
    display_publisher->publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

    RCLCPP_INFO(LOGGER, "Done");

    rclcpp::shutdown();
    return 0;
}