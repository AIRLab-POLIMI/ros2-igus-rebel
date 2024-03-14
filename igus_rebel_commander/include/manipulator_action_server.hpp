// ROS2 includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <aruco_interfaces/msg/aruco_markers.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// custom action msg 
#include "igus_rebel_commander/action/move_manipulator.hpp"

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// MoveIt2 imports
// #include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++ libraries includes
#include <cmath>
#include <thread>
#include <set>
#include <cstdint>

class ManipulatorActionServer : public rclcpp::Node {

public:
    using Manipulator = igus_rebel_commander::action::MoveManipulator;
    using GoalHandleManipulator = rclcpp_action::ServerGoalHandle<Manipulator>;

    // constructor
	ManipulatorActionServer(const rclcpp::NodeOptions & options);

    // initialize moveit planner and other utilities
	void initPlanner(void);
	// load moveit planner given the robot model loaded
	void loadPlanner(const moveit::core::RobotModelPtr &robot_model);
	// initialize rviz visual tools for text and markers visualization
	void initRvizVisualTools(void);

	/**
	 * @brief Plan and move the robot to the joint space goal
	 * @param joint_space_goal the joint space goal, sequence of 6 joint values expressed in radians
	 * @return true if plan and movement were successful, false otherwise
	 */
	bool robotPlanAndMove(std::vector<double> joint_space_goal);

private:
    rclcpp_action::Server<Manipulator>::SharedPtr action_server_;

	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::aruco_toucher");

	const std::string aruco_topic = "/aruco/poses";

	// vector of double values for the joint position of the parked group state value
	std::vector<double> parked_joint_positions;

	// vector of double values for the joint position of the stand group state value
	std::vector<double> aruco_stand_joint_positions;

	// vector of double values for the target joint position used to complete the action
	std::vector<double> target;

    const std::vector<double> search_joint_positions = {M_PI/3.0, -M_PI_2, 40.0*M_PI/180.0, 0.0, 100.0 * M_PI/180.0, 0.0}; // radians

	// tf2 listener and buffer for frame transformations
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	// goal pose stamped
	geometry_msgs::msg::PoseStamped::SharedPtr goal_pose;
	std::mutex goal_pose_mutex;
	geometry_msgs::msg::PoseStamped::SharedPtr current_goal_pose;
	bool ready = false;

	// goal pose subscriber
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub;

	// thread for tracking the goal pose
	std::thread manipulator_thread_;

	const std::string PLANNING_GROUP = "rebel_arm";

	std::string end_effector_link; // toucher_endpoint

	// the source frame of the aruco markers is the camera frame
	std::string camera_frame_name;
	// planning and frame transformations are done in the fixed base frame
	const std::string fixed_base_frame = "igus_rebel_base_link";
	std::string root_base_frame; // base footprint or map

	// planning and moving utilities
	moveit::planning_interface::MoveGroupInterface *move_group;
	const moveit::core::JointModelGroup *joint_model_group;
	planning_interface::PlannerManagerPtr planner_instance;
	planning_scene::PlanningScene *planning_scene;
	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;

	// this node
	std::shared_ptr<rclcpp::Node> manipulator_node;

	// rviz visual tools
	moveit_visual_tools::MoveItVisualTools *visual_tools;
	
	// callback function for handling new goals
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Manipulator::Goal> goal);
	// callback function for dealing with cancellation
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleManipulator> goal_handle);
	// callback function that accepts a new goal and starts processing it
    void handle_accepted(const std::shared_ptr<GoalHandleManipulator> goal_handle);
	// function that execute the given goal
    void execute(const std::shared_ptr<GoalHandleManipulator> goal_handle);
};