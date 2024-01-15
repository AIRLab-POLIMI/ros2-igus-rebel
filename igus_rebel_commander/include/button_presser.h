// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

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

// custom message definition for aruco markers
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

class ButtonPresser : public rclcpp::Node {
   private:
	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::button_presser");

	const std::string PLANNING_GROUP = "chain_arm_manipulator";

	std::string end_effector_link;

	const static int n_btns = 3;
	
	// aruco markers ids, from left to right
	int btn_1 = 4, btn_2 = 5, btn_3 = 6;

	// robot arm joint values for the looking pose
	std::vector<double> search_joints_positions = {1.0, -1.0, 1.0, 0.0, 1.74, 0.0};  // radians

	// aruco markers array (sorted)
	geometry_msgs::msg::Pose aruco_markers[n_btns]; 
	const int btn_ids[n_btns] = {btn_1, btn_2, btn_3};

	//position deltas between the aruco marker and the button (assuming sorted markers)
	const double x_delta[n_btns] = {0.0, 0.0, 0.0};
	const double y_delta[n_btns] = {0.0, 0.0, 0.0};
	const double z_delta[n_btns] = {0.0, 0.0, 0.0};

	// goal pose subscriber
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_sub;

	// thread for tracking the goal pose
	std::thread button_presser_demo_thread;

	// ready when aruco markers have been collected and the demo can start
	bool ready;

	// planning and moving utilities
	moveit::planning_interface::MoveGroupInterface* move_group;
	const moveit::core::JointModelGroup* joint_model_group;
	planning_interface::PlannerManagerPtr planner_instance;
	planning_scene::PlanningScene* planning_scene;

	// this node
	std::shared_ptr<rclcpp::Node> button_presser_node;

	// rviz visual tools
	moveit_visual_tools::MoveItVisualTools* visual_tools;

   public:
	ButtonPresser(const rclcpp::NodeOptions& node_options);

	// initialize moveit planner and other utilities
	void initPlanner(void);
	// load moveit planner given the robot model loaded
	void loadPlanner(const moveit::core::RobotModelPtr& robot_model);
	// initialize rviz visual tools for text and markers visualization
	void initRvizVisualTools(void);

	// callback for the goal pose subscriber from the aruco marker pose tracker publisher
	// saves the markers array and sorts them left to right
	void arucoMarkersCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array);

	// main thread to press the buttons demonstration
	void buttonPresserDemoThread(void);

	// this function may be substitured by one making a series of movements in order to find the buttons
	// void searchForButtonSetup(void);

	// move to predefined searching pose: robot looking at the buttons setup
	void moveToSearchingPose(void);

	// move to looking pose: robot facing the buttons setup after recognizing it
	// looking pose is the pose where the robot is facing the buttons setup, from above it or in front of it
	void moveToLookingPose();

	// compute the looking pose: positioning along the z-axis such that the robot faces the buttons setup from a distance
	geometry_msgs::msg::Pose::SharedPtr computeLookingPose(void);

	// compute the pose just above the button before pressing it
	geometry_msgs::msg::Pose::SharedPtr getPoseAboveButton(const geometry_msgs::msg::Pose::SharedPtr aruco_pose, const int button_id);

	// compute the pose to reach when pressing the button
	geometry_msgs::msg::Pose::SharedPtr getPosePressingButton(const geometry_msgs::msg::Pose::SharedPtr pose_above_button, const int button_id);

	// descent to button pose: robot moves in a straight line to the button and presses it (from pose_above_button to pose_pressing_button)
	void descentMovement(const geometry_msgs::msg::Pose::SharedPtr pose_pressing_button, const int button_id);

	// ascent to looking pose: robot moves in a straight line back to the looking pose (from pose_pressing_button to pose_above_button)
	void ascentMovement(const geometry_msgs::msg::Pose::SharedPtr pose_above_button, const int button_id);
};