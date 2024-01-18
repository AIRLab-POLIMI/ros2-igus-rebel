// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// ROS2 imports
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt2 imports
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// custom message definition for aruco markers
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

// C++ imports
#include <cmath>
#include <thread>
#include <vector>

class ButtonPresser : public rclcpp::Node {
private:
	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::button_presser");

	const std::string PLANNING_GROUP = "chain_arm_manipulator";

	std::string end_effector_link;

	// the source frame of the aruco markers is the camera frame
	std::string camera_frame_name;
	std::string fixed_base_frame;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	const static int n_btns = 3;

	// aruco markers ids, from left to right
	const int btn_1 = 4, btn_2 = 5, btn_3 = 6;

	// robot arm joint values for the looking pose
	const std::vector<double> search_joints_positions = {1.0, -1.0, 1.0, 0.0, 1.74, 0.0}; // radians

	// aruco markers array (sorted)
	std::vector<geometry_msgs::msg::Pose::SharedPtr> aruco_markers; // one for each button
	const int btn_ids[n_btns] = {btn_1, btn_2, btn_3};

	// position deltas in meters between the aruco marker and the button (assuming sorted markers)
	//  in order: looking pose, button 1, button 2, button 3
	const float delta_x[n_btns + 1] = {0.0, 0.01, 0.01, 0.01};
	const float delta_y[n_btns + 1] = {0.1, -0.10, -0.09, -0.09};
	const float delta_z[n_btns + 1] = {0.15, 0.08, 0.08, 0.08};

	// position vertical axis delta required to go down and press the button (or release it)
	// in order: button 1, button 2, button 3
	const float delta_pressing[n_btns] = {0.07, 0.09, 0.07};

	// this quaternion describes a rotation of -pi/2 radians around the y axis.
	const tf2::Quaternion flip_rotation = tf2::Quaternion(std::cos(-M_PI_2 / 2.0), 0.0, std::sin(-M_PI_2 / 2.0), 0.0);
	const tf2::Quaternion extra_rotation = tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -M_PI_2);

	// goal pose subscriber
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_sub;

	// thread for tracking the goal pose
	std::thread button_presser_demo_thread;

	// ready when aruco markers have been collected and the demo can start
	bool ready;

	// planning and moving utilities
	moveit::planning_interface::MoveGroupInterface *move_group;
	const moveit::core::JointModelGroup *joint_model_group;
	planning_interface::PlannerManagerPtr planner_instance;
	planning_scene::PlanningScene *planning_scene;

	// this node
	std::shared_ptr<rclcpp::Node> button_presser_node;

	// rviz visual tools
	moveit_visual_tools::MoveItVisualTools *visual_tools;

public:
	ButtonPresser(const rclcpp::NodeOptions &node_options);

	// initialize moveit planner and other utilities
	void initPlanner(void);
	// load moveit planner given the robot model loaded
	void loadPlanner(const moveit::core::RobotModelPtr &robot_model);
	// initialize rviz visual tools for text and markers visualization
	void initRvizVisualTools(void);

	/**
	 * @param aruco_markers_array the array of aruco markers detected by the camera published on /aruco_markers
	 * @brief callback for the goal pose subscriber from the aruco marker pose tracker publisher
	 *        saves the markers array and sorts them left to right
	 */
	void arucoMarkersCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_markers_array);

	// main thread to press the buttons demonstration
	void buttonPresserDemoThread(void);

	// this function may be substitured by one making a series of movements in order to find the buttons
	// void searchForButtonSetup(void);

	/**
	 * @brief Move the robot to the static predefined searching pose
	 */
	void moveToSearchingPose(void);

	/**
	 * @brief the looking pose: positioning along the x-axis such that the robot faces the buttons setup from a distance
	 * @return the looking pose
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr computeLookingPose(void);

	/**
	 * @brief compute the pose just above the button before pressing it
	 * @param button_id the number of the button to press - 1, 2, 3
	 * @return the pose just above the button before pressing it
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr getPoseAboveButton(const int button_id);

	/**
	 * @brief compute the pose to reach when pressing the button (button pressed)
	 * @param pose_above_button the pose just above the button before pressing it
	 * @param button_id the number of the button to press - 1, 2, 3
	 * @return the pose to reach when pressing the button (button pressed)
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr getPosePressingButton(const geometry_msgs::msg::Pose::SharedPtr pose_above_button,
																	 const int button_id);

	/**
	 * @param pose the pose of the aruco marker or a button
	 * @param delta_x the delta x to apply to the pose
	 * @param delta_y the delta y to apply to the pose
	 * @param delta_z the delta z to apply to the pose
	 * @param flip whether to apply a rotation of 180 degrees around the y axis or not
	 * @brief Apply a transform to the pose of the aruco marker or a button
	 */
	geometry_msgs::msg::Pose::UniquePtr apply_transform(geometry_msgs::msg::Pose::SharedPtr pose,
														float delta_x, float delta_y, float delta_z,
														bool flip = false);

	/**
	 * @param target_pose the target pose for the robotic arm to reach
	 * @param planar_movement whether to fix end effector orientation during movement (with constrained planning)
	 * @brief Plan and move the robotic arm to the target pose
	 * @return true if the movement and planning was successful, false otherwise
	 */
	bool robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose, bool planar_movement = false);
};