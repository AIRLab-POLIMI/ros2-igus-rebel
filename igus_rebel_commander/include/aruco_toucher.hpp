// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
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


class ArucoToucher : public rclcpp::Node {
private:
	const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::aruco_toucher");

	const std::string aruco_topic = "/aruco/poses";

	// vector of double values for the joint position of the parked group state value
	std::vector<double> parked_joint_positions;

    const std::vector<double> search_joint_positions = {60.0*M_PI/180.0, -60.0*M_PI/180.0, 70.0*M_PI/180.0, 0.0, 90.0 * M_PI/180.0, 0.0}; // radians

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
	std::thread aruco_toucher_thread_;

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
	std::shared_ptr<rclcpp::Node> aruco_toucher_node;

	// rviz visual tools
	moveit_visual_tools::MoveItVisualTools *visual_tools;

public:
	ArucoToucher(const rclcpp::NodeOptions &node_options);

	// initialize moveit planner and other utilities
	void initPlanner(void);
	// load moveit planner given the robot model loaded
	void loadPlanner(const moveit::core::RobotModelPtr &robot_model);
	// initialize rviz visual tools for text and markers visualization
	void initRvizVisualTools(void);

	// callback for the goal pose subscriber from the aruco marker pose tracker publisher
	void aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

	// thread for tracking the goal pose and moving the robot to the goal pose
	void aruco_toucher_thread(void);

	/**
	 * @param target_pose the cartesian pose target with reference frame associated
	 * @return result of the movement
	 * @brief Plan and move the robot to the target pose
	 */
	bool robotPlanAndMove(geometry_msgs::msg::PoseStamped::SharedPtr target_pose);

	/**
	 * @brief Plan and move the robot to the joint space goal
	 * @param joint_space_goal the joint space goal, sequence of 6 joint values expressed in radians
	 * @return true if plan and movement were successful, false otherwise
	 */
	bool robotPlanAndMove(std::vector<double> joint_space_goal);
};