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

class ArucoFollower : public rclcpp::Node {
   private:
    const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::aruco_follower");

    const std::string PLANNING_GROUP = "rebel_arm";

    std::string end_effector_link; // toucher_endpoint

    // goal pose stamped
    geometry_msgs::msg::PoseStamped goal_pose;
    std::mutex goal_pose_mutex;
    geometry_msgs::msg::PoseStamped current_goal_pose;

    // goal pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;

    // thread for tracking the goal pose
    std::thread track_goal_pose_thread;

    // planning and moving utilities

    moveit::planning_interface::MoveGroupInterface* move_group;
    const moveit::core::JointModelGroup* joint_model_group;
    planning_interface::PlannerManagerPtr planner_instance;
    planning_scene::PlanningScene* planning_scene;

    // this node
    std::shared_ptr<rclcpp::Node> follower_node;

    // rviz visual tools
    moveit_visual_tools::MoveItVisualTools* visual_tools;

   public:
    ArucoFollower(const rclcpp::NodeOptions& node_options);

	// initialize moveit planner and other utilities
    void initPlanner(void);
	// load moveit planner given the robot model loaded
    void loadPlanner(const moveit::core::RobotModelPtr& robot_model);
	// initialize rviz visual tools for text and markers visualization
    void initRvizVisualTools(void);

	// callback for the goal pose subscriber from the aruco marker pose tracker publisher
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// thread for tracking the goal pose and moving the robot to the goal pose
    void trackGoalPose(void);
};