// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// ROS2 C++ imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// button presser API code
#include "button_presser.hpp"

// custom action definition in mobile_manipulation_interfaces
#include "mobile_manipulation_interfaces/action/button_find.hpp"
#include "mobile_manipulation_interfaces/action/button_press.hpp"

namespace igus_rebel_commander_action_servers {

class ButtonPresserActionServers : public rclcpp::Node {

	// action sever types for ButtonPressAction
	using ButtonPressAction = mobile_manipulation_interfaces::action::ButtonPress;
	using GoalHandleButtonPress = rclcpp_action::ServerGoalHandle<ButtonPressAction>;

	// action sever types for ButtonFindAction
	using ButtonFindAction = mobile_manipulation_interfaces::action::ButtonFind;
	using GoalHandleButtonFind = rclcpp_action::ServerGoalHandle<ButtonFindAction>;

public:
	/**
	 * @brief Construct a new ButtonPresserActionServers object
	 * @param button_presser_api the button presser API object instantiated in the main function
	 * @param options the node options
	 */
	ButtonPresserActionServers(std::shared_ptr<ButtonPresser> &button_presser_api,
							  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	/**
	 * @brief handle goal request for the button press action server
	 * @param uuid the unique identifier of the goal request
	 * @param goal the goal request object
	 * @return rclcpp_action::GoalResponse the response to the goal request
	 */
	rclcpp_action::GoalResponse handle_press_goal(const rclcpp_action::GoalUUID &uuid,
												  std::shared_ptr<const ButtonPressAction::Goal> goal);

	/**
	 * @brief handle cancel request for the button press action server
	 * @param goal_handle the goal to be cancelled
	 * @return rclcpp_action::CancelResponse the response to the cancel request
	 */
	rclcpp_action::CancelResponse handle_press_cancel(const std::shared_ptr<GoalHandleButtonPress> goal_handle);

	/**
	 * @brief handle accepted request for the button press action server, starts a new thread to execute the goal request
	 * @param goal_handle the goal accepted and ready to be executed
	 */
	void handle_press_accepted(const std::shared_ptr<GoalHandleButtonPress> goal_handle);

	/**
	 * @brief execute callback for the button press action server
	 * @param goal_handle the goal to be executed
	 */
	void execute_press_callback(const std::shared_ptr<GoalHandleButtonPress> goal_handle);

	/**
	 * @brief handle goal request for the button finder action server
	 * @param uuid the unique identifier of the goal request
	 * @param goal the goal request object
	 * @return rclcpp_action::GoalResponse the response to the goal request
	 */
	rclcpp_action::GoalResponse handle_find_goal(const rclcpp_action::GoalUUID &uuid,
												 std::shared_ptr<const ButtonFindAction::Goal> goal);

	/**
	 * @brief handle cancel request for the button finder action server
	 * @param goal_handle the goal to be cancelled
	 * @return rclcpp_action::CancelResponse the response to the cancel request
	 */
	rclcpp_action::CancelResponse handle_find_cancel(const std::shared_ptr<GoalHandleButtonFind> goal_handle);

	/**
	 * @brief handle accepted request for the button finder action server, starts a new thread to execute the goal request
	 * @param goal_handle the goal accepted and ready to be executed
	 */
	void handle_find_accepted(const std::shared_ptr<GoalHandleButtonFind> goal_handle);

	/**
	 * @brief execute callback for the button finder action server
	 * @param goal_handle the goal to be executed
	 */
	void execute_find_callback(const std::shared_ptr<GoalHandleButtonFind> goal_handle);

	/**
	 * @brief look in the vicinity for aruco markers until they are found.
	 * move around the robot until the defined aruco markers in button presser api are found.
	 * once the markers are found, the robot will execute the button presser demonstration
	 * @param goal_handle the goal to be executed
	 */
	void lookAroundForArucoMarkers(const std::shared_ptr<GoalHandleButtonPress> goal_handle);

	/**
	 * @brief execute main thread to press the buttons demonstration, after aruco markers have been found
	 * @param goal_handle the goal to be executed
	 */
	void buttonPresserThread(const std::shared_ptr<GoalHandleButtonPress> goal_handle);

	/**
	 * @brief look in the surroundings for an aruco marker until it is found.
	 * rotate the robot along a single layer until it finds the defined placed marker in the robot's surroundings
	 * once the marker is found, the robot will move to the static parked position on the mobile robot
	 * @param goal_handle the goal to be executed
	 * @return geometry_msgs::msg::PoseStamped::SharedPtr the position of the found aruco marker
	 */
	geometry_msgs::msg::PoseStamped::SharedPtr lookAroundForArucoMarkers(
		const std::shared_ptr<GoalHandleButtonFind> goal_handle);

	/**
	 * @brief move the robot arm to the static predefined parked position, after the aruco marker has been found
	 * @param goal_handle the goal to be executed
	 */
	void moveToParkedPosition(const std::shared_ptr<GoalHandleButtonFind> goal_handle);

private:
	// action server object for ButtonPressAction
	rclcpp_action::Server<ButtonPressAction>::SharedPtr button_presser_action_server_;
	rclcpp_action::Server<ButtonFindAction>::SharedPtr button_finder_action_server_;

	// button presser API object
	std::shared_ptr<ButtonPresser> button_presser_api_;

	// logger
	const rclcpp::Logger LOGGER = rclcpp::get_logger("commander::button_presser");
};

} // namespace igus_rebel_commander_action_servers
