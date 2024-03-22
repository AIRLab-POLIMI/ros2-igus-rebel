

#ifndef IGUS_REBEL_GRIPPER_CONTROLLER_HPP_
#define IGUS_REBEL_GRIPPER_CONTROLLER_HPP_

// C++ imports
#include <string>
#include <vector>

// ROS2-Control imports
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace igus_rebel_gripper_controller {

class GripperController : public hardware_interface::ActuatorInterface {

public:
	hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

	LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces(void) override;

	std::vector<hardware_interface::CommandInterface> export_command_interfaces(void) override;

	LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

	LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

	hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
	// Store the command and state for the soft gripper
	double hw_grip_cmd;
	double hw_grip_state;

	rclcpp::Logger logger_ = rclcpp::get_logger("igus_rebel::gripper_controller");

	const std::string GPIO_CMD = "grip_cmd";
	const std::string GPIO_STATE = "grip_state";
};

} // namespace igus_rebel_gripper_controller

#endif // IGUS_REBEL_GRIPPER_CONTROLLER_HPP_