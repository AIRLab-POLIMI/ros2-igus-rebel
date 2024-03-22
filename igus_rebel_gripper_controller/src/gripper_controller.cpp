
#include "gripper_controller.hpp"

namespace igus_rebel_gripper_controller {

hardware_interface::CallbackReturn GripperController::on_init(const hardware_interface::HardwareInfo &info) {
	if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
		return hardware_interface::CallbackReturn::ERROR;
	}

	// GripperController has exactly one state and command interface for the GPIO
	const hardware_interface::ComponentInfo &gpio = info_.gpios[0];
	
	if (gpio.command_interfaces.size() != 1) {
		RCLCPP_ERROR(logger_, "GPIO '%s' has %zu command interfaces found. 1 expected.",
					 gpio.name.c_str(), gpio.command_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.command_interfaces[0].name != GPIO_CMD) {
		RCLCPP_ERROR(logger_, "Joint '%s' have %s command interfaces found. Only '%s' expected.",
					 gpio.name.c_str(), gpio.command_interfaces[0].name.c_str(), GPIO_CMD);
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.state_interfaces.size() != 1) {
		RCLCPP_ERROR(logger_, "GPIO '%s' has %zu state interface. 1 expected.",
					 gpio.name.c_str(), gpio.state_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.state_interfaces[0].name != GPIO_STATE) {
		RCLCPP_ERROR(logger_, "GPIO '%s' have %s state interface. Only '%s' expected.",
					 gpio.name.c_str(), gpio.state_interfaces[0].name.c_str(), GPIO_STATE);
		return hardware_interface::CallbackReturn::ERROR;
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> GripperController::export_state_interfaces() {
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(hardware_interface::StateInterface(
		info_.gpios[0].name, GPIO_STATE, &hw_grip_state));

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperController::export_command_interfaces() {
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	command_interfaces.emplace_back(hardware_interface::CommandInterface(
		info_.gpios[0].name, GPIO_CMD, &hw_grip_cmd));

	return command_interfaces;
}

hardware_interface::CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

	return hardware_interface::return_type::OK;
}

} // namespace igus_rebel_gripper_controller

PLUGINLIB_EXPORT_CLASS(igus_rebel_gripper_controller::GripperController, hardware_interface::ActuatorInterface)