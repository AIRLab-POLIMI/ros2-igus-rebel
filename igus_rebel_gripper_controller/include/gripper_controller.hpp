

#ifndef IGUS_REBEL_GRIPPER_CONTROLLER_HPP_
#define IGUS_REBEL_GRIPPER_CONTROLLER_HPP_

// C++ imports
#include <string>
#include <vector>
#include <fstream>

// C imports for termios library
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ROS2-Control imports
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

// custom service definition
#include "igus_rebel_gripper_controller/srv/gripper_actuation.hpp"

namespace igus_rebel_gripper_controller {

class GripperController : public hardware_interface::ActuatorInterface {

public:
	GripperController();

	hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

	hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces(void) override;

	std::vector<hardware_interface::CommandInterface> export_command_interfaces(void) override;

	hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

	hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

	hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	enum class GripperStates { GRIPPED, RELEASED, OFF };

private:
	
	const std::vector<std::string> gripper_states = { "grip", "release", "off" };

	rclcpp::Service<igus_rebel_gripper_controller::srv::GripperActuation>::SharedPtr grip_service_;

	rclcpp::Node::SharedPtr node_;

	/**
	 * @brief Callback function for the gripper actuation service "/gripper_actuate"
	 * @param request the request message containing the desired gripper state
	 * @param response the response message containing the current gripper state
	*/
	void grip_service_callback(const std::shared_ptr<igus_rebel_gripper_controller::srv::GripperActuation::Request> request,
							   const std::shared_ptr<igus_rebel_gripper_controller::srv::GripperActuation::Response> response);

	/**
	 * @brief Setup the serial communication via UART to the arduino controller
	 * @return true if the serial communication was successfully setup, false otherwise
	 */
	bool setup_serial_comm(void);

	rclcpp::Logger logger_;

	// Serial port configuration (UART)
	std::string serial_port; // serial port to the arduino controller
	int baud_rate;			 // baud rate for the serial port
	int serial_fd;			 // file descriptor for the serial port

	// Store the command and state for the soft gripper
	double *hw_grip_cmd;
	double *hw_grip_state;

	int grip_actuation_time;

	// Store the command and state string equivalent for the soft gripper
	std::string grip_cmd;
	std::string grip_state;

	// GPIO names in the ros2 control hardware interface
	const std::string GPIO_CMD = "grip_cmd";
	const std::string GPIO_STATE = "grip_state";

	const std::string service_topic_ = "/gripper_actuate";
	std::thread node_spin_thread_;
	
};

} // namespace igus_rebel_gripper_controller

#endif // IGUS_REBEL_GRIPPER_CONTROLLER_HPP_