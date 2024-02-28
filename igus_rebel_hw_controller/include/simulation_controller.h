
//  hardware interface definitions for ros2 control
//  https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

#ifndef SIMULATION_CONTROLLER_H
#define SIMULATION_CONTROLLER_H

#include <math.h>

#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp" // system type interface
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// namespace for encapsulating the hardware controller package
namespace igus_rebel_hw_controller {

// class extending the system interface according to ros2 control
class SimulationController : public hardware_interface::SystemInterface {

private:
	const unsigned int n_joints = 6;
	int aliveWaitMs;

	// ROS2 controller input variables
	const bool control_by_velocity = true;

	// Current jogs
	// 0-5 are used for internal axis
	// additional elements can be added for external axes control when more controllers are placed on the robot
	std::vector<double> jogs_ = {0, 0, 0, 0, 0, 0}; // percentage of the maximum speed

	// encoder feedback values fed to the hardware_interface
	std::vector<double> position_feedback_ = {0, 0, 0, 0, 0, 0}; // [rad]
	std::vector<double> velocity_feedback_ = {0, 0, 0, 0, 0, 0}; // [rad/s]

	// command values given from the controller to the hardware_interface
	std::vector<double> cmd_position_;		// [rad]
	std::vector<double> cmd_last_position_; // [rad]
	std::vector<double> cmd_velocity_;		// [rad/s]
	std::vector<double> cmd_last_velocity_; // [rad/s]

	// Used to counteract the offsets in EmbeddedCtrl, read from .ros2_control.xacro files
	std::vector<double> pos_offset_; // [rad]

	const double rads_to_jogs_ratio = 100.0 / (0.25 * M_PI);

public:
	SimulationController(void);
	~SimulationController();

	// ROS2 Control functions override for defining the system hardware interface
	hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
	hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
	void shutdown(void);
	hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	// ROS2 Control functions override for reading and writing control variables
	hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &duration) override;
	hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &duration) override;

	bool detect_change(std::vector<double> &v1, std::vector<double> &v2);
};
} // namespace igus_rebel_hw_controller

#endif
