
//  hardware interface definitions for ros2 control
//  https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

#ifndef REBEL_CONTROLLER_H
#define REBEL_CONTROLLER_H

// C++ imports
#include <math.h>
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <map>

// local imports
#include "cri_keywords.h"
#include "cri_messages.h"
#include "cri_socket.h"

// ROS2-Control imports
#include <hardware_interface/system_interface.hpp>  // system type interface
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

// namespace for encapsulating the hardware controller package
namespace igus_rebel_hw_controller {

// class extending the system interface according to ros2 control
class RebelController : public hardware_interface::SystemInterface {

   private:
    const std::string ip_address = "192.168.3.11";
    const int port = 3920;

    CriSocket cri_socket;

	// status message
    cri_messages::Status currentStatus;

    bool continueAlive;
    bool continueMessage;
    std::thread aliveThread;
    std::thread messageThread;
    int aliveWaitMs;

    int cmd_counter;
    std::mutex counterLock;
    std::mutex aliveLock;

	// code for computing the rate at which the STATUS messages are received
	int status_count = 0;
	std::thread statusFrequencyThread;
	void computeStatusFrequencyThread();

    // ROS2 controller input variables

    // Current jogs
    // 0-5 are used for internal axis
    // additional elements can be added for external axes control when more controllers are placed on the robot
    std::vector<double> jogs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // percentage of the maximum speed

    // encoder feedback values fed to the hardware_interface
    std::vector<double> position_feedback_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // [rad]
    std::vector<double> velocity_feedback_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // [rad/s]

    // command values given from the controller to the hardware_interface
    std::vector<double> cmd_position_;  // [rad]
    std::vector<double> cmd_last_position_; // [rad]
    std::vector<double> cmd_velocity_;  // [rad/s]
	std::vector<double> cmd_last_velocity_;  // [rad/s]

    // Used to counteract the offsets in EmbeddedCtrl, read from .ros2_control.xacro files
    std::vector<double> pos_offset_;  // [rad]

	// kinematic state and error messages
    cri_messages::Kinstate lastKinstate;
    std::array<int, 16> lastErrorJoints;
    std::string kinstateMessage;

	cri_messages::KinematicLimits kinematicLimits;

    // Thread functions
    void alivejogThread(void);
    void socketMessagesThread(void);

    // CRI protocol service functions
    int get_cmd_counter();
    void Command(const std::string&);
    void GetConfig(const std::string&);
    void GetReferenceInfo();

	// checks whether the input commands from the controller have changed since the last one received
	bool detect_change(std::vector<double> &v1, std::vector<double> &v2);

    // Function to react to specific status values, to display warnings, error messages, etc.
    void processStatus(const cri_messages::Status&);

	// Function to convert the jog values to the correct format for the robot controller
	// std::vector< std::pair< rclcpp::Time&, std::vector<double> > > position_feedbacks_log;
	void savePositionFeedback(const rclcpp::Time& time);

	const double move_velocity = 20.0; // [20 % max velocity]
	// NOTE: this constant is valid as long as the override value in the STATUS message = 80.0
	// otherwise if it's lower, then the robot will move slower than expected
    const double rads_to_jogs_ratio = 100.0 / (move_velocity / 100.0 * M_PI);
	const unsigned int n_joints = 6;

   public:
    RebelController(const std::string& ip, const int& port);
    RebelController(void);
    ~RebelController();

    // ROS2 Control functions override for defining the system hardware interface
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
	void shutdown(void);
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ROS2 Control functions override for reading and writing control variables
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& duration) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& duration) override;


};
}  // namespace igus_rebel_hw_controller

#endif
