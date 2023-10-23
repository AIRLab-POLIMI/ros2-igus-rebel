#include "fake_controller.h"

namespace igus_rebel_hw_controller {

/**
 * @brief Construct a new Rebel Controller object and initialize joints values
 * 	Uses the default ip address and port for the socket connection
 */
FakeController::FakeController() : aliveWaitMs(50),
                                   cmd_counter(1) {
}

// empty destructor because not needed
FakeController::~FakeController() {
}

/**
 **********************************************
 * ROS2 Controller for the hardware interfaces
 ***********************************************
 */

/**
 * @brief initialize all member variables and process the parameters from the info argument.
 *  This method overrides on_init from hardware_interface::SystemInterface.
 *  on_init is called immediately after the controller plugin is dynamically loaded. The method is
 * 	called only once during the lifetime for the controller, hence memory that exists for
 * 	the lifetime of the controller should be allocated. Additionally, the parameter values for
 * 	joints, command_interfaces and state_interfaces should be declared and accessed.
 * @param info contains specific information obtained from the URDF
 * @return CallbackReturn::SUCCESS If all required parameters are set and valid and everything works fine
 * 	or CallbackReturn::ERROR otherwise
 */
hardware_interface::CallbackReturn FakeController::on_init(const hardware_interface::HardwareInfo &info) {
    // In the first line usually the parents on_init is called to process standard values
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info.joints) {
        // Read the joint offset from the ros2_control configuration
        double cri_joint_offset = 0.0;
        if (joint.parameters.count("cri_joint_offset") > 0) {
            // convert string to double
            cri_joint_offset = stod(joint.parameters.at("cri_joint_offset"));
        } else {
            RCLCPP_WARN(rclcpp::get_logger("hw_controller::fake_controller"),
                        "No cri_joint_offset specified for joint %s, using default value of %lf", joint.name.c_str(), cri_joint_offset);
        }
        pos_offset_.push_back(cri_joint_offset);
    }

    // print command interfaces names
    for (const hardware_interface::InterfaceInfo &cmd_interface : info.joints.at(0).command_interfaces) {
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "Command interface found: %s", cmd_interface.name.c_str());
    }
    // print state interfaces names
    for (const hardware_interface::InterfaceInfo &state_interface : info.joints.at(0).state_interfaces) {
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "State interface found: %s", state_interface.name.c_str());
    }

    // initialize the vectors with NaN values
    for (unsigned int i = 0; i < n_joints; i++) {
        cmd_position_.push_back(std::numeric_limits<double>::quiet_NaN());
        cmd_last_position_.push_back(std::numeric_limits<double>::quiet_NaN());
        cmd_velocity_.push_back(std::numeric_limits<double>::quiet_NaN());
    }

    RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "Detected %lu joints in the urdf file", jogs_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief where hardware “power” is enabled. The on_activate is called once when the controller
 * 	is activated. This method should handle controller restarts, such as setting the resetting
 * 	reference to safe values. It should also perform controller specific safety checks.
 * @return CallbackReturn::SUCCESS if the controller was successfully activated
 * 	else CallbackReturn::FAILURE
 */
hardware_interface::CallbackReturn FakeController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief on_deactivate is called when a controller stops running. It is important to release the claimed command interface
 *  in this method, so other controllers can use them if needed. This method is called before the controller is shut down
 * @return CallbackReturn::SUCCESS if the controller was successfully activated
 * 	else CallbackReturn::FAILURE
 */
hardware_interface::CallbackReturn FakeController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    std::fill(jogs_.begin(), jogs_.end(), 0.0f);

    std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief defines the interfaces that the hardware components are offering
 * @return a vector of StateInterface, describing the state_interfaces for each joint.
 * 	The StateInterface objects are read only data handles. Their constructors require an interface name,
 *  interface type, and a pointer to a double data value.
 */
std::vector<hardware_interface::StateInterface> FakeController::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Add the position and velocity states for each joint defined
    for (unsigned int i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_feedback_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_feedback_[i]));
    }
    // NOTE: include here specific state_interfaces for additional controllers such as digital IO

    return state_interfaces;
}

/**
 * @brief defines the interfaces that the hardware components are offering. Similar to export_state_interfaces()
 * @return a vector of CommandInterface. The vector contains objects describing the command_interfaces for each joint.
 */
std::vector<hardware_interface::CommandInterface> FakeController::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Add the position and velocity command signals for each joint defined
    for (unsigned int i = 0; i < info_.joints.size(); i++) {
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]));

        // command only with velocity
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]));
    }

    // NOTE: include here specific state_interfaces for additional controllers such as digital IO

    return command_interfaces;
}

/**
 * @brief Reads the current joint positions and converts them to radians for ROS2.
 * 	getting the states from the hardware and storing them to internal variables defined in export_state_interfaces()
 * 	This is the core method in the ros2_control loop. During the main loop, ros2_control loops over all hardware components
 *  and calls the read method. It is executed on the realtime thread, hence the method must obey by realtime constraints.
 * 	The read method is responsible for updating the data values of the state_interfaces. Since the data value point to
 * 	class member variables, those values can be filled with their corresponding sensor values, which will in turn update
 * 	the values of each exported StateInterface object.
 * @return hardware_interface::return_type::OK if the read was successful
 */
hardware_interface::return_type FakeController::read(const rclcpp::Time &time, const rclcpp::Duration &duration) {
    /*
    // feedback for controller by position
for (unsigned int i = 0; i < n_joints; i++) {
            // derivate position to get estimated velocity vector
    velocity_feedback_[i] = (cmd_position_[i] - position_feedback_[i]) / duration.seconds();

    // position directly from the command vector (measured input = command)
    position_feedback_[i] = cmd_position_[i];
}
    */
    std::vector<double> temp_pos;
    temp_pos.reserve(n_joints);

    // feedback fror controller by velocity
    for (unsigned int i = 0; i < n_joints; i++) {
        // position directly from the command vector (measured input = command)
        temp_pos[i] = position_feedback_[i] + velocity_feedback_[i] * duration.seconds();
        if (std::isfinite(cmd_velocity_[i])) {
            velocity_feedback_[i] = cmd_velocity_[i];
        }

        position_feedback_[i] = temp_pos[i];
    }

    return hardware_interface::return_type::OK;
}

/**
 * @brief commands the hardware based on the values stored in internal variables defined in export_command_interfaces()
 * 	If we did not set a velocity we want the jog set to 0.0f and instead send a position command
 * 	If neither velocity nor position have been send yet do no movement and only send jog messages
 * 	with the velocity set to 0.0f.
 * 	It is called after update in the realtime loop. For this reason, it must also obey by realtime constraints.
 * 	The write method is responsible for updating the data values of the command_interfaces.
 * 	As opposed to read, write accesses data values pointed by the exported CommandInterface objects, and then the function
 * 	sends them to the corresponding hardware via the defined interface.
 * 	@return hardware_interface::return_type::OK if the write was successful
 */
hardware_interface::return_type FakeController::write(const rclcpp::Time &time, const rclcpp::Duration &duration) {
    // Make it possible to use different movement commands after another.

    // if all cmd_position_ and cmd_velocity_ are filled with zeros, return OK
    if (std::any_of(cmd_position_.begin(), cmd_position_.end(), [](double d) { return !std::isfinite(d); }) &&
        std::any_of(cmd_velocity_.begin(), cmd_velocity_.end(), [](double d) { return !std::isfinite(d); })) {
        std::fill(jogs_.begin(), jogs_.end(), 0.0f);
        return hardware_interface::return_type::OK;
    }

    // print the set pos and set vel vectors in the console
    std::string output = "";
    for (unsigned int i = 0; i < n_joints; i++) {
        output += std::to_string(cmd_position_[i]) + " ";
    }
    // RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "cmd_position_: %s", output.c_str());
    output = "";
    for (unsigned int i = 0; i < n_joints; i++) {
        output += std::to_string(cmd_velocity_[i]) + " ";
    }
    RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "cmd_velocity_: %s", output.c_str());

    // Velocity command
    if (std::none_of(cmd_velocity_.begin(), cmd_velocity_.end(), [](double d) { return !std::isfinite(d); })) {
        // Use the velocities from the command vector and convert them to the right unit
		output = "";
		for (unsigned int i = 0; i < n_joints; i++) {
			//cmd_velocity_[i] = std::abs(cmd_velocity_[i]) < 0.07 ? 0.0f : cmd_velocity_[i];

			// conversion from [rad/s] to jogs [%max/s]
			int sign = cmd_velocity_[i] >= 0 ? 1 : -1;
			jogs_[i] = sign * (cmd_velocity_[i] * 100.0 / ( M_PI / 4.0) ) * move_velocity_factor;
			output += std::to_string(jogs_[i]) + " ";
		}
		
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "Moved with velocity %s", output.c_str());

    } else if (std::none_of(cmd_position_.begin(), cmd_position_.end(), [](double d) { return !std::isfinite(d); })) {  // Position command

        // Only send a message if the command changed. Since the CRI controller can't handle new
        // position goals while still moving we only want to send the goal position without any
        // interpolation.

        if (cmd_position_ != cmd_last_position_) {  // do not repeat command if it is the same as the last one
            std::ostringstream msg;
            // command move function
            // Limit the precision to one digit behind the decimal point
            msg << "Move Joint " << std::fixed << std::setprecision(1);

            // Add the joint goals as degrees, one value per joint
            for (unsigned int i = 0; i < cmd_position_.size(); i++) {
                msg << ((cmd_position_[i] + pos_offset_[i]) * 180.0 / M_PI) << " ";
            }

            // send command here

            RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "Move position msg: %s", msg.str().c_str());

            cmd_last_position_ = cmd_position_;
        }
    }  // No movement
    else {
        std::fill(jogs_.begin(), jogs_.end(), 0.0f);  // fill with zeros
    }

    return hardware_interface::return_type::OK;
}

}  // namespace igus_rebel_hw_controller

// include the macro below to make the controller a plugin
#include "pluginlib/class_list_macros.hpp"
// export the class as a callable plugin for ROS2-control
PLUGINLIB_EXPORT_CLASS(igus_rebel_hw_controller::FakeController, hardware_interface::SystemInterface)
