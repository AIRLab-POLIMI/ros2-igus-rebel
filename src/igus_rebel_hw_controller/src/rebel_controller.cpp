#include "rebel_controller.h"

namespace igus_rebel_hw_controller {

/**
 * @brief Construct a new Rebel Controller object and initialize joints values
 *
 * @param ip string ip address for the socket connection
 * @param port port for the socket connection
 */
RebelController::RebelController(const std::string &ip, const int &port) : cri_socket(ip, port, 200),
                                                                           currentStatus(),
                                                                           controlMode("velocity"),
                                                                           continueAlive(false),
                                                                           continueMessage(false),
                                                                           aliveWaitMs(50),
                                                                           cmd_counter(1),
                                                                           lastKinstate(cri_messages::Kinstate::NO_ERROR),
                                                                           kinematicLimits(cri_messages::KinematicLimits()) {
}

/**
 * @brief Construct a new Rebel Controller object and initialize joints values
 * 	Uses the default ip address and port for the socket connection
 */
RebelController::RebelController() : cri_socket(ip_address, port, 200),
                                     currentStatus(),
                                     controlMode("velocity"),
                                     continueAlive(false),
                                     continueMessage(false),
                                     aliveWaitMs(1000),
                                     cmd_counter(1),
                                     lastKinstate(cri_messages::Kinstate::NO_ERROR),
                                     kinematicLimits(cri_messages::KinematicLimits()) {
}

// empty destructor because not needed
RebelController::~RebelController() {
}

/**
 * @brief This sends the keep alive heartbeat to the CRI controller. The message always
 * contains a jog, under normal position command operation it should be left at 0.
 */
void RebelController::AliveThreadFunction() {
    RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::rebel_controller"), "Starting to send ALIVEJOG");

    // continue alive must be set to true as long as the robot is not moving (sends 0 jog velocities)
    // or if the robot is moving, send the correct jog values (computed velocities)
    while (continueAlive) {
        std::ostringstream msg;
        msg << std::showpoint;
        msg << std::fixed;
        msg << std::setprecision(8);
        msg << "CRISTART " << get_cmd_counter() << " ";
        msg << "ALIVEJOG ";
        // sends the jog values as an heartbeat for the manipulator
        for (auto j : jogs_) {
            msg << j << " ";
        }

        for (int i = jogs_.size(); i < 9; i++) {
            msg << 0.0f << " ";
        }
        msg << "CRIEND" << std::endl;

        // This seems unnecessary, if any weird communication behaviour occurs
        // then it might have been there for a reason. Else remove this in a
        // future release
        {
            std::lock_guard<std::mutex> lockGuard(aliveLock);
            cri_socket.sendMessage(msg.str());
        }

        // sends an alive message every aliveWaitMs milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs));
    }

    RCLCPP_WARN(rclcpp::get_logger("hw_controller::rebel_controller"), "Stopped to send ALIVEJOG");
}

/**
 * @brief processes the messages by type from the socket queue
 */
void RebelController::MessageThreadFunction() {
    RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::rebel_controller"), "Starting to process robot messages");

    while (continueMessage) {
        if (cri_socket.hasMessage()) {
            std::string msg = cri_socket.getMessage();

            //RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "raw msg data: %s", msg.c_str());
            cri_messages::MessageType type = cri_messages::CriMessage::GetMessageType(msg);
            // RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::rebel_controller"), "data type: %d", type);

            switch (type) {
                case cri_messages::MessageType::STATUS: {
                    try {
                        cri_messages::Status status = cri_messages::Status(msg);
                        // status.Print();
                        currentStatus = status;
                        ProcessStatus(currentStatus);
                    } catch (std::invalid_argument &e) {
						RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_messages"), "Conversion error from string to integer/float: %s", e.what());
                    }
                    break;
                }

                case cri_messages::MessageType::MESSAGE: {
                    cri_messages::Message message = cri_messages::Message(msg);
                    // Not sure if the ROS node should display these?
                    // RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "MESSAGE: %s", message.message.c_str());
                    break;
                }

                case cri_messages::MessageType::CMD: {  // this type of messages are received, not sent
                    cri_messages::Command command = cri_messages::Command(msg);

                    // Not sure if the ROS node should display these?
                    RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "CMD: %s", command.command.c_str());
                    break;
                }

                case cri_messages::MessageType::CONFIG: {
                    cri_messages::ConfigType configType = cri_messages::Config::GetConfigType(msg);
                    // print kinematic limits as degree angles once the robot is set up
                    if (configType == cri_messages::ConfigType::KINEMATICLIMITS) {
                        kinematicLimits.print_once(msg);
                    }
                    break;
                }

                case cri_messages::MessageType::INFO: {
                    cri_messages::Info info = cri_messages::Info(msg);
                    RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "INFO: %s", info.info.c_str());
                    break;
                }
                case cri_messages::MessageType::CMDERROR: {
                    RCLCPP_ERROR(rclcpp::get_logger("hw_controller::rebel_controller"), "command error received: %s", msg.c_str());
                    break;
                }

                default:
                    if (type != cri_messages::MessageType::OPINFO && type != cri_messages::MessageType::GSIG &&
                        type != cri_messages::MessageType::GRIPPERSTATE && type != cri_messages::MessageType::RUNSTATE &&
                        type != cri_messages::MessageType::CMDACK) {
                        RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "raw unknown data: %s", msg.c_str());
                    }

                    break;
            }
        }
    }

    // NOTE: this should be a debug message
    RCLCPP_WARN(rclcpp::get_logger("hw_controller::rebel_controller"), "Stopped to process robot messages");
}

/**
 * @brief The current 4-digits command counter in [1:9999]
 *
 * From the CRI reference:
 * All messages from the server do have an incrementing sCnt as first parameter,
 * the messages from the client an independent cCnt. Both are incremented with each
 * message from 1 to 9999, then reset to 1
 */
int RebelController::get_cmd_counter() {
    std::lock_guard<std::mutex> lockGuard(counterLock);
    int current = cmd_counter;
    cmd_counter = (cmd_counter % 9999) + 1;
    return current;
}

/**
 * @brief Adds the header and footer to a command message and sends it to the controller.
 */
void RebelController::Command(const std::string &command) {
    std::ostringstream msg;
    msg << cri_keywords::START << " " << get_cmd_counter() << " ";
    msg << cri_keywords::TYPE_CMD << " ";
    msg << command << " ";
    msg << cri_keywords::END << std::endl;

    cri_socket.sendMessage(msg.str());
}

/**
 * @brief Adds the header and footer to a config request message and sends it to the controller.
 */
void RebelController::GetConfig(const std::string &config) {
    std::ostringstream msg;
    msg << cri_keywords::START << " " << get_cmd_counter() << " ";
    msg << cri_keywords::TYPE_CONFIG << " ";
    msg << config << " ";
    msg << cri_keywords::END << std::endl;

    cri_socket.sendMessage(msg.str());
}

/**
 * @brief This is the main read method. It parses the information sent from the controller
 * and adjusts the internal state as such.
 */
void RebelController::ProcessStatus(const cri_messages::Status &status) {
    cri_messages::Kinstate currentKinstate = status.kinstate;
    std::array<int, 16> currentErrorJoints = status.errorJoints;

    if (lastKinstate != currentKinstate) {
        if (lastKinstate != cri_messages::Kinstate::NO_ERROR) {
            RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "Kinematics error resolved [%s]", kinstateMessage.c_str());
        }

        if (currentKinstate != cri_messages::Kinstate::NO_ERROR) {
            static const std::map<cri_messages::Kinstate, std::string> kinstate_msg_map = {
                {cri_messages::Kinstate::JOINT_LIMIT_MIN, "joint at minimum limit"},
                {cri_messages::Kinstate::JOINT_LIMIT_MAX, "joint at maximum limit"},
                {cri_messages::Kinstate::CARTESIAN_SINGULARITY_CENTER, "cartesian singularity (center)"},
                {cri_messages::Kinstate::CARTESIAN_SINGULARITY_REACH, "cartesian singularity (reach)"},
                {cri_messages::Kinstate::CARTESIAN_SINGULARITY_WRIST, "cartesian singularity (wrist)"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_1, "tool at virtual box limit 1"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_2, "tool at virtual box limit 2"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_3, "tool at virtual box limit 3"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_4, "tool at virtual box limit 4"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_5, "tool at virtual box limit 5"},
                {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_6, "tool at virtual box limit 6"},
                {cri_messages::Kinstate::MOTION_NOT_ALLOWED, "motion not allowed"},
                {cri_messages::Kinstate::UNKNOWN, "unknown error"},
            };

            auto it = kinstate_msg_map.find(currentKinstate);

            if (it != kinstate_msg_map.end()) {
                kinstateMessage = kinstate_msg_map.at(currentKinstate);
            }

            RCLCPP_ERROR(rclcpp::get_logger("hw_controller::rebel_controller"), "Kinematics error [%s]", kinstateMessage.c_str());
        }
    }

    if (currentErrorJoints != lastErrorJoints) {
        // loop throught the 6 joint errors
        for (unsigned int i = 0; i < 6; i++) {
            int errorJoint = currentErrorJoints.at(i);
            // std::array<int, 8> errorJointBit; // could be represented as a uint8_t

            // extract bits from the error to analyze it
            std::bitset<8> errorJointBit(errorJoint);

            if (errorJoint != lastErrorJoints.at(i)) {
                std::string errorMsg = "";
                if (errorJointBit[0]) {
                    errorMsg += "'Overtemperature' ";
                }

                if (errorJointBit[1]) {
                    errorMsg += "'Supply too low: Is emergency button pressed?' ";
                }

                if (errorJointBit[2]) {
                    errorMsg += "'Motor not enabled' ";
                }

                if (errorJointBit[3]) {
                    errorMsg += "'Communication watch dog' ";
                }

                if (errorJointBit[4]) {
                    errorMsg += "'Position lag' ";
                }

                if (errorJointBit[5]) {
                    errorMsg += "'Encoder Error' ";
                }

                if (errorJointBit[6]) {
                    errorMsg += "'Overcurrent' ";
                }

                if (errorJointBit[7]) {
                    errorMsg += "'Driver error/SVM' ";
                }

                if (errorMsg != "") {
                    RCLCPP_ERROR(rclcpp::get_logger("hw_controller::rebel_controller"), "Joint %i Error: [%s]", i, errorMsg.c_str());
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "Joint %i Error: Cleaned (empty error message string)", i);
                }
            }
        }
    }

    lastKinstate = currentKinstate;
    lastErrorJoints = currentErrorJoints;
}

/**
 * @brief Currently unused method that requests info about the state of the referencing.
 */
void RebelController::GetReferenceInfo() {
    Command(std::string("GetReferencingInfo"));
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
hardware_interface::CallbackReturn RebelController::on_init(const hardware_interface::HardwareInfo &info) {
    // In the first line usually the parents on_init is called to process standard values
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Overwrite the default IP address in the urdf
    if (info_.hardware_parameters.count("ip") > 0) {  // if ip address is set in the urdf
        std::string ip = info_.hardware_parameters.at("ip");
        cri_socket.setIp(ip);
    }

    for (const hardware_interface::ComponentInfo &joint : info.joints) {
        // Read the joint offset from the ros2_control configuration
        double cri_joint_offset = 0.0;
        if (joint.parameters.count("cri_joint_offset") > 0) {
            // convert string to double
            cri_joint_offset = stod(joint.parameters.at("cri_joint_offset"));
        } else {
            RCLCPP_WARN(rclcpp::get_logger("hw_controller::rebel_controller"),
                        "No cri_joint_offset specified for joint %s, using default value of %lf", joint.name.c_str(), cri_joint_offset);
        }
        pos_offset_.push_back(cri_joint_offset);
    }

    // print command interfaces names
    for (const hardware_interface::InterfaceInfo &cmd_interface : info.joints.at(0).command_interfaces) {
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "Command interface found: %s", cmd_interface.name.c_str());
    }
    // print state interfaces names
    for (const hardware_interface::InterfaceInfo &state_interface : info.joints.at(0).state_interfaces) {
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "State interface found: %s", state_interface.name.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "Detected %lu joints in the urdf file", jogs_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief where hardware “power” is enabled. The on_activate is called once when the controller
 * 	is activated. This method should handle controller restarts, such as setting the resetting
 * 	reference to safe values. It should also perform controller specific safety checks.
 * @return CallbackReturn::SUCCESS if the controller was successfully activated
 * 	else CallbackReturn::FAILURE
 */
hardware_interface::CallbackReturn RebelController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    continueMessage = true;
    messageThread = std::thread(&RebelController::MessageThreadFunction, this);

    cri_socket.start();

    // The following delay does not appear to be necessary, re-add if problems occur
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    Command(cri_keywords::COMMAND_CONNECT);
    Command(cri_keywords::COMMAND_RESET);
    Command(cri_keywords::COMMAND_ENABLE);

    continueAlive = true;
    aliveThread = std::thread(&RebelController::AliveThreadFunction, this);

    GetConfig(cri_keywords::CONFIG_GETKINEMATICLIMITS);

    // setControlMode("velocity");

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief on_deactivate is called when a controller stops running. It is important to release the claimed command interface
 *  in this method, so other controllers can use them if needed. This method is called before the controller is shut down
 * @return CallbackReturn::SUCCESS if the controller was successfully activated
 * 	else CallbackReturn::FAILURE
 */
hardware_interface::CallbackReturn RebelController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    std::fill(jogs_.begin(), jogs_.end(), 0.0f);

    std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

    Command(cri_keywords::COMMAND_DISABLE);
    Command(cri_keywords::COMMAND_DISCONNECT);

    continueAlive = false;

    // waits for the alive thread to finish its execution and terminates it
    if (aliveThread.joinable()) {
        aliveThread.join();
    }

    cri_socket.stop();

    continueMessage = false;

    // waits for the message thread to finish its execution and terminates it
    if (messageThread.joinable()) {
        messageThread.join();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief defines the interfaces that the hardware components are offering
 * @return a vector of StateInterface, describing the state_interfaces for each joint.
 * 	The StateInterface objects are read only data handles. Their constructors require an interface name,
 *  interface type, and a pointer to a double data value.
 */
std::vector<hardware_interface::StateInterface> RebelController::export_state_interfaces() {
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
std::vector<hardware_interface::CommandInterface> RebelController::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Add the position and velocity command signals for each joint defined
    for (unsigned int i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]));
    }

    // NOTE: include here specific state_interfaces for additional controllers such as digital IO

    return command_interfaces;
}

/**
 * @brief Reads the current joint positions from the robot and converts them to radians for the ROS2 controller.
 * 	getting the states from the hardware and storing them to internal variables defined in export_state_interfaces()
 * 	This is the core method in the ros2_control loop. During the main loop, ros2_control loops over all hardware components
 *  and calls the read method. It is executed on the realtime thread, hence the method must obey by realtime constraints.
 * 	The read method is responsible for updating the data values of the state_interfaces. Since the data value point to
 * 	class member variables, those values can be filled with their corresponding sensor values, which will in turn update
 * 	the values of each exported StateInterface object.
 * @return hardware_interface::return_type::OK if the read was successful
 */
hardware_interface::return_type RebelController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &duration) {
    std::vector<double> temp_pos;
    temp_pos.reserve(n_joints);

    // copy joint rotation angles [deg] values into temp_pos
    std::copy(currentStatus.posJointCurrent.begin(), currentStatus.posJointCurrent.begin() + position_feedback_.size(), temp_pos.begin());

    // degrees to radians and apply offset
    for (size_t i = 0; i < n_joints; i++) {
        temp_pos[i] = temp_pos[i] + pos_offset_[i];
        temp_pos[i] = temp_pos[i] * M_PI / 180.0;

        // compute estimated velocity by derivating positions in time
        velocity_feedback_[i] = (temp_pos[i] - position_feedback_[i]) / duration.seconds();

        // update position_feedback_ with temp_pos
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
hardware_interface::return_type RebelController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*duration*/) {
    // Make it possible to use different movement commands after another.
    /* wrong check for new commands received
        // If more than one set_... var is not NaN reset them all and wait for ros2 control to send a new goal.
        if (std::none_of(cmd_position_.begin(), cmd_position_.end(), [](double d) { return std::isnan(d); }) &&
                std::none_of(cmd_velocity_.begin(), cmd_velocity_.end(), [](double d) { return std::isnan(d); })) {
                // if there are no NaN values in the cmd_position_ and cmd_velocity_ vectors then reset them
                std::fill(cmd_position_.begin(), cmd_position_.end(), std::numeric_limits<double>::quiet_NaN());
                std::fill(cmd_velocity_.begin(), cmd_velocity_.end(), std::numeric_limits<double>::quiet_NaN());
        }
    */
    // if all cmd_position_ and cmd_velocity_ are filled with zeros, return OK
    if (std::all_of(cmd_position_.begin(), cmd_position_.end(), [](double d) { return d == 0.0; }) &&
        std::all_of(cmd_velocity_.begin(), cmd_velocity_.end(), [](double d) { return d == 0.0; })) {
        std::fill(jogs_.begin(), jogs_.end(), 0.0f);
        return hardware_interface::return_type::OK;
    }

    // print the set pos and set vel vectors in the console
    std::string output = "";
    for (unsigned int i = 0; i < cmd_position_.size(); i++) {
        output += std::to_string(cmd_position_[i]) + " ";
    }
    RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::fake_controller"), "cmd_position_: %s", output.c_str());
    output = "";
    for (unsigned int i = 0; i < cmd_velocity_.size(); i++) {
        output += std::to_string(cmd_velocity_[i]) + " ";
    }
    // RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::fake_controller"), "cmd_velocity_: %s", output.c_str());

    // Velocity command
    if (std::none_of(cmd_velocity_.begin(), cmd_velocity_.end(), [](double d) { return d == 0.0; })) {
        // Use the velocities from the command vector and convert them to the right unit
        // make sure the alive status messaging thread is still running
        std::copy(cmd_velocity_.begin(), cmd_velocity_.end(), jogs_.begin());
        RCLCPP_INFO(rclcpp::get_logger("hw_controller::fake_controller"), "Moved with velocity control");

    } else if (std::none_of(cmd_position_.begin(), cmd_position_.end(), [](double d) { return d == 0.0; })) {  // Position command

        // Only send a message if the command changed. Since the CRI controller can't handle new
        // position goals while still moving we only want to send the goal position without any
        // interpolation.
        //
        // TODO: Find ros2_controller which only sends the final position once or wait for a protocol
        // update for CRI (Issue #72)

        if (cmd_position_ != cmd_last_position_) {
            std::ostringstream msg;
            // command move function
            // Limit the precision to one digit behind the decimal point
            msg << "Move Joint " << std::fixed << std::setprecision(2);

            // Add the joint goals as degrees, one value per joint
            for (unsigned int i = 0; i < n_joints; i++) {
                msg << ((cmd_position_[i] + pos_offset_[i]) * 180.0 / M_PI) << " ";
            }
            // 3 additional control variables, add zeros as padding
            for (int i = n_joints; i < 9; i++) {
                msg << 0.0f << " ";
            }
            // add movement velocity as percentage (from 0 to 100) of the maximum velocity
            msg << move_velocity;

            Command(msg.str());  // sends move joint command via position control

            RCLCPP_INFO(rclcpp::get_logger("hw_controller::rebel_controller"), "Move position msg: %s", msg.str().c_str());

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
PLUGINLIB_EXPORT_CLASS(igus_rebel_hw_controller::RebelController, hardware_interface::SystemInterface)

/***********************************************
 * hardware interfaces setup for ros1 controller
 * *********************************************
 */
/*
// interfaces definitions equivalent to the export_state_interfaces() and export_command_interfaces() methods in ros2 control
// first initializes the joints states and then the joints commands interfaces
void Rebel::SetUpRosHardwareInterface() {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1(JOINT_NAME_1, &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_2(JOINT_NAME_2, &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_handle_3(JOINT_NAME_3, &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_handle_4(JOINT_NAME_4, &pos[3], &vel[3], &eff[3]);
    hardware_interface::JointStateHandle state_handle_5(JOINT_NAME_5, &pos[4], &vel[4], &eff[4]);
    hardware_interface::JointStateHandle state_handle_6(JOINT_NAME_6, &pos[5], &vel[5], &eff[5]);

    jnt_state_interface.registerHandle(state_handle_1);
    jnt_state_interface.registerHandle(state_handle_2);
    jnt_state_interface.registerHandle(state_handle_3);
    jnt_state_interface.registerHandle(state_handle_4);
    jnt_state_interface.registerHandle(state_handle_5);
    jnt_state_interface.registerHandle(state_handle_6);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle(JOINT_NAME_1), &vel_cmd[0]);
    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle(JOINT_NAME_2), &vel_cmd[1]);
    hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle(JOINT_NAME_3), &vel_cmd[2]);
    hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle(JOINT_NAME_4), &vel_cmd[3]);
    hardware_interface::JointHandle vel_handle_5(jnt_state_interface.getHandle(JOINT_NAME_5), &vel_cmd[4]);
    hardware_interface::JointHandle vel_handle_6(jnt_state_interface.getHandle(JOINT_NAME_6), &vel_cmd[5]);

    jnt_vel_interface.registerHandle(vel_handle_1);
    jnt_vel_interface.registerHandle(vel_handle_2);
    jnt_vel_interface.registerHandle(vel_handle_3);
    jnt_vel_interface.registerHandle(vel_handle_4);
    jnt_vel_interface.registerHandle(vel_handle_5);
    jnt_vel_interface.registerHandle(vel_handle_6);

    registerInterface(&jnt_vel_interface);
}

void Rebel::SetJog(const float &joint1, const float &joint2, const float &joint3,
                   const float &joint4, const float &joint5, const float &joint6) {
    j1 = joint1;
    j2 = joint2;
    j3 = joint3;
    j4 = joint4;
    j5 = joint5;
    j6 = joint6;
}

void Rebel::GetJoints(float &joint1, float &joint2, float &joint3,
                      float &joint4, float &joint5, float &joint6) {
    joint1 = currentStatus.posJointCurrent.at(0);
    joint2 = currentStatus.posJointCurrent.at(1);
    joint3 = currentStatus.posJointCurrent.at(2);
    joint4 = currentStatus.posJointCurrent.at(3);
    joint5 = currentStatus.posJointCurrent.at(4);
    joint6 = currentStatus.posJointCurrent.at(5);
}

void Rebel::read(const ros::Time & time, const ros::Duration &period) {
    read();

    vel[0] = (pos[0] - last_pos[0]) / period.toSec();
    vel[1] = (pos[1] - last_pos[1]) / period.toSec();
    vel[2] = (pos[2] - last_pos[2]) / period.toSec();
    vel[3] = (pos[3] - last_pos[3]) / period.toSec();
    vel[4] = (pos[4] - last_pos[4]) / period.toSec();
    vel[5] = (pos[5] - last_pos[5]) / period.toSec();

    last_pos[0] = pos[0];
    last_pos[1] = pos[1];
    last_pos[2] = pos[2];
    last_pos[3] = pos[3];
    last_pos[4] = pos[4];
    last_pos[5] = pos[5];
}

void Rebel::read() {
    pos[0] = currentStatus.posJointCurrent.at(0) * degToRad;
    pos[1] = currentStatus.posJointCurrent.at(1) * degToRad;
    pos[2] = currentStatus.posJointCurrent.at(2) * degToRad;
    pos[3] = currentStatus.posJointCurrent.at(3) * degToRad;
    pos[4] = currentStatus.posJointCurrent.at(4) * degToRad;
    pos[5] = currentStatus.posJointCurrent.at(5) * degToRad;
}

void Rebel::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Curently no use for time or period, here.
    write();
}

void Rebel::write() {
    j1 = (float)vel_cmd[0];
    j2 = (float)vel_cmd[1];
    j3 = (float)vel_cmd[2];
    j4 = (float)vel_cmd[3];
    j5 = (float)vel_cmd[4];
    j6 = (float)vel_cmd[5];
}

// equivalent to the on_activate method in ros2_control
void Rebel::Start() {
    continueMessage = true;
    messageThread = std::thread(&Rebel::MessageThreadFunction, this);

    rebelSocket.Start();

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    Command(cri_keywords::COMMAND_CONNECT);
    Command(cri_keywords::COMMAND_RESET);
    Command(cri_keywords::COMMAND_ENABLE);

    continueAlive = true;
    aliveThread = std::thread(&Rebel::AliveThreadFunction, this);

    GetConfig(cri_keywords::CONFIG_GETKINEMATICLIMITS);
}

// equivalent to the on_deactivate method in ros2_control
void Rebel::Stop() {
    j1 = 0.0f;
    j2 = 0.0f;
    j3 = 0.0f;
    j4 = 0.0f;
    j5 = 0.0f;
    j6 = 0.0f;

    std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

    Command(cri_keywords::COMMAND_DISABLE);
    Command(cri_keywords::COMMAND_DISCONNECT);

    continueAlive = false;

    // waits for the alive thread to finish its execution and terminates it
    if (aliveThread.joinable()) {
        aliveThread.join();
    }

    cri_socket.Stop();

    continueMessage = false;

    // waits for the message thread to finish its execution and terminates it
    if (messageThread.joinable()) {
        messageThread.join();
    }
}
*/
//}  // namespace igus_rebel_hw_controller
