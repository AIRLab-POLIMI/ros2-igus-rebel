
#include "gripper_controller.hpp"

namespace igus_rebel_gripper_controller {

GripperController::GripperController()
	: logger_(rclcpp::get_logger("igus_rebel::GripperController")),
	  serial_port("/dev/ttyUSB0"),
	  baud_rate(B115200),
	  serial_fd(-1),
	  grip_actuation_time(1500) {
	node_ = rclcpp::Node::make_shared("gripper_controller");
	grip_cmd = "off";
	grip_state = "off";
}

/**
 * @brief Callback function for the gripper actuation service "/gripper_actuate"
 * @param request the request message containing the desired gripper state
 * @param response the response message containing the current gripper state
 */
void GripperController::grip_service_callback(
	const std::shared_ptr<igus_rebel_gripper_controller::srv::GripperActuation::Request> request,
	const std::shared_ptr<igus_rebel_gripper_controller::srv::GripperActuation::Response> /*response*/) {

	// get the command from the service request
	grip_cmd = request->command;

	// check if the command is valid
	if (gripper_states[static_cast<int>(GripperStates::GRIPPED)] != grip_cmd &&
		gripper_states[static_cast<int>(GripperStates::RELEASED)] != grip_cmd &&
		gripper_states[static_cast<int>(GripperStates::OFF)] != grip_cmd) {
		RCLCPP_ERROR(logger_, "Invalid command received: %s", grip_cmd.c_str());
		return;
	}

	RCLCPP_INFO(logger_, "Service command: %s", grip_cmd.c_str());

	// wait until grip_state is updated and equal to the command
	while (grip_state != grip_cmd) {
		// update the state
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	// wait for a predefined time to wait for the gripper to actuate
	std::this_thread::sleep_for(std::chrono::milliseconds(grip_actuation_time));

	// return from the service call to the client
}

/**
 * @brief Setup the serial communication via UART to the arduino controller
 * @return true if the serial communication was successfully setup, false otherwise
 */
bool GripperController::setup_serial_comm(void) {
	serial_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial_fd == -1) {
		return false;
	}

	// configure the serial UART communication
	struct termios options;	
	tcgetattr(serial_fd, &options);
	cfsetispeed(&options, baud_rate); // Set baud rate
	cfsetospeed(&options, baud_rate);
	options.c_cflag &= ~PARENB; // No parity
	options.c_cflag &= ~CSTOPB; // 1 stop bit
	options.c_cflag |= CS8;		// 8 data bits
	tcflush(serial_fd, TCIFLUSH);
	tcsetattr(serial_fd, TCSANOW, &options);
	return true;
}

hardware_interface::CallbackReturn GripperController::on_init(const hardware_interface::HardwareInfo &info) {
	if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
		return hardware_interface::CallbackReturn::ERROR;
	}

	// initialize the hardware interface command and state
	hw_grip_cmd = new double;
	hw_grip_state = new double;
	*hw_grip_cmd = std::numeric_limits<double>::quiet_NaN();
	*hw_grip_state = std::numeric_limits<double>::quiet_NaN();

	// read the parameters from the ros2 control hardware interface
	// and set the serial port and baud rate
	baud_rate = stoi(info_.hardware_parameters["baud_rate"]);
	serial_port = info_.hardware_parameters["serial_port"];

	RCLCPP_INFO(logger_, "Serial port: %s, Baud rate: %d", serial_port.c_str(), baud_rate);

	// GripperController has exactly one state and command interface for the GPIO
	const hardware_interface::ComponentInfo &gpio = info_.gpios[0];

	if (gpio.command_interfaces.size() != 1) {
		RCLCPP_ERROR(logger_, "GPIO '%s' has %zu command interfaces found. 1 expected.",
					 gpio.name.c_str(), gpio.command_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.command_interfaces[0].name != GPIO_CMD) {
		RCLCPP_ERROR(logger_, "Joint '%s' have %s command interfaces found. Only '%s' expected.",
					 gpio.name.c_str(), gpio.command_interfaces[0].name.c_str(), GPIO_CMD.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.state_interfaces.size() != 1) {
		RCLCPP_ERROR(logger_, "GPIO '%s' has %zu state interface. 1 expected.",
					 gpio.name.c_str(), gpio.state_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (gpio.state_interfaces[0].name != GPIO_STATE) {
		RCLCPP_ERROR(logger_, "GPIO '%s' have %s state interface. Only '%s' expected.",
					 gpio.name.c_str(), gpio.state_interfaces[0].name.c_str(), GPIO_STATE.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperController::export_state_interfaces() {
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(hardware_interface::StateInterface(
		info_.gpios[0].name, GPIO_STATE, hw_grip_state));

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperController::export_command_interfaces() {
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	command_interfaces.emplace_back(hardware_interface::CommandInterface(
		info_.gpios[0].name, GPIO_CMD, hw_grip_cmd));

	return command_interfaces;
}

hardware_interface::CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

	grip_service_ = node_->create_service<igus_rebel_gripper_controller::srv::GripperActuation>(
		service_topic_, std::bind(&GripperController::grip_service_callback, this, std::placeholders::_1, std::placeholders::_2));

	node_spin_thread_ = std::thread([this]() {
		rclcpp::spin(node_);
		rclcpp::shutdown();
	});

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
	// set some default values for the state and command
	if (std::isnan(*hw_grip_cmd)) {
		*hw_grip_cmd = 0.0;
	}
	if (std::isnan(*hw_grip_state)) {
		*hw_grip_state = 0.0;
	}

	// attempt to establish communication with the arduino controller
	bool comm_established = false;
	int attempts = 0;
	while (!comm_established && attempts < 10) {
		comm_established = setup_serial_comm();
		attempts++;
		if (!comm_established) {
			RCLCPP_WARN(logger_, "Failed to establish communication with the arduino controller. Retrying...");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	if (!comm_established) {
		RCLCPP_ERROR(logger_, "Failed to establish communication with the arduino controller.");
		return hardware_interface::CallbackReturn::ERROR;
	} else {
		RCLCPP_INFO(logger_, "Communication with the arduino controller established.");
	
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {

	// close the serial port communication
	if (serial_fd != -1) {
		close(serial_fd);
	}

	node_spin_thread_.join();

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

	// no input from the serial port, skip

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
	// checks if new commands arrived
	// checks if the command is the same as the state
	if (grip_cmd == grip_state) {
		// no new command received, skip
		return hardware_interface::return_type::OK;
	}

	// write the command to the serial port
	int len = grip_cmd.length();
	char *command = (char*) malloc((len + 2) * sizeof(char));
	strcpy(command, grip_cmd.c_str());
	command[len] = '\n';
	command[len + 1] = '\0';
	::write(serial_fd, command, len + 2);

	// update the state
	grip_state = grip_cmd;

	return hardware_interface::return_type::OK;
}

} // namespace igus_rebel_gripper_controller

PLUGINLIB_EXPORT_CLASS(igus_rebel_gripper_controller::GripperController, hardware_interface::ActuatorInterface)