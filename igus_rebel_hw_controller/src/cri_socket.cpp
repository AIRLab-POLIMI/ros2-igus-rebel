#include "cri_socket.h"

namespace igus_rebel_hw_controller {

CriSocket::CriSocket(const std::string &ip, const int &port, const int &timeout) : sock(0),
																				   ip(ip),
																				   port(port),
																				   timeout(timeout),
																				   unprocessedMessages(),
																				   continueReceive(false),
																				   maxUnprocessedMessages(25),
																				   listCheckWaitMs(500),
																				   connectionNeeded(false),
																				   fragmentBuffer{0},
																				   fragmentLength(0) {}

CriSocket::~CriSocket() {
	stop();
}

// establish a connection to the robot via ethernet socket
void CriSocket::makeConnection() {
	// Make sure that we do not try to establish the same connection multiple times
	// at the same time.
	std::lock_guard<std::mutex> lockGuard(connectionLock);

	while (connectionNeeded) {
		sock = 0;
		struct sockaddr_in serv_addr;

		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			RCLCPP_ERROR(logger_, "Socket creation error.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(port);

		// Convert IPv4 and IPv6 addresses from text to binary form
		if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
			RCLCPP_ERROR(logger_, "Invalid robot IP address / Address not supported.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
			RCLCPP_ERROR(logger_, "Connection Failed.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		connectionNeeded = false;
		RCLCPP_INFO(logger_, "Connected to ReBeL at %s:%d", ip.c_str(), port);
	}
}

/**
 * @brief Add data to the stack of unprocessed messages.
 * @param msg The message containing the data to be separated and added to the stack.
 * @param length The length of the message.
 */
void CriSocket::addDataToStack(const char *msg, int length) {
	std::string data_msg = std::string(msg, length);

	bool complete = true;
	long unsigned int start_pos = data_msg.find(cri_keywords::START);
	long unsigned int end_pos = data_msg.find(cri_keywords::END);

	do {

		// check whether there is a complete message remaining in the buffer
		complete = !(start_pos == std::string::npos || end_pos == std::string::npos);

		if (complete) {
			// after "START" there is a whitespace, a number and a whitespace
			// we need to skip these
			start_pos = start_pos + cri_keywords::START.size() + 1;
			start_pos = data_msg.find(" ", start_pos) + 1;

			std::string data = data_msg.substr(start_pos, end_pos - start_pos);
			{
				// defines the local scope of the lockGuard, critical section here
				std::lock_guard<std::mutex> lockGuard(messageLock);
				unprocessedMessages.push_front(data);
			}

			data_msg = data_msg.substr(end_pos + cri_keywords::END.size());

			start_pos = data_msg.find(cri_keywords::START);
			end_pos = data_msg.find(cri_keywords::END);
		}

	} while (!complete);
}

void CriSocket::receiveThreadFunction() {
	if (connectionNeeded) {
		makeConnection();
	}

	RCLCPP_INFO(logger_, "Starting to receive messages from robot.");

	char buffer[bufferSize] = {0};

	while (continueReceive) {
		if (isSocketOk()) {
			int valread = read(sock, buffer, bufferSize);

			if (valread == 0) {
				RCLCPP_WARN(logger_, "Empty message received");
				connectionNeeded = true;
			} else {
				addDataToStack(buffer, valread);
			}
		}
	}

	RCLCPP_INFO(logger_, "Stopped to receive messages from robot.");
}

// Make sure that we do not fill our entire memory with messages from the robot in case something
// goes wrong with processing them.
// Also, later we should just stop the robot here, because this could be unsafe.
void CriSocket::listCheckThreadFunction() {
	RCLCPP_DEBUG(logger_, "Starting to check if the message list is being processed.");

	while (continueReceive) {
		if (unprocessedMessages.size() > maxUnprocessedMessages) {
			RCLCPP_WARN(logger_, "Robot messages are not processed fast enough. Discarding messages.");

			while (unprocessedMessages.size() > (maxUnprocessedMessages * 0.9)) {
				unprocessedMessages.pop_back();
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(listCheckWaitMs));
	}

	RCLCPP_DEBUG(logger_, "Stopped to check if the message list is being processed.");
}

void CriSocket::setIp(std::string ip) {
	this->ip = ip;
}

bool CriSocket::isSocketOk() {
	int error = 0;
	socklen_t len = sizeof(error);
	int retval = getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);

	if (retval != 0) {
		RCLCPP_ERROR(logger_, "Error getting socket error code: %s", strerror(retval));
		return false;
	}

	if (error != 0) {
		RCLCPP_ERROR(logger_, "Socket error: %s", strerror(error));
		return false;
	}

	return true;
}

void CriSocket::start() {
	connectionNeeded = true;
	continueReceive = true;

	// listCheckThread = std::thread(&CriSocket::listCheckThreadFunction, this);
	receiveThread = std::thread(&CriSocket::receiveThreadFunction, this);
}

void CriSocket::stop() {
	connectionNeeded = false;
	continueReceive = false;

	// waits for the threads to finish and then terminates them
	if (receiveThread.joinable()) {
		receiveThread.join();
	}

	// if (listCheckThread.joinable()) {
	//	listCheckThread.join();
	// }
}

bool CriSocket::hasMessage() {
	bool check;
	{
		std::lock_guard<std::mutex> lockGuard(messageLock);
		check = unprocessedMessages.size() > 0;
	}
	return check;
}

std::string CriSocket::getMessage() {
	std::string msg;
	{
		std::lock_guard<std::mutex> lockGuard(messageLock);
		msg = unprocessedMessages.back();
		unprocessedMessages.pop_back();
	}

	return msg;
}

void CriSocket::sendMessage(const std::string &msg) {
	std::lock_guard<std::mutex> lockGuard(socketWriteLock);

	if (connectionNeeded) {
		makeConnection();
	}

	int sent = send(sock, msg.c_str(), msg.length(), 0);

	if (sent < 0) {
		// connectionNeeded = true;
		RCLCPP_WARN(logger_, "failed to send message data to the robot.");
	}
}
} // namespace igus_rebel_hw_controller
