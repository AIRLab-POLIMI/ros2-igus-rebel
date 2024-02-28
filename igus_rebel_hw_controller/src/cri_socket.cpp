#include "cri_socket.h"

namespace igus_rebel_hw_controller {

CriSocket::CriSocket(const std::string &ip, const int &port, const int &timeout) : sock(0),
																				   ip(ip),
																				   port(port),
																				   timeout(timeout),
																				   unprocessedMessages(),
																				   continueReceive(false),
																				   maxUnprocessedMessages(25), // Small for testing
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
			RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Socket creation error.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(port);

		// Convert IPv4 and IPv6 addresses from text to binary form
		if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
			RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Invalid robot IP address / Address not supported.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Connection Failed.");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		connectionNeeded = false;
		RCLCPP_INFO(rclcpp::get_logger("hw_controller::cri_socket"), "Connected to ReBeL at %s:%d", ip.c_str(), port);
	}
}

void CriSocket::separateMessages(const char *msg) {
	const char *start;
	const char *end = msg;

	if (fragmentLength != 0) {
		start = std::strstr(msg, cri_keywords::START.c_str());
		end = std::strstr(msg, cri_keywords::END.c_str());

		if (end == nullptr || (start != nullptr && end > start)) {
			RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "There was a partial robot message, but could not find the end of it in the next message.");
		} else {
			std::string result1(fragmentBuffer.front(), fragmentLength);
			std::string result2(msg, end - msg);

			{ // defines the local scope of the lockGuard, critical section here
				std::lock_guard<std::mutex> lockGuard(messageLock);
				unprocessedMessages.push_front(result1 + result2);
			}
		}

		fragmentLength = 0;
	}

	while (true) {
		start = std::strstr(end, cri_keywords::START.c_str());

		if (start == nullptr) {
			break;
		}

		end = std::strstr(start, cri_keywords::END.c_str());

		if (end == nullptr) {
			// Found a start without end.
			const char *remainingStart = start + cri_keywords::START.size();
			const char *remainingEnd = std::strchr(remainingStart, '\0');

			if (remainingEnd != nullptr) {
				fragmentLength = remainingEnd - remainingStart;

				for (int i = 0; i < fragmentLength; i++) {
					fragmentBuffer[i] = *(remainingStart + i);
				}
			} else {
				RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Socket read was not null-terminated, somehow.");
			}

			break;
		}

		{ // defines the local scope of the lockGuard, critical section here
			std::lock_guard<std::mutex> lockGuard(messageLock);
			unprocessedMessages.push_front(std::string(
				start + cri_keywords::START.size() + 1,
				end - (start + cri_keywords::START.size() + 1) - 1));
		}
	}
}

void CriSocket::receiveThreadFunction() {
	RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::cri_socket"), "Starting to receive messages from robot.");

	char buffer[bufferSize] = {0};

	while (continueReceive) {
		if (connectionNeeded) {
			makeConnection();
		}

		int valread = read(sock, buffer, bufferSize);

		if (!isSocketOk()) {
			connectionNeeded = true;
		} else {
			if (valread == 0) {
				RCLCPP_WARN(rclcpp::get_logger("hw_controller::cri_socket"), "Empty message received");
				connectionNeeded = true;
			} else {
				separateMessages(buffer);
			}
		}

		// std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::cri_socket"), "Stopped to receive messages from robot.");
}

// Make sure that we do not fill our entire memory with messages from the robot in case something
// goes wrong with processing them.
// Also, later we should just stop the robot here, because this could be unsafe.
void CriSocket::listCheckThreadFunction() {
	RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::cri_socket"), "Starting to check if the message list is being processed.");

	while (continueReceive) {
		if (unprocessedMessages.size() > maxUnprocessedMessages) {
			RCLCPP_WARN(rclcpp::get_logger("hw_controller::cri_socket"), "Robot messages are not processed fast enough. Discarding messages.");

			while (unprocessedMessages.size() > (maxUnprocessedMessages * 0.9)) {
				unprocessedMessages.pop_back();
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(listCheckWaitMs));
	}

	RCLCPP_DEBUG(rclcpp::get_logger("hw_controller::cri_socket"), "Stopped to check if the message list is being processed.");
}

void CriSocket::setIp(std::string ip) {
	this->ip = ip;
}

bool CriSocket::isSocketOk() {
	int error = 0;
	socklen_t len = sizeof(error);
	int retval = getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);

	if (retval != 0) {
		RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Error getting socket error code: %s", strerror(retval));
		return false;
	}

	if (error != 0) {
		RCLCPP_ERROR(rclcpp::get_logger("hw_controller::cri_socket"), "Socket error: %s", strerror(error));
		return false;
	}

	return true;
}

void CriSocket::start() {
	connectionNeeded = true;
	continueReceive = true;

	listCheckThread = std::thread(&CriSocket::listCheckThreadFunction, this);
	receiveThread = std::thread(&CriSocket::receiveThreadFunction, this);
}

void CriSocket::stop() {
	connectionNeeded = false;
	continueReceive = false;

	// waits for the threads to finish and then terminates them
	if (receiveThread.joinable()) {
		receiveThread.join();
	}

	if (listCheckThread.joinable()) {
		listCheckThread.join();
	}
}

bool CriSocket::hasMessage() {
	return unprocessedMessages.size() > 0;
}

std::string CriSocket::getMessage() {
	std::lock_guard<std::mutex> lockGuard(messageLock);

	if (!hasMessage()) {
		return "";
	}

	std::string msg = unprocessedMessages.back();
	unprocessedMessages.pop_back();

	return msg;
}

void CriSocket::sendMessage(const std::string &msg) {
	std::lock_guard<std::mutex> lockGuard(socketWriteLock);

	if (connectionNeeded) {
		makeConnection();
	}

	int sent = send(sock, msg.c_str(), msg.length(), 0);

	if (!isSocketOk()) {
		connectionNeeded = true;
	}

	if (sent < 0) {
		connectionNeeded = true;
	}
}
} // namespace igus_rebel_hw_controller
