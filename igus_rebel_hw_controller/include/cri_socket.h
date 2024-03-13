
#ifndef CRI_SOCKET_H
#define CRI_SOCKET_H

#include <array>
#include <list>
#include <mutex>
#include <stdio.h>
#include <string>
#include <sys/socket.h>
#include <thread>

#include "cri_keywords.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>

#include "rclcpp/rclcpp.hpp"

namespace igus_rebel_hw_controller {

class CriSocket {

private:
	rclcpp::Logger logger_ = rclcpp::get_logger("hw_controller::cri_socket");

	int sock;
	std::string ip;
	int port;
	int timeout;
	std::list<std::string> unprocessedMessages;

	bool continueReceive;
	std::thread receiveThread;
	std::thread listCheckThread;
	std::mutex socketWriteLock;
	std::mutex connectionLock;
	std::mutex messageLock;
	unsigned int maxUnprocessedMessages;
	int listCheckWaitMs;

	bool connectionNeeded;
	static const int bufferSize = 4096;

	std::array<char, bufferSize> fragmentBuffer;
	int fragmentLength;

	void makeConnection();
	void addDataToStack(const char *msg, int length);

	void receiveThreadFunction();
	void listCheckThreadFunction();

	bool isSocketOk();

public:
	CriSocket(const std::string &ip, const int &port, const int &timeout);
	~CriSocket();

	void setIp(std::string ip);
	void start();
	void stop();
	bool hasMessage();
	std::string getMessage();
	void sendMessage(const std::string &msg);
};
} // namespace igus_rebel_hw_controller

#endif
