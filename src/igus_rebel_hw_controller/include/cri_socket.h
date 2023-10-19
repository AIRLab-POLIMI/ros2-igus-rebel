
#ifndef CRI_SOCKET_H
#define CRI_SOCKET_H

#include <stdio.h>
#include <sys/socket.h>

#include <list>
#include <mutex>
#include <string>
#include <thread>

#include "cri_keywords.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>

#include "rclcpp/rclcpp.hpp"

namespace igus_rebel_hw_controller {

class CriSocket {

   private:
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

    char fragmentBuffer[bufferSize];
    int fragmentLength;

    void makeConnection();
    void separateMessages(const char* msg);

    void receiveThreadFunction();
    void listCheckThreadFunction();

    bool isSocketOk();

   public:
    CriSocket(const std::string& ip, const int& port, const int& timeout);
    ~CriSocket();

	void setIp(std::string ip);
    void start();
    void stop();
    bool hasMessage();
    std::string getMessage();
    void sendMessage(const std::string& msg);

};
}  // namespace igus_rebel_hw_controller

#endif
