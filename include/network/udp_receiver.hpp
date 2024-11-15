/**
 * @file udp_receiver.hpp
 * @author Jiwon Seok (pauljiwon96@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-11-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __UDP_RECEIVER_HPP__
#define __UDP_RECEIVER_HPP__

#pragma once
#include <string>
#include <stdexcept>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define SOCKET_TYPE SOCKET
    #define SOCKADDR struct sockaddr
    #define CLOSE_SOCKET closesocket
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define SOCKET_TYPE int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define SOCKADDR struct sockaddr
    #define CLOSE_SOCKET close
#endif

class UDPReceiver {
public:
    UDPReceiver(const std::string& ip, int port);
    ~UDPReceiver();
    
    bool init();
    bool receive(char* buffer, size_t buffer_size, size_t& received_size);
    void close();

private:
    std::string ip_;
    int port_;
    SOCKET_TYPE socket_;
    struct sockaddr_in server_addr_;
    bool is_initialized_;

#ifdef _WIN32
    WSADATA wsa_data_;
#endif
};

#endif
