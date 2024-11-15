#include "network/udp_sender.hpp"
#include <iostream>
#include <cstring>

UDPSender::UDPSender(const std::string& ip, int port)
    : ip_(ip)
    , port_(port)
    , socket_(INVALID_SOCKET)
    , is_initialized_(false)
{
}

UDPSender::~UDPSender() {
    close();
}

bool UDPSender::init() {
#ifdef _WIN32
    // Windows socket initialization
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data_) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return false;
    }
#endif

    // Create socket
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
#ifdef _WIN32
        WSACleanup();
#endif
        return false;
    }

    // Set target address
    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(port_);

#ifdef _WIN32
    inet_pton(AF_INET, ip_.c_str(), &target_addr_.sin_addr);
#else
    target_addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
#endif

    is_initialized_ = true;
    return true;
}

bool UDPSender::send(const char* data, size_t size) {
    if (!is_initialized_) {
        std::cerr << "Socket not initialized" << std::endl;
        return false;
    }

#ifdef _WIN32
    int result = sendto(socket_, data, static_cast<int>(size), 0,
                       (SOCKADDR*)&target_addr_, sizeof(target_addr_));
#else
    ssize_t result = sendto(socket_, data, size, 0,
                           (SOCKADDR*)&target_addr_, sizeof(target_addr_));
#endif

    if (result == SOCKET_ERROR) {
        std::cerr << "Send failed" << std::endl;
        return false;
    }

    return true;
}

void UDPSender::close() {
    if (socket_ != INVALID_SOCKET) {
        CLOSE_SOCKET(socket_);
        socket_ = INVALID_SOCKET;
    }

#ifdef _WIN32
    if (is_initialized_) {
        WSACleanup();
    }
#endif

    is_initialized_ = false;
}
