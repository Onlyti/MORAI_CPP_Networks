#include "network/udp_receiver.hpp"
#include <iostream>
#include <cstring>

UDPReceiver::UDPReceiver(const std::string& ip, int port)
    : ip_(ip)
    , port_(port)
    , socket_(INVALID_SOCKET)
    , is_initialized_(false)
{
}

UDPReceiver::~UDPReceiver() {
    close();
}

bool UDPReceiver::init() {
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

    // Set server address
    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);

#ifdef _WIN32
    inet_pton(AF_INET, ip_.c_str(), &server_addr_.sin_addr);
#else
    server_addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
#endif

    // Bind socket
    if (bind(socket_, (SOCKADDR*)&server_addr_, sizeof(server_addr_)) == SOCKET_ERROR) {
        std::cerr << "Bind failed" << std::endl;
        close();
        return false;
    }

    is_initialized_ = true;
    return true;
}

bool UDPReceiver::receive(char* buffer, size_t buffer_size, size_t& received_size) {
    if (!is_initialized_) {
        std::cerr << "Socket not initialized" << std::endl;
        return false;
    }

    sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

#ifdef _WIN32
    int result = recvfrom(socket_, buffer, static_cast<int>(buffer_size), 0,
                         (SOCKADDR*)&client_addr, &client_addr_len);
#else
    ssize_t result = recvfrom(socket_, buffer, buffer_size, 0,
                             (SOCKADDR*)&client_addr, &client_addr_len);
#endif

    if (result == SOCKET_ERROR) {
        std::cerr << "Receive failed" << std::endl;
        return false;
    }

    received_size = static_cast<size_t>(result);
    return true;
}

void UDPReceiver::close() {
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
