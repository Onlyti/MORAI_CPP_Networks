#include "network/udp_sender.hpp"
#include <iostream>
#include <cstring>

using namespace MoraiCppUdp;

UDPSender::UDPSender(const std::string& ip, int port)
    : ip_(ip)
    , dest_port_(port)
    , host_port_(-1)
    , socket_(INVALID_SOCKET)
    , is_initialized_(false)
{
    Init();
}

UDPSender::UDPSender(const std::string& ip, int dest_port, int host_port)
    : ip_(ip)
    , dest_port_(dest_port)
    , host_port_(host_port)
    , socket_(INVALID_SOCKET)
    , is_initialized_(false)
{
    Init();
}

UDPSender::~UDPSender() {
    Close();
}

bool UDPSender::Init() {
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

    // Bind to host port if specified
    if (host_port_ != -1) {
        sockaddr_in host_addr;
        std::memset(&host_addr, 0, sizeof(host_addr));
        host_addr.sin_family = AF_INET;
        host_addr.sin_port = htons(host_port_);
        host_addr.sin_addr.s_addr = INADDR_ANY;  // Bind to any local address

        if (bind(socket_, (struct sockaddr*)&host_addr, sizeof(host_addr)) == SOCKET_ERROR) {
            std::cerr << "Binding to host port failed" << std::endl;
            CLOSE_SOCKET(socket_);
#ifdef _WIN32
            WSACleanup();
#endif
            return false;
        }
    }

    // Set target address
    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(dest_port_);

#ifdef _WIN32
    inet_pton(AF_INET, ip_.c_str(), &target_addr_.sin_addr);
#else
    target_addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
#endif

    is_initialized_ = true;
    return true;
}

bool UDPSender::Send(const char* data, size_t size) {
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

void UDPSender::Close() {
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
