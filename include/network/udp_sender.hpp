#pragma once
#include <string>
#include <stdexcept>
#include <thread>
#include <memory>
#include <cstring>

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

class UDPSender {
public:
    UDPSender(const std::string& ip, int port);
    UDPSender(const std::string& ip, int dest_port, int host_port);
    ~UDPSender();
    
    bool Init();
    bool Send(const char* data, size_t size);
    void Close();

private:
    std::string ip_;
    int dest_port_;
    int host_port_;
    SOCKET_TYPE socket_;
    struct sockaddr_in target_addr_;
    bool is_initialized_;

#ifdef _WIN32
    WSADATA wsa_data_;
#endif
};
