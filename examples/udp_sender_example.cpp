#include "network/udp_sender.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

int main() {
    UDPSender sender("127.0.0.1", 12345);
    
    if (!sender.Init()) {
        std::cerr << "Failed to initialize UDP sender" << std::endl;
        return -1;
    }

    // 예제 데이터 전송
    const std::string message = "Hello, UDP!";
    
    while (true) {
        if (sender.Send(message.c_str(), message.length())) {
            std::cout << "Sent: " << message << std::endl;
        }
        
        // 1초 대기
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
} 