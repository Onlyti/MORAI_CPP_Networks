#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "network/udp_receiver.hpp"

int main(int argc, char* argv[]) {
    // initialize UDP receiver
    std::string ip = "127.0.0.1";  // local host IP
    int port = atoi(argv[1]);               // port number from argument
    
    // create UDP receiver object
    std::shared_ptr<MoraiCppUdp::UDPReceiver> ptr_receiver;
    
    try {
        // connect UDP socket
        ptr_receiver = std::make_shared<MoraiCppUdp::UDPReceiver>(ip, port);
        
        while(true) {
            // receive data
            char received_data[65000];
            size_t buffer_size = 65000;
            size_t received_size = 0;
            if(ptr_receiver->Receive(received_data, buffer_size, received_size)) {
                std::cout << "received size: " << received_size << std::endl;
                std::cout << "Received data: " << received_data << std::endl;
            }

            // delay
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    // 연결 종료
    ptr_receiver->Close();

    return 0;
}
