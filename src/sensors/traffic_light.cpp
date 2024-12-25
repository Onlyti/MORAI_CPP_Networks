#include "sensors/traffic_light.hpp"

#include <cstring>
#include <iostream>
#include <thread>

using namespace MoraiCppUdp;

TrafficLight::TrafficLight(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false) {
    is_running_ = true;
    thread_traffic_light_receiver_ = std::thread(&TrafficLight::ThreadTrafficLightReceiver, this);
}

TrafficLight::TrafficLight(const std::string& ip_address, uint16_t port, TrafficLightCallback callback)
    : UDPReceiver(ip_address, port), is_running_(false), traffic_light_callback_(callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    traffic_light_callback_ = callback;
    is_running_ = true;
    thread_traffic_light_receiver_ = std::thread(&TrafficLight::ThreadTrafficLightReceiver, this);
}

TrafficLight::~TrafficLight() {
    is_running_ = false;
    if (thread_traffic_light_receiver_.joinable()) {
        thread_traffic_light_receiver_.join();
    }
    Close();
}

void TrafficLight::ThreadTrafficLightReceiver() {
    char packet_buffer[PACKET_SIZE];

    while (is_running_) {
        try {
            size_t received_size = 0;
            if (!Receive(packet_buffer, PACKET_SIZE, received_size)) {
                std::cerr << "Failed to receive traffic light data" << std::endl;
                continue;
            }

            memset(&packet_data_, 0, sizeof(TrafficLightPacketStruct));
            if (ParseTrafficLight(packet_buffer, received_size, packet_data_)) {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (traffic_light_callback_) {
                    traffic_light_callback_(packet_data_.packet.traffic_light_data);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Traffic light error: " << e.what() << std::endl;
            continue;
        }
    }
}

bool TrafficLight::ParseTrafficLight(const char* buffer, size_t size, TrafficLightPacketStruct& data) {
    if (size != PACKET_SIZE) {
        return false;
    }

    size_t offset = 0;

    // Parse Sharp (1 byte) - "#"
    std::memcpy(&data.packet.sharp, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    if (data.packet.sharp != '#') {
        std::cerr << "Invalid sharp character" << std::endl;
        return false;
    }

    // Parse Header (12 bytes) - "TrafficLight"
    std::memcpy(&data.packet.header, buffer + offset, sizeof(char[12]));
    offset += sizeof(char[12]);
    if (strncmp(data.packet.header, "TrafficLight", 12) != 0) {
        std::cerr << "Invalid header: " << data.packet.header << std::endl;
        return false;
    }

    // Parse Dollar (1 byte) - "$"
    std::memcpy(&data.packet.dollar, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    if (data.packet.dollar != '$') {
        std::cerr << "Invalid dollar character" << std::endl;
        return false;
    }

    // Parse Length (4 bytes)
    std::memcpy(&data.packet.length, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Parse Aux Data (12 bytes)
    std::memcpy(&data.packet.AuxData, buffer + offset, sizeof(uint8_t[12]));
    offset += sizeof(uint8_t[12]);

    // Parse Traffic Light Data (6 bytes total)
    std::memcpy(&data.packet.traffic_light_data.traffic_light_index, buffer + offset, 12);
    offset += 12;

    std::memcpy(&data.packet.traffic_light_data.traffic_light_type, buffer + offset, sizeof(int16_t));
    offset += sizeof(int16_t);

    std::memcpy(&data.packet.traffic_light_data.traffic_light_status, buffer + offset, sizeof(int16_t));
    offset += sizeof(int16_t);

    // Parse Tail (2 bytes) - 0x0D 0x0A
    std::memcpy(&data.packet.tail, buffer + offset, sizeof(uint8_t[2]));
    offset += sizeof(uint8_t[2]);
    if (data.packet.tail[0] != 0x0D || data.packet.tail[1] != 0x0A) {
        std::cerr << "Invalid tail" << std::endl;
        return false;
    }

    return true;
}