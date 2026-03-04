/**
 * @file vehicle_state2.cpp
 * @brief VehicleState2 class definition
 * @details This file contains the definition of the VehicleState2 class, which is used to receive and parse the VehicleState2 data from the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-03-03
 */
#include "sensors/vehicle_state2.hpp"

#include <cstring>
#include <iostream>

using namespace MoraiCppUdp;

VehicleState2::VehicleState2(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false) {
    std::memset(&packet_data_, 0, sizeof(VehicleStatePacketStruct));
    is_running_ = true;
    thread_vehicle_state_receiver_ = std::thread(&VehicleState2::ThreadVehicleStateReceiver, this);
}

VehicleState2::VehicleState2(const std::string& ip_address, uint16_t port, VehicleState2Callback callback)
    : UDPReceiver(ip_address, port), is_running_(false), vehicle_state2_callback_(callback) {
    std::memset(&packet_data_, 0, sizeof(VehicleStatePacketStruct));
    std::lock_guard<std::mutex> lock(callback_mutex_);
    vehicle_state2_callback_ = callback;
    is_running_ = true;
    thread_vehicle_state_receiver_ = std::thread(&VehicleState2::ThreadVehicleStateReceiver, this);
}

VehicleState2::~VehicleState2() {
    is_running_ = false;
    if (thread_vehicle_state_receiver_.joinable()) {
        thread_vehicle_state_receiver_.join();
    }
    Close();
}

bool VehicleState2::GetVehicleState(VehicleData& data) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    data = packet_data_.packet.vehicle_data;
    return true;
}

void VehicleState2::ThreadVehicleStateReceiver() {
    char packet_buffer[PACKET_SIZE];

    while (is_running_) {
        try {
            size_t received_size = 0;
            if (!Receive(packet_buffer, PACKET_SIZE, received_size)) {
                std::cerr << "Failed to receive vehicle state2 data" << std::endl;
                continue;
            }

            memset(&packet_data_, 0, sizeof(VehicleStatePacketStruct));
            if (ParseVehicleState(packet_buffer, received_size, packet_data_)) {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (vehicle_state2_callback_) {
                    vehicle_state2_callback_(packet_data_.packet.vehicle_data);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Vehicle state2 error: " << e.what() << std::endl;
            continue;
        }
    }
}

bool VehicleState2::ParseVehicleState(const char* buffer, size_t size, VehicleStatePacketStruct& data) {
    if (size != PACKET_SIZE) {
        return false;
    }

    if (buffer == nullptr) {
        return false;
    }

    size_t offset = 0;

    // Parse according to VehicleState2::VehicleData layout in vehicle_state2.hpp.
    std::memcpy(&data.packet.vehicle_data.timestamp.sec, buffer + offset, sizeof(int64_t));
    offset += sizeof(int64_t);
    std::memcpy(&data.packet.vehicle_data.timestamp.nsec, buffer + offset, sizeof(int32_t));
    offset += sizeof(int32_t);
    
    // Parse Vehicle ID
    std::memcpy(&data.packet.vehicle_data.vehicle_id, buffer + offset, sizeof(char[24]));
    offset += sizeof(char[24]);

    // Parse Vehicle position
    std::memcpy(&data.packet.vehicle_data.position, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle rotation
    std::memcpy(&data.packet.vehicle_data.rotation, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle linear velocity
    std::memcpy(&data.packet.vehicle_data.velocity, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle linear acceleration
    std::memcpy(&data.packet.vehicle_data.acceleration, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle angular velocity
    std::memcpy(&data.packet.vehicle_data.angular_velocity, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse control inputs
    std::memcpy(&data.packet.vehicle_data.accel_input, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.brake_input, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.steering, buffer + offset, sizeof(float));
    offset += sizeof(float);

    return true;
}