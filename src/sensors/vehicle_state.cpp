#include "sensors/vehicle_state.hpp"

#include <cstring>
#include <iomanip>
#include <iostream>

VehicleState::VehicleState(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false), is_data_received_(false)
{
    is_running_ = true;
    thread_vehicle_state_receiver_ = std::thread(&VehicleState::ThreadVehicleStateReceiver, this);
}

VehicleState::~VehicleState()
{
    is_running_ = false;
    if (thread_vehicle_state_receiver_.joinable())
    {
        thread_vehicle_state_receiver_.join();
    }
    Close();
}

bool VehicleState::GetVehicleState(VehicleData& data)
{
    std::lock_guard<std::mutex> lock(mutex_vehicle_data_);
    if (!is_data_received_)
    {
        return false;
    }

    data = vehicle_data_;
    is_data_received_ = false;
    return true;
}

void VehicleState::ThreadVehicleStateReceiver()
{
    char packet_buffer[PACKET_SIZE];

    while (is_running_)
    {
        try
        {
            size_t received_size = 0;
            if (!Receive(packet_buffer, PACKET_SIZE, received_size))
            {
                std::cout << "received_size: " << received_size << std::endl;
                std::cerr << "Failed to receive vehicle state data" << std::endl;
                continue;
            }

            if (received_size != PACKET_SIZE)
            {
                std::cerr << "Received unexpected packet size: " << received_size
                          << " (expected: " << PACKET_SIZE << ")" << std::endl;
                continue;
            }
            
            memset(&packet_data_, 0, sizeof(VehicleStatePacketStruct));
            if (ParseVehicleState(packet_buffer, received_size, packet_data_))
            {
                // std::lock_guard<std::mutex> lock(mutex_vehicle_data_);
                mutex_vehicle_data_.lock();
                memcpy(&vehicle_data_, &packet_data_.packet.vehicle_data, sizeof(VehicleData));
                is_data_received_ = true;
                mutex_vehicle_data_.unlock();
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Vehicle state error: " << e.what() << std::endl;
            continue;
        }
    }
}

bool VehicleState::ParseVehicleState(const char* buffer, size_t size, VehicleStatePacketStruct& data)
{
    if (size != PACKET_SIZE)
    {
        return false;
    }

    size_t offset = 0;

    // std::memcpy(data.data, buffer, sizeof(VehicleStatePacketStruct));
    
    // Parse Sharp (1 byte) - "#"
    std::memcpy(&data.packet.sharp, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // Parse Header (9 bytes) - "MoraiInfo"
    std::memcpy(&data.packet.header, buffer + offset, sizeof(char[9]));
    offset += sizeof(char[9]);

    // Parse Dollar (1 byte) - "$"
    std::memcpy(&data.packet.dollar, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // Parse Length (4 bytes)
    std::memcpy(&data.packet.length, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Parse Aux Data (12 bytes)
    std::memcpy(&data.packet.AuxData, buffer + offset, sizeof(uint8_t[12]));
    offset += sizeof(uint8_t[12]);

    // Parse Vehicle Data (180 bytes)
    std::memcpy(&data.packet.vehicle_data, buffer + offset, sizeof(VehicleData));
    offset += sizeof(VehicleData);

    // Parse Tail (2 bytes)
    std::memcpy(&data.packet.tail, buffer + offset, sizeof(uint8_t[2]));
    offset += sizeof(uint8_t[2]);

    // // Parse Timestamp (8 bytes)
    // std::memcpy(&data.timestamp.seconds, buffer + offset, sizeof(uint32_t));
    // offset += sizeof(uint32_t);
    // std::memcpy(&data.timestamp.nanoseconds, buffer + offset, sizeof(uint32_t));
    // offset += sizeof(uint32_t);

    // // Parse Control Mode (1 byte)
    // uint8_t ctrl_mode;
    // std::memcpy(&ctrl_mode, buffer + offset, sizeof(uint8_t));
    // data.ctrl_mode = static_cast<ControlMode>(ctrl_mode);
    // offset += sizeof(uint8_t);

    // // Parse Gear (1 byte)
    // uint8_t gear;
    // std::memcpy(&gear, buffer + offset, sizeof(uint8_t));
    // data.gear = static_cast<GearMode>(gear);
    // offset += sizeof(uint8_t);

    // // Parse Signed Velocity (4 bytes)
    // std::memcpy(&data.signed_velocity, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Map Data ID (4 bytes)
    // std::memcpy(&data.map_data_id, buffer + offset, sizeof(int32_t));
    // offset += sizeof(int32_t);

    // // Parse Accel Input (4 bytes)
    // std::memcpy(&data.accel_input, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Brake Input (4 bytes)
    // std::memcpy(&data.brake_input, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Size XYZ (12 bytes)
    // std::memcpy(&data.size, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Overhang (4 bytes)
    // std::memcpy(&data.overhang, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Wheelbase (4 bytes)
    // std::memcpy(&data.wheelbase, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Rear Overhang (4 bytes)
    // std::memcpy(&data.rear_overhang, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse Position XYZ (12 bytes)
    // std::memcpy(&data.position, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Rotation (Roll/Pitch/Heading) (12 bytes)
    // std::memcpy(&data.rotation, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Velocity XYZ (12 bytes)
    // std::memcpy(&data.velocity, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Angular Velocity XYZ (12 bytes)
    // std::memcpy(&data.angular_velocity, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Acceleration XYZ (12 bytes)
    // std::memcpy(&data.acceleration, buffer + offset, sizeof(Vector3));
    // offset += sizeof(Vector3);

    // // Parse Steering (4 bytes)
    // std::memcpy(&data.steering, buffer + offset, sizeof(float));
    // offset += sizeof(float);

    // // Parse MGeo Link ID (38 bytes)
    // char link_id[39] = {0};  // +1 for null terminator
    // std::memcpy(link_id, buffer + offset, 38);
    // data.mgeo_link_id = std::string(link_id);
    // offset += 38;

    return true;
}