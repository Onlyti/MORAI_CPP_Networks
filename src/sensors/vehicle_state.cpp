#include "sensors/vehicle_state.hpp"

#include <cstring>
#include <iomanip>
#include <iostream>

VehicleState::VehicleState(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false)
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
                std::cerr << "Failed to receive vehicle state data" << std::endl;
                continue;
            }
            
            memset(&packet_data_, 0, sizeof(VehicleStatePacketStruct));
            if (ParseVehicleState(packet_buffer, received_size, packet_data_))
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (vehicle_state_callback_) {
                    vehicle_state_callback_(packet_data_.packet.vehicle_data);
                }
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

    // Parse Vehicle - Timestamp
    std::memcpy(&data.packet.vehicle_data.timestamp.seconds, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    std::memcpy(&data.packet.vehicle_data.timestamp.nanoseconds, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Parse Vehicle - Control Mode
    std::memcpy(&data.packet.vehicle_data.ctrl_mode, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // Parse Vehicle - Gear
    std::memcpy(&data.packet.vehicle_data.gear, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    // Parse Vehicle - Signed Velocity
    std::memcpy(&data.packet.vehicle_data.signed_velocity, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Map Data ID
    std::memcpy(&data.packet.vehicle_data.map_data_id, buffer + offset, sizeof(int32_t));
    offset += sizeof(int32_t);

    // Parse Vehicle - Accel Input
    std::memcpy(&data.packet.vehicle_data.accel_input, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Brake Input  
    std::memcpy(&data.packet.vehicle_data.brake_input, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Size XYZ
    std::memcpy(&data.packet.vehicle_data.size, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Overhang
    std::memcpy(&data.packet.vehicle_data.overhang, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Wheelbase
    std::memcpy(&data.packet.vehicle_data.wheelbase, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Rear Overhang
    std::memcpy(&data.packet.vehicle_data.rear_overhang, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Position
    std::memcpy(&data.packet.vehicle_data.position, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Rotation
    std::memcpy(&data.packet.vehicle_data.rotation, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Velocity
    std::memcpy(&data.packet.vehicle_data.velocity, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Angular Velocity
    std::memcpy(&data.packet.vehicle_data.angular_velocity, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Acceleration
    std::memcpy(&data.packet.vehicle_data.acceleration, buffer + offset, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Parse Vehicle - Steering
    std::memcpy(&data.packet.vehicle_data.steering, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - MGeo Link ID
    std::memcpy(&data.packet.vehicle_data.mgeo_link_id, buffer + offset, sizeof(char[38]));
    offset += sizeof(char[38]);

    // Parse Vehicle - Tire Lateral Forces
    std::memcpy(&data.packet.vehicle_data.tire_lateral_force_fl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_lateral_force_fr, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_lateral_force_rl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_lateral_force_rr, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Side Slip Angles
    std::memcpy(&data.packet.vehicle_data.side_slip_angle_fl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.side_slip_angle_fr, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.side_slip_angle_rl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.side_slip_angle_rr, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Vehicle - Tire Cornering Stiffness
    std::memcpy(&data.packet.vehicle_data.tire_cornering_stiffness_fl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_cornering_stiffness_fr, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_cornering_stiffness_rl, buffer + offset, sizeof(float));
    offset += sizeof(float);
    std::memcpy(&data.packet.vehicle_data.tire_cornering_stiffness_rr, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Parse Tail (2 bytes)
    std::memcpy(&data.packet.tail, buffer + offset, sizeof(uint8_t[2]));
    offset += sizeof(uint8_t[2]);


    return true;
}