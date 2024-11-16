#include "sensors/imu.hpp"

#include <iomanip>
#include <iostream>

IMU::IMU(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false)
{
    is_running_ = true;
    is_imu_data_received_ = false;
    thread_imu_udp_receiver_ = std::thread(&IMU::ThreadIMUUdpReceiver, this);
}

IMU::~IMU()
{
    is_running_ = false;
    if (thread_imu_udp_receiver_.joinable())
    {
        thread_imu_udp_receiver_.join();
    }
    Close();
}

bool IMU::GetIMUData(IMUData& data)
{
    if (!is_imu_data_received_)
    {
        return false;
    }
    is_imu_data_received_ = false;
    return GetLatestIMUData(data);
}

bool IMU::GetLatestIMUData(IMUData& data)
{
    std::lock_guard<std::mutex> lock(mutex_imu_data_);
    data = imu_data_;
    return true;
}

void IMU::ThreadIMUUdpReceiver()
{
    char packet_buffer[MAX_PACKET_SIZE];

    while (is_running_)
    {
        // Receive data from UDP
        size_t received_size = 0;
        if (!Receive(packet_buffer, MAX_PACKET_SIZE, received_size))
        {
            std::cerr << "Failed to receive IMU data" << std::endl;
            continue;
        }

        // Check header: "#IMUData$"
        IMUPacketHeader header;
        std::memcpy(&header, packet_buffer, sizeof(IMUPacketHeader));
        std::string header_str(reinterpret_cast<char*>(header.header), 9);
        if (header_str != "#IMUData$")
        {
            std::cerr << "Invalid header" << std::endl;
            continue;
        }

        // Parse IMU data
        IMUData temp_data;
        if (ParseIMUData(packet_buffer, received_size, temp_data))
        {
            std::lock_guard<std::mutex> lock(mutex_imu_data_);
            imu_data_ = std::move(temp_data);
            is_imu_data_received_ = true;
        }
    }
}

bool IMU::ParseIMUData(const char* buffer, size_t size, IMUData& data)
{
    if (size < sizeof(107))
    {
        std::cerr << "Received IMU data is too small" << std::endl;
        return false;
    }

    const IMUPacketStruct* imu_packet = reinterpret_cast<const IMUPacketStruct*>(buffer);

    // Check tail
    if (imu_packet->packet.tail[0] != 0x0D || imu_packet->packet.tail[1] != 0x0A)
    {
        std::cerr << "Invalid tail" << std::endl;
        return false;
    }

    // Parse orientation (Quaternion)
    data.w = imu_packet->packet.imu_data.orientation[0];
    data.x = imu_packet->packet.imu_data.orientation[1];
    data.y = imu_packet->packet.imu_data.orientation[2];
    data.z = imu_packet->packet.imu_data.orientation[3];

    // Parse angular velocity
    data.angular_velocity_x = imu_packet->packet.imu_data.angular_velocity[0];
    data.angular_velocity_y = imu_packet->packet.imu_data.angular_velocity[1];
    data.angular_velocity_z = imu_packet->packet.imu_data.angular_velocity[2];

    // Parse linear acceleration
    data.linear_acceleration_x = imu_packet->packet.imu_data.linear_acceleration[0];
    data.linear_acceleration_y = imu_packet->packet.imu_data.linear_acceleration[1];
    data.linear_acceleration_z = imu_packet->packet.imu_data.linear_acceleration[2];

    return true;
}