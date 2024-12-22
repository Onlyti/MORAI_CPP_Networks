#include "sensors/imu.hpp"
#include <cstring>
#include <iostream>

IMU::IMU(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false)
{
    is_running_ = true;
    is_imu_data_received_ = false;
    thread_imu_udp_receiver_ = std::thread(&IMU::ThreadIMUUdpReceiver, this);
    thread_imu_udp_receiver_.detach();
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
    return std::move(GetLatestIMUData(data));
}

bool IMU::GetLatestIMUData(IMUData& data)
{
    std::lock_guard<std::mutex> lock(mutex_imu_data_);
    data = imu_data_;
    return true;
}

void IMU::ThreadIMUUdpReceiver()
{
    char packet_buffer[PACKET_SIZE];

    while (is_running_)
    {
        size_t received_size = 0;
        if (!Receive(packet_buffer, PACKET_SIZE, received_size))
        {
            std::cerr << "Failed to receive IMU data" << std::endl;
            continue;
        }

        IMUData temp_data;
        if (ParseIMUData(packet_buffer, received_size, temp_data))
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (imu_callback_)
            {
                imu_callback_(temp_data);
            }
        }
    }
}

bool IMU::ParseIMUData(const char* buffer, size_t size, IMUData& data)
{
    // if (size != PACKET_SIZE)
    // {
    //     return false;
    // }

    size_t offset = 0;

    // Skip header "#IMUData$" (9 bytes)
    offset += 9;

    // Skip size field (4 bytes)
    offset += 4;

    // Skip aux data (12 bytes)
    offset += 20;  // Actually 20 bytes in protocol

    // Parse orientation (Quaternion)
    std::memcpy(&data.w, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.x, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.y, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.z, buffer + offset, sizeof(double));
    offset += sizeof(double);

    // Parse angular velocity
    std::memcpy(&data.angular_velocity_x, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.angular_velocity_y, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.angular_velocity_z, buffer + offset, sizeof(double));
    offset += sizeof(double);

    // Parse linear acceleration
    std::memcpy(&data.linear_acceleration_x, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.linear_acceleration_y, buffer + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&data.linear_acceleration_z, buffer + offset, sizeof(double));
    offset += sizeof(double);

    // Check tail (2 bytes)
    if (buffer[size-2] != 0x0D || buffer[size-1] != 0x0A)
    {
        std::cerr << "Invalid tail marker" << std::endl;
        return false;
    }

    return true;
}