#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "sensors/imu.hpp"

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7505" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        PrintUsage(argv[0]);
        return -1;
    }

    const std::string ip_address = argv[1];
    const int port = std::stoi(argv[2]);

    try
    {
        IMU imu(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        while (true)
        {
            IMU::IMUData data;
            if (imu.GetIMUData(data))
            {
                std::cout << "===== IMU Data =====" << std::endl;
                std::cout << "Linear Acceleration (m/s^2):" << std::endl;
                std::cout << "\tX: " << data.linear_acceleration_x << std::endl;
                std::cout << "\tY: " << data.linear_acceleration_y << std::endl;
                std::cout << "\tZ: " << data.linear_acceleration_z << std::endl;
                std::cout << "Angular Velocity (rad/s):" << std::endl;
                std::cout << "\tX: " << data.angular_velocity_x << std::endl;
                std::cout << "\tY: " << data.angular_velocity_y << std::endl;
                std::cout << "\tZ: " << data.angular_velocity_z << std::endl;
                std::cout << "Orientation (Quaternion):" << std::endl;
                std::cout << "\tX: " << data.x << std::endl;
                std::cout << "\tY: " << data.y << std::endl;
                std::cout << "\tZ: " << data.z << std::endl;
                std::cout << "\tW: " << data.w << std::endl;
                std::cout << "===================" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}