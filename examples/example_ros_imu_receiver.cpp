#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <atomic>

#include "sensors/imu.hpp"

#ifndef _WIN32
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#endif

std::atomic<bool> is_running(true);

#ifndef _WIN32
ros::Publisher imu_pub;

void PublishIMUData(const MoraiCppUdp::IMU::IMUData& data)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu";
    
    // Orientation (Quaternion)
    msg.orientation.w = data.w;
    msg.orientation.x = data.x;
    msg.orientation.y = data.y;
    msg.orientation.z = data.z;
    
    // Angular Velocity
    msg.angular_velocity.x = data.angular_velocity_x;
    msg.angular_velocity.y = data.angular_velocity_y;
    msg.angular_velocity.z = data.angular_velocity_z;
    
    // Linear Acceleration
    msg.linear_acceleration.x = data.linear_acceleration_x;
    msg.linear_acceleration.y = data.linear_acceleration_y;
    msg.linear_acceleration.z = data.linear_acceleration_z;
    
    // Set covariances to unknown
    std::fill(msg.orientation_covariance.begin(), msg.orientation_covariance.end(), 0.0);
    std::fill(msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end(), 0.0);
    std::fill(msg.linear_acceleration_covariance.begin(), msg.linear_acceleration_covariance.end(), 0.0);
    
    imu_pub.publish(msg);
}
#endif

void OnIMUData(const MoraiCppUdp::IMU::IMUData& data)
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

    #ifndef _WIN32
    PublishIMUData(data);
    #endif
}

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7505" << std::endl;
}

int main(int argc, char* argv[])
{
    #ifndef _WIN32
    ros::init(argc, argv, "imu_receiver_node");
    ros::NodeHandle nh("~");
    
    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7505);
    
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
    #else
    if (argc != 3)
    {
        PrintUsage(argv[0]);
        return -1;
    }
    const std::string ip_address = argv[1];
    const int port = std::stoi(argv[2]);
    #endif

    try
    {
        MoraiCppUdp::IMU imu(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        imu.RegisterCallback(OnIMUData);

        #ifndef _WIN32
        ros::Rate rate(100);  // 100Hz
        while (ros::ok() && is_running)
        {
            ros::spinOnce();
            rate.sleep();
        }
        #else
        while (is_running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (std::cin.get() == 'q')
            {
                is_running = false;
            }
        }
        #endif
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}