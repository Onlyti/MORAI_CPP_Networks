#include "sensors/traffic_light.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>
#include <sstream>

#ifndef _WIN32
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#endif

std::atomic<bool> is_running(true);

#ifndef _WIN32
ros::Publisher traffic_sign_pub;
ros::Publisher traffic_text_pub;

void PublishTrafficSign(const MoraiCppUdp::TrafficLight::TrafficLightData& data)
{
    // Text message
    std_msgs::String text_msg;
    std::stringstream ss;
    ss << "Traffic Light ID: " << data.traffic_light_index << "\n";
    ss << "Type: " << data.traffic_light_type << " ";
    
    // Convert type to string
    if(data.traffic_light_type == 0)
        ss << "(RYG)";
    else if(data.traffic_light_type == 1)
        ss << "(RYGLeft)";
    else if(data.traffic_light_type == 2)
        ss << "(RYGLeftG)";
    else if(data.traffic_light_type == 100)
        ss << "(YYY)";
    
    ss << "\nStatus: " << data.traffic_light_status << " ";
    
    // Convert status to string
    if(data.traffic_light_status == 1)
        ss << "(Red)";
    else if(data.traffic_light_status == 4)
        ss << "(Yellow)";
    else if(data.traffic_light_status == 16)
        ss << "(Green)";
    else if(data.traffic_light_status == 32)
        ss << "(GreenLeft)";
    else if(data.traffic_light_status == 48)
        ss << "(Green with GreenLeft)";
    else if(data.traffic_light_status == 20)
        ss << "(Yellow with Green)";
    else if(data.traffic_light_status == 36)
        ss << "(Yellow with GreenLeft)";
    else if(data.traffic_light_status == 5)
        ss << "(Red with Yellow)";
    else if(data.traffic_light_status == -1)
        ss << "(default)";
    
    text_msg.data = ss.str();
    traffic_text_pub.publish(text_msg);

    // Visual marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "traffic_light";
    marker.id = static_cast<int>(std::stoi(data.traffic_light_index));
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set position (assuming we have position data)
    marker.pose.position.x = 0;  // Replace with actual position
    marker.pose.position.y = 0;
    marker.pose.position.z = 2;

    // Set size
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1.0;

    // Set color based on status
    marker.color.a = 1.0;
    if(data.traffic_light_status == 1) {  // Red
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else if(data.traffic_light_status == 4) {  // Yellow
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if(data.traffic_light_status == 16) {  // Green
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    traffic_sign_pub.publish(marker);
}
#endif

void OnTrafficLightData(const MoraiCppUdp::TrafficLight::TrafficLightData& data)
{
    // Console output
    std::cout << "===== Traffic Light Data =====" << std::endl;
    std::cout << "ID: " << data.traffic_light_index << std::endl;
    std::cout << "Type: " << data.traffic_light_type;
    if(data.traffic_light_type == 0)
        std::cout << " (RYG)";
    else if(data.traffic_light_type == 1)
        std::cout << " (RYGLeft)";
    else if(data.traffic_light_type == 2)
        std::cout << " (RYGLeftG)";
    else if(data.traffic_light_type == 100)
        std::cout << " (YYY)";
    std::cout << std::endl;

    std::cout << "Status: " << data.traffic_light_status;
    if(data.traffic_light_status == 1)
        std::cout << " (Red)";
    else if(data.traffic_light_status == 4)
        std::cout << " (Yellow)";
    else if(data.traffic_light_status == 16)
        std::cout << " (Green)";
    else if(data.traffic_light_status == 32)
        std::cout << " (GreenLeft)";
    else if(data.traffic_light_status == 48)
        std::cout << " (Green with GreenLeft)";
    else if(data.traffic_light_status == 20)
        std::cout << " (Yellow with Green)";
    else if(data.traffic_light_status == 36)
        std::cout << " (Yellow with GreenLeft)";
    else if(data.traffic_light_status == 5)
        std::cout << " (Red with Yellow)";
    else if(data.traffic_light_status == -1)
        std::cout << " (default)";
    std::cout << std::endl;
    std::cout << "========================" << std::endl;

    #ifndef _WIN32
    PublishTrafficSign(data);
    #endif
}

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7094" << std::endl;
}

int main(int argc, char* argv[])
{
    #ifndef _WIN32
    ros::init(argc, argv, "traffic_light_receiver_node");
    ros::NodeHandle nh("~");
    
    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7094);
    
    traffic_sign_pub = nh.advertise<visualization_msgs::Marker>("/traffic_light/marker", 1);
    traffic_text_pub = nh.advertise<std_msgs::String>("/traffic_light/status", 1);
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
        MoraiCppUdp::TrafficLight traffic_light(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        traffic_light.RegisterCallback(OnTrafficLightData);

        #ifndef _WIN32
        ros::Rate rate(30);  // 30Hz
        while (ros::ok() && is_running)
        {
            ros::spinOnce();
            rate.sleep();
        }
        #else
        while (is_running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30Hz
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