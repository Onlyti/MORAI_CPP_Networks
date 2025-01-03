#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include "sensors/object_info.hpp"

#ifndef _WIN32
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#endif

std::atomic<bool> is_running(true);

// Object 타입을 문자열로 변환하는 헬퍼 함수
std::string ObjectTypeToString(int16_t type);

#ifndef _WIN32
ros::Publisher marker_pub;

// 헬퍼 함수 구현
std::string ObjectTypeToString(int16_t type) {
    switch(type) {
        case MoraiCppUdp::ObjectInfo::TYPE_EGO: return "EGO";
        case MoraiCppUdp::ObjectInfo::TYPE_PEDESTRIAN: return "PEDESTRIAN";
        case MoraiCppUdp::ObjectInfo::TYPE_VEHICLE: return "VEHICLE";
        case MoraiCppUdp::ObjectInfo::TYPE_OBJECT: return "OBJECT";
        default: return "UNKNOWN";
    }
}

void PublishObjectMarkers(const std::vector<MoraiCppUdp::ObjectInfo::ObjectData>& objects)
{
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < objects.size(); ++i)
    {
        const auto& obj = objects[i];
        
        // Skip empty objects
        if (obj.obj_id == 0 && obj.size_x == 0 && obj.size_y == 0 && obj.size_z == 0)
            continue;

        // Main body marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "objects";
        marker.id = obj.obj_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Position and orientation
        marker.pose.position.x = obj.pos_x;
        marker.pose.position.y = obj.pos_y;
        marker.pose.position.z = obj.pos_z;
        
        double heading_rad = obj.heading * M_PI / 180.0;
        marker.pose.orientation.w = cos(heading_rad / 2);
        marker.pose.orientation.z = sin(heading_rad / 2);
        
        // Size
        marker.scale.x = obj.size_x;
        marker.scale.y = obj.size_y;
        marker.scale.z = obj.size_z;
        
        // Color based on object type
        marker.color.a = 0.7;
        switch(obj.obj_type)
        {
            case MoraiCppUdp::ObjectInfo::TYPE_EGO:
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                break;
            case MoraiCppUdp::ObjectInfo::TYPE_PEDESTRIAN:
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                break;
            case MoraiCppUdp::ObjectInfo::TYPE_VEHICLE:
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
                break;
            default:
                marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5;
        }
        
        marker_array.markers.push_back(marker);
        
        // Text marker for object info
        visualization_msgs::Marker text_marker = marker;
        text_marker.id = obj.obj_id + 10000;  // Offset to avoid ID collision
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = "ID: " + std::to_string(obj.obj_id) + "\n" +
                          "Type: " + ObjectTypeToString(obj.obj_type) + "\n" +
                          "Speed: " + std::to_string(sqrt(pow(obj.velocity_x,2) + 
                                                        pow(obj.velocity_y,2))) + " km/h";
        text_marker.pose.position.z += obj.size_z + 0.5;  // Place text above object
        text_marker.scale.z = 0.5;  // Text size
        text_marker.color.r = 1.0;  // White text
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        marker_array.markers.push_back(text_marker);
    }
    
    marker_pub.publish(marker_array);
}
#endif

// Vector3 형식 출력을 위한 헬퍼 함수
void PrintVector3(const std::string& name, float x, float y, float z, 
                 const std::string& unit = "", int precision = 3) {
    std::cout << name << ":\n"
              << "\tx: " << std::fixed << std::setprecision(precision) << x << unit << "\n"
              << "\ty: " << std::fixed << std::setprecision(precision) << y << unit << "\n"
              << "\tz: " << std::fixed << std::setprecision(precision) << z << unit << "\n";
}

void OnObjectInfo(const MoraiCppUdp::ObjectInfo::ObjectsData& data)
{
    // Clear screen for better visibility
    #ifdef _WIN32
    if (system("cls") != 0) {
        std::cerr << "Failed to clear screen" << std::endl;
    }
    #else
    if (system("clear") != 0) {
        std::cerr << "Failed to clear screen" << std::endl;
    }
    #endif

    std::cout << "\n=== Object Information ===" << std::endl;
    std::cout << "Timestamp: " << data.timestamp.sec << "." << data.timestamp.nsec << " ns" << std::endl;
    std::cout << "Number of objects: " << data.objects.size() << std::endl;

    for (size_t i = 0; i < data.objects.size(); ++i)
    {
        const auto& obj = data.objects[i];
        if(obj.obj_id == 0)
        {
            continue;
        }
        std::cout << "\nObject #" << i + 1 << std::endl;
        std::cout << "ID: " << obj.obj_id << std::endl;
        std::cout << "Type: " << obj.obj_type 
                  << " (" << ObjectTypeToString(obj.obj_type) << ")" << std::endl;

        PrintVector3("Position", obj.pos_x, obj.pos_y, obj.pos_z, " m");
        std::cout << "Heading: " << obj.heading << " deg" << std::endl;
        PrintVector3("Size", obj.size_x, obj.size_y, obj.size_z, " m");
        
        if (obj.obj_type == MoraiCppUdp::ObjectInfo::TYPE_VEHICLE) {
            std::cout << "Vehicle Specific Data:" << std::endl;
            std::cout << "\tOverhang: " << obj.overhang << " m" << std::endl;
            std::cout << "\tWheelbase: " << obj.wheelbase << " m" << std::endl;
            std::cout << "\tRear Overhang: " << obj.rear_overhang << " m" << std::endl;
        }

        PrintVector3("Velocity", obj.velocity_x, obj.velocity_y, obj.velocity_z, " km/h");
        PrintVector3("Acceleration", obj.accel_x, obj.accel_y, obj.accel_z, " m/s²");

        if (obj.obj_type == MoraiCppUdp::ObjectInfo::TYPE_VEHICLE) {
            std::cout << "MGeo Link ID: " << obj.link_id << std::endl;
        }
        
        std::cout << "------------------------" << std::endl;
    }

    #ifndef _WIN32
    PublishObjectMarkers(data.objects);
    #endif
}

void PrintUsage() {
    std::cout << "Usage: object_info_receiver <ip_address> <port>" << std::endl;
    std::cout << "Example: object_info_receiver 127.0.0.1 7777" << std::endl;
}

int main(int argc, char** argv)
{
    #ifndef _WIN32
    ros::init(argc, argv, "object_info_receiver_node");
    ros::NodeHandle nh("~");
    
    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7507);
    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/objects/markers", 10);
    #else
    if (argc != 3) {
        PrintUsage();
        return 1;
    }
    std::string ip_address = argv[1];
    uint16_t port = static_cast<uint16_t>(std::stoi(argv[2]));
    #endif

    try
    {
        MoraiCppUdp::ObjectInfo object_info(ip_address, port);
        std::cout << "Connecting to " << ip_address << ":" << port << std::endl;
        
        object_info.RegisterCallback(OnObjectInfo);

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
        return 1;
    }

    return 0;
} 