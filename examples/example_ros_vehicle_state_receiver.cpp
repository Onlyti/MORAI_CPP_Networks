#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>

#include "sensors/vehicle_state.hpp"

#ifndef _WIN32
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#endif

// 기어 상태를 문자열로 변환하는 헬퍼 함수
std::string GearToString(VehicleState::GearMode gear) {
    switch (gear) {
        case VehicleState::GearMode::MANUAL: return "MANUAL";
        case VehicleState::GearMode::PARKING: return "PARKING";
        case VehicleState::GearMode::REVERSE: return "REVERSE";
        case VehicleState::GearMode::NEUTRAL: return "NEUTRAL";
        case VehicleState::GearMode::DRIVE: return "DRIVE";
        case VehicleState::GearMode::LOW: return "LOW";
        default: return "UNKNOWN";
    }
}

// 제어 모드를 문자열로 변환하는 헬퍼 함수
std::string ControlModeToString(VehicleState::ControlMode mode) {
    switch (mode) {
        case VehicleState::ControlMode::MORAI_AI: return "MORAI_AI";
        case VehicleState::ControlMode::KEYBOARD: return "KEYBOARD";
        case VehicleState::ControlMode::AUTO: return "AUTO";
        default: return "UNKNOWN";
    }
}

// Vector3 출력을 위한 헬퍼 함수
void PrintVector3(const std::string& name, const VehicleState::Vector3& vec, 
                 const std::string& unit = "", int precision = 3) {
    std::cout << name << ":" << "\n"
              << "\tx: " << std::fixed << std::setprecision(precision) << vec.x
              << unit << "\n"
              << "\ty: " << std::fixed << std::setprecision(precision) << vec.y
              << unit << "\n"
              << "\tz: " << std::fixed << std::setprecision(precision) << vec.z
              << unit << "\n";
}

void ClearScreen() {
    #ifdef _WIN32
    if (system("cls") != 0) {
        std::cerr << "Failed to clear screen" << std::endl;
    }
    #else
    if (system("clear") != 0) {
        std::cerr << "Failed to clear screen" << std::endl;
    }
    #endif
}

#ifndef _WIN32
ros::Publisher odom_pub;
ros::Publisher twist_pub;
ros::Publisher pose_pub;
tf2_ros::TransformBroadcaster* br = nullptr;

void PublishVehicleState(const VehicleState::VehicleData& data)
{
    // Odometry 메시지
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "vehicle";

    // Position
    odom.pose.pose.position.x = data.position.x;
    odom.pose.pose.position.y = data.position.y;
    odom.pose.pose.position.z = data.position.z;

    // Orientation (RPY to Quaternion)
    tf2::Quaternion q;
    q.setRPY(data.rotation.x * M_PI/180.0, 
             data.rotation.y * M_PI/180.0, 
             data.rotation.z * M_PI/180.0);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Velocity
    odom.twist.twist.linear.x = data.velocity.x / 3.6;  // km/h to m/s
    odom.twist.twist.linear.y = data.velocity.y / 3.6;
    odom.twist.twist.linear.z = data.velocity.z / 3.6;
    odom.twist.twist.angular.x = data.angular_velocity.x * M_PI/180.0;  // deg/s to rad/s
    odom.twist.twist.angular.y = data.angular_velocity.y * M_PI/180.0;
    odom.twist.twist.angular.z = data.angular_velocity.z * M_PI/180.0;

    odom_pub.publish(odom);

    // TF 브로드캐스트
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header = odom.header;
    transformStamped.child_frame_id = odom.child_frame_id;
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    transformStamped.transform.rotation = odom.pose.pose.orientation;
    
    br->sendTransform(transformStamped);

    // Twist 메시지
    geometry_msgs::TwistStamped twist;
    twist.header = odom.header;
    twist.twist = odom.twist.twist;
    twist_pub.publish(twist);

    // Pose 메시지
    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pose_pub.publish(pose);
}
#endif

void OnVehicleState(const VehicleState::VehicleData& data)
{
    ClearScreen();

    std::cout << "\n=== Vehicle State Data ===" << "\n";

    // Timestamp
    std::cout << "Timestamp:" << "\n"
              << "\tSeconds: " << data.timestamp.seconds << "\n"
              << "\tNanoseconds: " << data.timestamp.nanoseconds << "\n";

    // Control Mode and Gear
    std::cout << "Control Mode: " << ControlModeToString(data.ctrl_mode) << "\n"
              << "Gear: " << GearToString(data.gear) << "\n";

    // Vehicle Speed and Map Data
    std::cout << "Signed Velocity: " << std::fixed << std::setprecision(2) 
              << data.signed_velocity << " km/h" << "\n";
    
    std::cout << "Map Data ID: " << data.map_data_id 
              << (data.map_data_id < 10000 ? " (DigitalTwin)" : " (Virtual)") << "\n";
    
    // Pedal Inputs
    std::cout << "Pedal Inputs:" << "\n"
              << "\tAccel: " << std::fixed << std::setprecision(3) 
              << data.accel_input << "\n"
              << "\tBrake: " << std::fixed << std::setprecision(3) 
              << data.brake_input << "\n";

    // Vehicle Dimensions
    PrintVector3("Size", data.size, "m");

    std::cout << "Overhang: " << std::fixed << std::setprecision(3) 
              << data.overhang << "m" << "\n";
    std::cout << "Wheelbase: " << std::fixed << std::setprecision(3) 
              << data.wheelbase << "m" << "\n";
    std::cout << "Rear Overhang: " << std::fixed << std::setprecision(3) 
              << data.rear_overhang << "m" << "\n";

    // Vehicle State
    PrintVector3("Position", data.position, "m");
    PrintVector3("Rotation", data.rotation, "deg");
    PrintVector3("Velocity", data.velocity, "km/h");
    PrintVector3("Angular Velocity", data.angular_velocity, "deg/s");
    PrintVector3("Acceleration", data.acceleration, "m/s2");

    // Steering and Link ID
    std::cout << "Steering: " << std::fixed << std::setprecision(2) 
              << data.steering << " deg" << "\n";

    std::cout << "MGeo Link ID: " << data.mgeo_link_id << "\n";

    // tire data
    std::cout << "Tire Data:" << "\n";
    std::cout << "\tLateral Force FL: " << data.tire_lateral_force_fl << " N" << "\n";
    std::cout << "\tLateral Force FR: " << data.tire_lateral_force_fr << " N" << "\n";
    std::cout << "\tLateral Force RL: " << data.tire_lateral_force_rl << " N" << "\n";
    std::cout << "\tLateral Force RR: " << data.tire_lateral_force_rr << " N" << "\n";

    // side slip angle data
    std::cout << "Side Slip Angle:" << "\n";
    std::cout << "\tSide Slip Angle FL: " << data.side_slip_angle_fl << " deg" << "\n";
    std::cout << "\tSide Slip Angle FR: " << data.side_slip_angle_fr << " deg" << "\n";
    std::cout << "\tSide Slip Angle RL: " << data.side_slip_angle_rl << " deg" << "\n";
    std::cout << "\tSide Slip Angle RR: " << data.side_slip_angle_rr << " deg" << "\n";

    // tire cornering stiffness data
    std::cout << "Tire Cornering Stiffness:" << "\n";
    std::cout << "\tTire Cornering Stiffness FL: " << data.tire_cornering_stiffness_fl << " N/deg" << "\n";
    std::cout << "\tTire Cornering Stiffness FR: " << data.tire_cornering_stiffness_fr << " N/deg" << "\n";
    std::cout << "\tTire Cornering Stiffness RL: " << data.tire_cornering_stiffness_rl << " N/deg" << "\n";
    std::cout << "\tTire Cornering Stiffness RR: " << data.tire_cornering_stiffness_rr << " N/deg" << "\n";
    
    std::cout << std::endl;

    #ifndef _WIN32
    PublishVehicleState(data);
    #endif
}

int main(int argc, char** argv)
{
    #ifndef _WIN32
    ros::init(argc, argv, "vehicle_state_receiver_node");
    ros::NodeHandle nh("~");
    
    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7777);
    
    odom_pub = nh.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/vehicle/twist", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vehicle/pose", 10);
    br = new tf2_ros::TransformBroadcaster;
    #else
    std::string ip_address = argc > 1 ? argv[1] : "127.0.0.1";
    int port = argc > 2 ? std::stoi(argv[2]) : 7777;  // Vehicle State의 기본 포트는 7777로 설정
    #endif

    try
    {
        std::cout << "Vehicle State receiver example" << std::endl;
        std::cout << "Port: " << port << std::endl;

        // UDP 수신을 위한 VehicleState 객체 생성
        VehicleState vehicle_state(ip_address, port, OnVehicleState);
        bool is_running = true;
        #ifndef _WIN32
        while(ros::ok() && is_running)
        #else
        while(is_running)
        #endif
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "Vehicle State receiver example is running" << std::endl;
            // 'q' 키를 누르면 종료 (Windows에서만 작동)
            #ifdef _WIN32
            if (GetAsyncKeyState('Q') & 0x8000)
            {
                is_running = false;
            }
            #endif
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "Vehicle State receiver example finished" << std::endl;

    return 0;
} 