#include "sensors/gps.hpp"
#include <iostream>
#include <thread>
#include <iomanip>
#include <atomic>

#ifndef _WIN32
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#endif

std::atomic<bool> is_running(true);

#ifndef _WIN32
ros::Publisher gps_pub;
ros::Publisher map_pub;
ros::Publisher gps_marker_pub;
tf2_ros::TransformBroadcaster* br = nullptr;

// 지도 원점의 GPS 좌표 (설정 필요)
const double ORIGIN_LAT = 37.384215;  // 예시 좌표
const double ORIGIN_LON = 127.123123; // 예시 좌표

// GPS 좌표를 지도 좌표로 변환 (Mercator 투영 사용)
void gpsToMapCoordinates(double lat, double lon, double& x, double& y) {
    const double EARTH_RADIUS = 6378137.0;  // 지구 반지름 (미터)
    
    // 원점으로부터의 차이를 라디안으로 변환
    double dLat = (lat - ORIGIN_LAT) * M_PI / 180.0;
    double dLon = (lon - ORIGIN_LON) * M_PI / 180.0;
    
    // Mercator 투영 변환
    x = EARTH_RADIUS * dLon;
    y = EARTH_RADIUS * log(tan(M_PI/4 + dLat/2));
}

void PublishGPSData(const GPS::GPSData& data)
{
    // GPS 데이터 발행
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.frame_id = "map";
    gps_msg.header.stamp.sec = static_cast<uint32_t>(data.timestamp);
    gps_msg.header.stamp.nsec = static_cast<uint32_t>((data.timestamp - static_cast<uint32_t>(data.timestamp)) * 1e9);
    gps_msg.latitude = data.latitude;
    gps_msg.longitude = data.longitude;
    gps_msg.altitude = data.altitude;
    if(gps_pub.getNumSubscribers() != 0)
    {
        gps_pub.publish(gps_msg);
    }

    // GPS 마커 생성
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp.sec = static_cast<uint32_t>(data.timestamp);
    marker.header.stamp.nsec = static_cast<uint32_t>((data.timestamp - static_cast<uint32_t>(data.timestamp)) * 1e9);
    marker.ns = "gps";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // GPS 좌표를 지도 좌표로 변환
    double x, y;
    gpsToMapCoordinates(data.latitude, data.longitude, x, y);
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = data.altitude;
    
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    if(gps_marker_pub.getNumSubscribers() != 0)
    {
        gps_marker_pub.publish(marker);
    }

    // TF 브로드캐스트
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp.sec = static_cast<uint32_t>(data.timestamp);
    transformStamped.header.stamp.nsec = static_cast<uint32_t>((data.timestamp - static_cast<uint32_t>(data.timestamp)) * 1e9);
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "gps";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = data.altitude;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    
    br->sendTransform(transformStamped);
}

// 지도 로드 및 발행 함수
void publishMap() {
    if(map_pub.getNumSubscribers() == 0)
    {
        return;
    }
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = ros::Time::now();
    
    // 지도 메타데이터 설정
    map.info.resolution = 0.1;  // 10cm per pixel
    map.info.width = 1000;      // 100m
    map.info.height = 1000;     // 100m
    map.info.origin.position.x = -50;  // 중심이 원점이 되도록
    map.info.origin.position.y = -50;
    
    // 지도 데이터 로드 (예: PNG 파일에서)
    cv::Mat map_image = cv::imread("map.png", cv::IMREAD_GRAYSCALE);
    if (!map_image.empty()) {
        map.data.resize(map.info.width * map.info.height);
        for (int y = 0; y < map_image.rows; y++) {
            for (int x = 0; x < map_image.cols; x++) {
                int index = y * map.info.width + x;
                map.data[index] = map_image.at<uchar>(y, x) < 128 ? 100 : 0;
            }
        }
    }
    
    map_pub.publish(map);
}
#endif

void OnGPSData(const GPS::GPSData& data)
{
    std::cout << "===== GPS Data =====" << std::endl;
    std::cout << "\tSentence Type: " << data.sentence_type << std::endl;
    std::cout << "\tTimestamp: " << std::fixed << std::setprecision(8) 
              << data.timestamp << std::resetiosflags(std::ios::fixed) << std::endl;
    std::cout << "\tLatitude: " << std::fixed << std::setprecision(8) << data.latitude << std::endl;
    std::cout << "\tLatitude Direction: " << data.lat_dir << std::endl;
    std::cout << "\tLongitude: " << std::fixed << std::setprecision(8) << data.longitude << std::endl;
    std::cout << "\tLongitude Direction: " << data.lon_dir << std::endl;
    std::cout << "\tAltitude: " << std::fixed << std::setprecision(6) 
              << data.altitude << std::resetiosflags(std::ios::fixed) << std::endl;
    std::cout << "===================" << std::endl;

    #ifndef _WIN32
    PublishGPSData(data);
    #endif
}

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7506" << std::endl;
}

int main(int argc, char* argv[])
{
    #ifndef _WIN32
    ros::init(argc, argv, "gps_receiver_node");
    ros::NodeHandle nh("~");
    
    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7506);
    
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    gps_marker_pub = nh.advertise<visualization_msgs::Marker>("/gps/position_marker", 1);
    br = new tf2_ros::TransformBroadcaster;
    
    // 지도 발행
    publishMap();
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
        GPS gps(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        gps.RegisterCallback(OnGPSData);

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