#include <atomic>
#include <iostream>
#include <thread>
#include "sensors/camera.hpp"

#ifndef _WIN32
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#endif

std::atomic<bool> is_running(true);

#ifndef _WIN32
image_transport::Publisher image_pub;

void PublishImage(const cv::Mat& image) {
    if (image.empty()) return;

    // Convert OpenCV image to ROS message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera";

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    image_pub.publish(msg);
}
#endif

void OnCameraData(const Camera::CameraData& camera_data) {
    if (!camera_data.image_data.empty()) {
        cv::imshow("Camera Image", camera_data.image_data);
        cv::waitKey(1);

#ifndef _WIN32
        PublishImage(camera_data.image_data);
#endif
    }
}

void OnBoundingBoxData(const Camera::BoundingBoxData& bbox_data) {
    // 바운딩 박스 데이터는 현재 무시
    (void)bbox_data;
}

void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7777" << std::endl;
}

int main(int argc, char* argv[]) {
#ifndef _WIN32
    ros::init(argc, argv, "camera_receiver_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string ip_address;
    int port;
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 7777);

    image_pub = it.advertise("/camera_receiver/raw_image", 1);
#else
    if (argc != 3) {
        PrintUsage(argv[0]);
        return -1;
    }
    const std::string ip_address = argv[1];
    const int port = std::stoi(argv[2]);
#endif

    try {
        Camera camera(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        camera.RegisterCameraCallback(OnCameraData);
        camera.RegisterBoundingBoxCallback(OnBoundingBoxData);  // 바운딩 박스 콜백도 등록

#ifndef _WIN32
        ros::Rate rate(30); // 30Hz
        while (ros::ok() && is_running) {
            ros::spinOnce();
            rate.sleep();
        }
#else
        while (is_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30Hz
            if (cv::waitKey(1) == 'q') {
                is_running = false;
            }
        }
#endif

        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}