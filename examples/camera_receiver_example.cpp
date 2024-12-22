#include <iostream>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <mutex>
#include <cmath>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensors/camera.hpp"

class CameraNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher raw_image_pub_;
    image_transport::Publisher bbox_image_pub_;
    ros::Publisher bbox_marker_pub_;

    Camera::CameraData last_camera_data_;
    Camera::BoundingBoxData last_bbox_data_;
    std::mutex data_mutex_;
    const double MAX_TIME_DIFF = 0.01; // 10ms
    std::unique_ptr<Camera> camera_;

public:
    CameraNode() : nh_("~"), it_(nh_) {
        // ROS 파라미터 읽기
        std::string ip_address;
        int port;
        nh_.param<std::string>("ip_address", ip_address, "127.0.0.1");
        nh_.param<int>("port", port, 7777);

        // 퍼블리셔 설정
        raw_image_pub_ = it_.advertise("raw_image", 1);
        bbox_image_pub_ = it_.advertise("bbox_image", 1);
        bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("bbox_markers", 1);

        // 카메라 객체 생성 및 콜백 등록
        camera_ = std::make_unique<Camera>(ip_address, port);
        camera_->RegisterCameraCallback(
            std::bind(&CameraNode::OnCameraData, this, std::placeholders::_1));
        camera_->RegisterBoundingBoxCallback(
            std::bind(&CameraNode::OnBoundingBoxData, this, std::placeholders::_1));

        ROS_INFO("Camera node initialized with IP: %s, Port: %d", ip_address.c_str(), port);
    }

    void OnCameraData(const Camera::CameraData& camera_data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_camera_data_ = camera_data;
        
        // Raw 이미지 퍼블리시
        PublishImage(camera_data.image_data, raw_image_pub_, camera_data.timestamp);
        
        // 시간 동기화 확인 및 처리
        if (!last_bbox_data_.bbox_2d.empty() && 
            std::abs(camera_data.timestamp - last_bbox_data_.timestamp) < MAX_TIME_DIFF)
        {
            ProcessSyncData(camera_data.image_data, last_bbox_data_);
        }
    }

    void OnBoundingBoxData(const Camera::BoundingBoxData& bbox_data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_bbox_data_ = bbox_data;
        
        // 바운딩 박스 마커 퍼블리시
        PublishBBoxMarkers(bbox_data);
        
        // 시간 동기화 확인 및 처리
        if (!last_camera_data_.image_data.empty() && 
            std::abs(bbox_data.timestamp - last_camera_data_.timestamp) < MAX_TIME_DIFF)
        {
            ProcessSyncData(last_camera_data_.image_data, bbox_data);
        }
    }

private:
    void ProcessSyncData(const cv::Mat& image, const Camera::BoundingBoxData& bbox_data) {
        if (!image.empty()) {
            cv::Mat display_image = image.clone();

            // 이미지에 2D 바운딩 박스 그리기
            for (size_t i = 0; i < bbox_data.bbox_2d.size(); i++) {
                const auto& bbox = bbox_data.bbox_2d[i];
                const auto& cls = bbox_data.classes[i];

                // 바운딩 박스 그리기
                cv::rectangle(display_image, 
                            cv::Point(bbox.x_min, bbox.y_min),
                            cv::Point(bbox.x_max, bbox.y_max), 
                            cv::Scalar(0, 255, 0), 2);

                // 클래스 정보 표시
                std::string class_text = "Class: " + std::to_string(cls.group_id) + "," +
                                       std::to_string(cls.class_id) + "," +
                                       std::to_string(cls.sub_class_id);
                cv::putText(display_image, class_text,
                           cv::Point(bbox.x_min, bbox.y_min - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5,
                           cv::Scalar(0, 255, 0), 1);
            }

            // 바운딩 박스가 그려진 이미지 퍼블리시
            PublishImage(display_image, bbox_image_pub_, bbox_data.timestamp);
        }
    }

    void PublishImage(const cv::Mat& image, const image_transport::Publisher& pub, double timestamp) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = ros::Time(timestamp);
        pub.publish(msg);
    }

    void PublishBBoxMarkers(const Camera::BoundingBoxData& bbox_data) {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < bbox_data.bbox_3d.size(); i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_frame";
            marker.header.stamp = ros::Time(bbox_data.timestamp);
            marker.ns = "bboxes";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 3D 바운딩 박스 중심점 설정
            marker.pose.position.x = bbox_data.bbox_3d[i].x;
            marker.pose.position.y = bbox_data.bbox_3d[i].y;
            marker.pose.position.z = bbox_data.bbox_3d[i].z;
            
            // 크기는 임의로 설정 (실제 데이터에 맞게 수정 필요)
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            
            marker_array.markers.push_back(marker);
        }
        
        bbox_marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_receiver_node");
    
    try {
        CameraNode node;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error: %s", e.what());
        return -1;
    }

    return 0;
}