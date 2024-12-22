#include "sensors/camera.hpp"

#include <iostream>

Camera::Camera(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false), camera_callback_(nullptr), bbox_callback_(nullptr) {
    is_running_ = true;
    thread_camera_udp_receiver_ = std::thread(&Camera::ThreadCameraUdpReceiver, this);
    thread_camera_udp_receiver_.detach();
}

Camera::~Camera() {
    is_running_ = false;
    if (thread_camera_udp_receiver_.joinable()) {
        thread_camera_udp_receiver_.join();
    }
    Close();
}

void Camera::ThreadCameraUdpReceiver() {
    std::vector<uint8_t> buffer;
    char packet_buffer[MAX_PACKET_SIZE];

    while (is_running_) {
        // Receive data from UDP
        size_t received_size = 0;
        if (!Receive(packet_buffer, MAX_PACKET_SIZE, received_size)) {
            std::cerr << "Failed to receive camera data" << std::endl;
            continue;
        }

        // Check header: MOR-Camera, BOX-BoundingBox
        CameraPacketHeader header;
        std::memcpy(&header, packet_buffer, sizeof(CameraPacketHeader));
        std::string header_str(reinterpret_cast<char*>(header.header), 3);

        if (header_str == "MOR") {
            CameraPacketStruct camera_packet;
            std::memcpy(&camera_packet, packet_buffer, sizeof(CameraPacketStruct));
            // Copy first 3 bytes to header
            std::memcpy(camera_packet.image.header.header, packet_buffer, 3);

            // Copy last 2 bytes to tail
            if (received_size >= 2) {
                std::memcpy(camera_packet.image.tail, packet_buffer + received_size - 2, 2);
            }

            if (received_size > 2) {
                buffer.insert(buffer.end(), packet_buffer + 19, packet_buffer + received_size - 2);
                if (camera_packet.image.tail[0] == 'E' && camera_packet.image.tail[1] == 'I') {
                    CameraData temp_data;
                    if (ParseCameraData(buffer, temp_data)) {
                        temp_data.timestamp = static_cast<double>(header.timestamp.sec) +
                                              static_cast<double>(header.timestamp.nsec) * 1e-9;

                        // 콜백 호출
                        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
                        if (camera_callback_) { // 콜백이 등록되어 있으면 호출
                            camera_callback_(temp_data);
                        }
                    }
                    buffer.clear();
                }
            }
        }
        else if (header_str == "BOX") {
            BoundingBoxData temp_data;
            if (ParseBoundingBoxData(packet_buffer, received_size, temp_data)) {
                temp_data.timestamp =
                        static_cast<double>(header.timestamp.sec) + static_cast<double>(header.timestamp.nsec) * 1e-9;

                // 콜백 호출
                std::lock_guard<std::mutex> callback_lock(callback_mutex_);
                if (bbox_callback_) { // 콜백이 등록되어 있으면 호출
                    bbox_callback_(temp_data);
                }
            }
        }
    }
}

bool Camera::ParseCameraData(const std::vector<uint8_t>& buffer, CameraData& data) {
    if (buffer.size() < sizeof(int) * 2 + sizeof(double)) {
        std::cerr << "Received data is too small" << std::endl;
        return false;
    }

    // Decode image using OpenCV
    cv::Mat decoded_image = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (decoded_image.empty()) {
        std::cerr << "Failed to decode image" << std::endl;
        return false;
    }

    data.image_data = decoded_image;
    data.height = decoded_image.rows;
    data.width = decoded_image.cols;
    return true;
}

bool Camera::ParseBoundingBoxData(const char* buffer, size_t size, BoundingBoxData& data) {
    if (size < sizeof(BoundingBoxPacketStruct)) {
        std::cerr << "Received bounding box data is too small" << std::endl;
        return false;
    }

    const BoundingBoxPacketStruct* bbox_packet = reinterpret_cast<const BoundingBoxPacketStruct*>(buffer);

    data.bbox_2d.clear();
    data.bbox_3d.clear();
    data.classes.clear();

    // Calculate actual number of objects from packet size
    CameraPacketHeader header;
    std::memcpy(&header, buffer, sizeof(CameraPacketHeader));
    size_t payload_size = header.size;
    size_t num_objects = payload_size / sizeof(BoundingBoxPacketStruct::boundingbox.object_data[0]);
    size_t tail_index = sizeof(CameraPacketHeader) + payload_size;

    // Iterate only through valid objects
    for (size_t i = 0; i < num_objects; i++) {
        const auto& obj = bbox_packet->boundingbox.object_data[i];

        // Skip if all values are 0 (empty object)
        bool is_empty = true;
        for (int j = 0; j < 4; j++) {
            if (obj.bbox2d[j] != 0.0f) {
                is_empty = false;
                break;
            }
        }
        if (is_empty) continue;

        // Add 2D bounding box
        BoundingBox2D bbox2d;
        bbox2d.x_min = obj.bbox2d[0];
        bbox2d.y_min = obj.bbox2d[1];
        bbox2d.x_max = obj.bbox2d[2];
        bbox2d.y_max = obj.bbox2d[3];
        data.bbox_2d.push_back(bbox2d);

        // Add 3D bounding box center point
        BoundingBox3D bbox3d;
        // Calculate center point from 8 points
        bbox3d.x = 0.0f;
        bbox3d.y = 0.0f;
        bbox3d.z = 0.0f;
        for (int j = 0; j < 8; j++) {
            bbox3d.x += obj.bbox3d[j * 3] / 8.0f;
            bbox3d.y += obj.bbox3d[j * 3 + 1] / 8.0f;
            bbox3d.z += obj.bbox3d[j * 3 + 2] / 8.0f;
        }
        data.bbox_3d.push_back(bbox3d);

        // Add class information
        BoundingBoxClass cls;
        cls.group_id = obj.classTag[0];
        cls.class_id = obj.classTag[1];
        cls.sub_class_id = obj.classTag[2];
        data.classes.push_back(cls);
    }

    return true;
}
