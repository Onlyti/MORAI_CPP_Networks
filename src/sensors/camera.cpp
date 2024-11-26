#include "sensors/camera.hpp"

#include <iostream>

Camera::Camera(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false)
{
    is_running_ = true;
    is_camera_data_received_ = false;
    is_bbox_data_received_ = false;
    max_camera_data_queue_size_ = 10;
    max_bbox_data_queue_size_ = 10;
    thread_camera_udp_receiver_ = std::thread(&Camera::ThreadCameraUdpReceiver, this);
    thread_camera_udp_receiver_.detach();
}

Camera::~Camera()
{
    is_running_ = false;
    if (thread_camera_udp_receiver_.joinable())
    {
        thread_camera_udp_receiver_.join();
    }
    Close();
}

bool Camera::GetCameraData(CameraData& data)
{
    if (!is_camera_data_received_)
    {
        return false;
    }
    is_camera_data_received_ = false;
    return std::move(GetLatestCameraData(data));
}

bool Camera::GetBoundingBoxData(BoundingBoxData& data)
{
    if (!is_bbox_data_received_)
    {
        return false;
    }
    is_bbox_data_received_ = false;
    return std::move(GetLatestBoundingBoxData(data));
}

bool Camera::GetSyncData(CameraData& camera_data, BoundingBoxData& bbox_data)
{
    GetLatestCameraData(camera_data);
    int camera_timestamp_ns = static_cast<int>(camera_data.timestamp * 1e9);

    GetLatestBoundingBoxData(bbox_data);
    int bbox_timestamp_ns = static_cast<int>(bbox_data.timestamp * 1e9);

    if (std::abs(camera_timestamp_ns - bbox_timestamp_ns) >
        static_cast<int>(MAX_SYNC_DATA_TIME_MS * 1e6))
    {
        return false;
    }
    return true;
}

bool Camera::GetBoundingBoxedImage(cv::Mat& image)
{
    if (!is_bbox_data_received_)
    {
        return false;
    }
    is_bbox_data_received_ = false;
    return GetLatestBoundingBoxedImage(image);
}

bool Camera::GetImage(cv::Mat& image)
{
    CameraData camera_data;
    if (!GetCameraData(camera_data))
    {
        return false;
    }
    image = std::move(camera_data.image_data);
    return true;
}
bool Camera::GetLatestCameraData(CameraData& data)
{
    std::lock_guard<std::mutex> lock(mutex_camera_data_);
    data = camera_data_;
    return !data.image_data.empty();
}

bool Camera::GetLatestBoundingBoxData(BoundingBoxData& data)
{
    std::lock_guard<std::mutex> lock(mutex_bbox_data_);
    data = bbox_data_;
    return !data.bbox_2d.empty();
}

bool Camera::GetLatestData(CameraData& camera_data, BoundingBoxData& bbox_data)
{
    return GetLatestCameraData(camera_data) && GetLatestBoundingBoxData(bbox_data);
}

bool Camera::GetLatestBoundingBoxedImage(cv::Mat& image)
{
    CameraData camera_data;
    if (!GetLatestCameraData(camera_data))
    {
        return false;
    }
    // 바운딩 박스 데이터 가져오기
    Camera::BoundingBoxData bbox_data;
    if (!GetLatestBoundingBoxData(bbox_data))
    {
        return false;
    }
    // 이미지에 2D 바운딩 박스 그리기
    for (size_t i = 0; i < bbox_data.bbox_2d.size(); i++)
    {
        const auto& bbox = bbox_data.bbox_2d[i];
        const auto& cls = bbox_data.classes[i];

        // 바운딩 박스 그리기
        cv::rectangle(camera_data.image_data, cv::Point(bbox.x_min, bbox.y_min),
                      cv::Point(bbox.x_max, bbox.y_max), cv::Scalar(0, 255, 0), 2);

        // 클래스 정보 표시
        std::string class_text = "Class: " + std::to_string(cls.group_id) + "," +
                                 std::to_string(cls.class_id) + "," +
                                 std::to_string(cls.sub_class_id);
        cv::putText(camera_data.image_data, class_text, cv::Point(bbox.x_min, bbox.y_min - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    image = camera_data.image_data;
    return true;
}

bool Camera::GetLatestImage(cv::Mat& image)
{
    CameraData camera_data;
    if (!GetLatestCameraData(camera_data))
    {
        return false;
    }
    image = camera_data.image_data;
    return true;
}

void Camera::ThreadCameraUdpReceiver()
{
    std::vector<uint8_t> buffer;
    char packet_buffer[MAX_PACKET_SIZE];

    while (is_running_)
    {
        // Receive data from UDP
        size_t received_size = 0;
        if (!Receive(packet_buffer, MAX_PACKET_SIZE, received_size))
        {
            std::cerr << "Failed to receive camera data" << std::endl;
            continue;
        }
        // Check header: MOR-Camera, BOX-BoundingBox
        CameraPacketHeader header;
        std::memcpy(&header, packet_buffer, sizeof(CameraPacketHeader));
        std::string header_str(reinterpret_cast<char*>(header.header), 3);
        if (header_str != "MOR" && header_str != "BOX")
        {
            std::cerr << "Invalid header" << std::endl;
            continue;
        }
        // std::cout << "Header: " << header_str << std::endl;
        // std::cout << "Size: " << header.size << std::endl;
        // std::cout << "Time: " << header.timestamp.sec << "." << std::setw(9) << std::setfill('0')
        //           << header.timestamp.nsec << std::endl;
        if (header_str == "MOR")
        {
            CameraPacketStruct camera_packet;
            std::memcpy(&camera_packet, packet_buffer, sizeof(CameraPacketStruct));

            if (received_size > 2)
            {
                // 19: Header (3) + Timestamp (8) + Index (4) + Size (4)
                buffer.insert(buffer.end(), packet_buffer + 19, packet_buffer + received_size - 2);
                if (camera_packet.image.tail[0] == 'E' && camera_packet.image.tail[1] == 'I')
                {
                    // Image data is complete
                    CameraData temp_data;
                    if (ParseCameraData(buffer, temp_data))
                    {
                        temp_data.timestamp = static_cast<double>(header.timestamp.sec) +
                                              static_cast<double>(header.timestamp.nsec) * 1e-9;
                        std::lock_guard<std::mutex> lock(mutex_camera_data_);
                        camera_data_ = std::move(temp_data);
                        is_camera_data_received_ = true;
                    }
                    buffer.clear();
                    // // 소수점 이후는 9자리 고정
                    // std::cout << "Camera timestamp: " << std::endl
                    //           << header.timestamp.sec << "." << std::setw(9) << std::setfill('0')
                    //           << header.timestamp.nsec << std::endl;
                }
                continue;
            }
        }
        else if (header_str == "BOX")
        {
            BoundingBoxData temp_data;
            if (ParseBoundingBoxData(packet_buffer, received_size, temp_data))
            {
                temp_data.timestamp = static_cast<double>(header.timestamp.sec) +
                                      static_cast<double>(header.timestamp.nsec) * 1e-9;

                std::lock_guard<std::mutex> lock(mutex_bbox_data_);
                bbox_data_ = std::move(temp_data);
                is_bbox_data_received_ = true;
            }
            // std::cout << "Bounding box timestamp: " << std::endl
            //           << header.timestamp.sec << "." << std::setw(9) << std::setfill('0')
            //           << header.timestamp.nsec << std::endl;
        }
    }
}

bool Camera::ParseCameraData(const std::vector<uint8_t>& buffer, CameraData& data)
{
    if (buffer.size() < sizeof(int) * 2 + sizeof(double))
    {
        std::cerr << "Received data is too small" << std::endl;
        return false;
    }

    // Decode image using OpenCV
    cv::Mat decoded_image = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (decoded_image.empty())
    {
        std::cerr << "Failed to decode image" << std::endl;
        return false;
    }

    data.image_data = decoded_image;
    data.height = decoded_image.rows;
    data.width = decoded_image.cols;
    return true;
}

bool Camera::ParseBoundingBoxData(const char* buffer, size_t size, BoundingBoxData& data)
{
    if (size < sizeof(BoundingBoxPacketStruct))
    {
        std::cerr << "Received bounding box data is too small" << std::endl;
        return false;
    }

    const BoundingBoxPacketStruct* bbox_packet =
        reinterpret_cast<const BoundingBoxPacketStruct*>(buffer);

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
    for (size_t i = 0; i < num_objects; i++)
    {
        const auto& obj = bbox_packet->boundingbox.object_data[i];

        // Skip if all values are 0 (empty object)
        bool is_empty = true;
        for (int j = 0; j < 4; j++)
        {
            if (obj.bbox2d[j] != 0.0f)
            {
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
        for (int j = 0; j < 8; j++)
        {
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
