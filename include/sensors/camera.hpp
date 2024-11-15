#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>

#include "../network/udp_receiver.hpp"

class Camera : public UDPReceiver
{
    const static size_t MAX_PACKET_SIZE = 65000;

 public:
    // Camera Data Type
    struct CameraPacketHeader
    {
#pragma pack(push, 1)
        // Header (3byte) - one of "MOR"("Camera"), "BOX"("BoundingBox")
        uint8_t header[3];
        // Timestamp (8byte)
        struct Timestamp
        {
            uint32_t sec;
            uint32_t nsec;
        } timestamp;

        // Index and Size (8byte)
        int32_t index;
        int32_t size;
#pragma pack(pop)
    };
    struct CameraData
    {
        int width;
        int height;
        cv::Mat image_data;
        double timestamp;
    };

    union CameraPacketStruct
    {
        uint8_t data[MAX_PACKET_SIZE];
        struct
        {
// Turn off 4-byte alignment
#pragma pack(push, 1)
            // Header
            CameraPacketHeader header;

            // Image Data (64979byte)
            uint8_t data[64979];

            // Tail (2byte) - Fixed value "EO"
            uint8_t tail[2];
#pragma pack(pop)  // Turn on 4-byte alignment
        } image;
    };

    // Bounding Box Data Type
    struct BoundingBox2D
    {
        float x_min;
        float y_min;
        float x_max;
        float y_max;
    };
    struct BoundingBox3D
    {
        float x;
        float y;
        float z;
    };

    struct BoundingBoxClass
    {
        uint8_t group_id;
        uint8_t class_id;
        uint8_t sub_class_id;
    };

    
    // 바운딩 박스 데이터를 저장할 구조체 추가
    struct BoundingBoxData {
        std::vector<BoundingBox2D> bbox_2d;
        std::vector<BoundingBox3D> bbox_3d;
        std::vector<BoundingBoxClass> classes;
        double timestamp;
    };

    union BoundingBoxPacketStruct
    {
        uint8_t data[MAX_PACKET_SIZE];
        struct
        {
#pragma pack(push, 1)
            // Header
            CameraPacketHeader header;

            // Object Data Block (115byte)
            struct ObjectData
            {
                float bbox3d[24];     // 3D BBOX (96byte = 4byte * 3 * 8 points)
                float bbox2d[4];      // 2D BBOX (16byte = 4byte * 4)
                uint8_t classTag[3];  // Class/Tag info (3byte)
            } object_data[565];

            // Tail (2byte) - Fixed value "EO"
            uint8_t tail[2];
#pragma pack(pop)
        } boundingbox;
    };

    // UDP Receiver
    std::thread thread_camera_udp_receiver_;
    std::atomic<bool> is_running_;

    // Buffers
    std::vector<uint8_t> camera_buffer_;
    std::vector<uint8_t> boundingbox_buffer_;

    // Camera Data
    std::mutex mutex_camera_data_;
    CameraData camera_data_;
    bool is_camera_data_received_;


    // 바운딩 박스 데이터 멤버 변수와 뮤텍스 추가
    std::mutex mutex_bbox_data_;
    BoundingBoxData bbox_data_;
    bool is_bbox_data_received_;

    Camera() = delete;  // Prevent default constructor
    Camera(const std::string& ip_address, uint16_t port);
    ~Camera();

    bool GetCameraData(CameraData& data);
    bool GetBoundingBoxData(BoundingBoxData& data);

 private:
    void ThreadCameraUdpReceiver();
    bool ParseCameraData(const std::vector<uint8_t>& buffer, CameraData& data);
    bool ParseBoundingBoxData(const char* buffer, size_t size, BoundingBoxData& data);
};

#endif  // __CAMERA_HPP__
