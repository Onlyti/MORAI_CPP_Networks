#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "../network/udp_receiver.hpp"

namespace MoraiCppUdp{
/**
 * @brief MORAI 시뮬레이터의 카메라 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 카메라 이미지와 바운딩 박스 데이터를 수신하고 처리합니다
 */
class Camera : public UDPReceiver {
    const static size_t MAX_PACKET_SIZE = 65000;

public:
    // Camera Data Type
    struct CameraPacketHeader // (19byte)
    {
#pragma pack(push, 1)
        // Header (3byte) - one of "MOR"("Camera"), "BOX"("BoundingBox")
        uint8_t header[3];
        // Timestamp (8byte)
        struct Timestamp {
            uint32_t sec;
            uint32_t nsec;
        } timestamp;

        // Index and Size (8byte)
        int32_t index;
        int32_t size;
#pragma pack(pop)
    };

    struct CameraData {
        int width;
        int height;
        cv::Mat image_data;
        double timestamp;
    };

    union CameraPacketStruct {
        uint8_t data[MAX_PACKET_SIZE] = {0};
        struct {
#pragma pack(push, 1)
            CameraPacketHeader header;
            uint8_t data[64979];
            uint8_t tail[2];
#pragma pack(pop)
        } image;
    };

    // Bounding Box Data Types
    struct BoundingBox2D {
        float x_min;
        float y_min;
        float x_max;
        float y_max;
    };

    struct BoundingBox3D {
        float x;
        float y;
        float z;
    };

    struct BoundingBoxClass {
        uint8_t group_id;
        uint8_t class_id;
        uint8_t sub_class_id;
    };

    struct BoundingBoxData {
        std::vector<BoundingBox2D> bbox_2d;
        std::vector<BoundingBox3D> bbox_3d;
        std::vector<BoundingBoxClass> classes;
        double timestamp;
    };

    union BoundingBoxPacketStruct {
        uint8_t data[MAX_PACKET_SIZE] = {0};
        struct {
#pragma pack(push, 1)
            CameraPacketHeader header;
            struct ObjectData {
                float bbox3d[24];    // 3D BBOX (96byte = 4byte * 3 * 8 points)
                float bbox2d[4];     // 2D BBOX (16byte = 4byte * 4)
                uint8_t classTag[3]; // Class/Tag info (3byte)
            } object_data[565];
            uint8_t tail[2];
#pragma pack(pop)
        } boundingbox;
    };

    // 콜백 함수 타입 정의
    using CameraDataCallback = std::function<void(const CameraData&)>;
    using BoundingBoxDataCallback = std::function<void(const BoundingBoxData&)>;

    Camera() = delete;

    /**
     * @brief Camera 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    Camera(const std::string& ip_address, uint16_t port);

    /**
     * @brief Camera 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    Camera(const std::string& ip_address, uint16_t port, CameraDataCallback callback);

    /**
     * @brief Camera 클래스의 소멸자
     */
    ~Camera();

    /**
     * @brief 카메라 데이터 콜백 함수 등록
     * @param callback 카메라 데이터를 처리할 콜백 함수
     */
    void RegisterCameraCallback(CameraDataCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        camera_callback_ = callback;
    }

    /**
     * @brief 바운딩 박스 데이터 콜백 함수 등록
     * @param callback 바운딩 박스 데이터를 처리할 콜백 함수
     */
    void RegisterBoundingBoxCallback(BoundingBoxDataCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        bbox_callback_ = callback;
    }

private:
    // 기본 콜백 함수
    static void DefaultCameraCallback(const CameraData& data) {
        std::cout << "Note: Camera data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCameraCallback() "
                  << "to process the camera data." << std::endl;
    }

    static void DefaultBoundingBoxCallback(const BoundingBoxData&) {
        // 바운딩 박스는 무음 처리
    }

    std::thread thread_camera_udp_receiver_;
    std::atomic<bool> is_running_;

    // 콜백 관련 멤버 - 기본 콜백으로 초기화
    CameraDataCallback camera_callback_{DefaultCameraCallback};
    BoundingBoxDataCallback bbox_callback_{DefaultBoundingBoxCallback};
    std::mutex callback_mutex_;

    // 데이터 파싱 함수
    void ThreadCameraUdpReceiver();
    bool ParseCameraData(const std::vector<uint8_t>& buffer, CameraData& data);
    bool ParseBoundingBoxData(const char* buffer, size_t size, BoundingBoxData& data);
};

} // namespace MoraiCppUdp
#endif // __CAMERA_HPP__
