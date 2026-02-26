#ifndef __CAMERA2_HPP__
#define __CAMERA2_HPP__

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
class Camera2 : public UDPReceiver {
    const static size_t MAX_PACKET_SIZE = 65000;

public:
    // Camera Data Type
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
            int32_t seq;
            // int32_t size;
            uint8_t data[64979];
#pragma pack(pop)
        } image;
    };

    // 콜백 함수 타입 정의
    using CameraDataCallback = std::function<void(const CameraData&)>;

    Camera2() = delete;

    /**
     * @brief Camera 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    Camera2(const std::string& ip_address, uint16_t port);

    /**
     * @brief Camera 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    Camera2(const std::string& ip_address, uint16_t port, CameraDataCallback callback);

    /**
     * @brief Camera 클래스의 소멸자
     */
    ~Camera2();

    /**
     * @brief 카메라 데이터 콜백 함수 등록
     * @param callback 카메라 데이터를 처리할 콜백 함수
     */
    void RegisterCameraCallback(CameraDataCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        camera_callback_ = callback;
    }

private:
    // 기본 콜백 함수
    static void DefaultCameraCallback(const CameraData& data) {
        std::cout << "Note: Camera data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCameraCallback() "
                  << "to process the camera data." << std::endl;
    }

    std::thread thread_camera_udp_receiver_;
    std::atomic<bool> is_running_;

    // 콜백 관련 멤버 - 기본 콜백으로 초기화
    CameraDataCallback camera_callback_{DefaultCameraCallback};
    std::mutex callback_mutex_;

    // 데이터 파싱 함수
    void ThreadCameraUdpReceiver();
    bool ParseCameraData(const char* buffer, size_t received_size, CameraData& data);

    // Reassembly state for seq + data camera packet stream.
    bool has_assembling_seq_{false};
    int32_t assembling_seq_{0};
    uint16_t assembling_total_chunks_{0};
    uint16_t assembling_last_chunk_no_{0};
    size_t assembling_expected_image_size_{0};
    std::vector<uint8_t> assembling_image_data_;
};

} // namespace MoraiCppUdp
#endif // __CAMERA2_HPP__
