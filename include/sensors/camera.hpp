#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "../network/udp_receiver.hpp"

/**
 * @brief MORAI 시뮬레이터의 카메라 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 카메라 이미지와 바운딩 박스 데이터를 수신하고 처리합니다
 */
class Camera : public UDPReceiver
{
    const static uint32_t MAX_SYNC_DATA_TIME_MS = 10;
    const static size_t MAX_PACKET_SIZE = 65000;

 public:
    // Camera Data Type
    struct CameraPacketHeader  // (19byte)
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
        // Initialize data to 0
        uint8_t data[MAX_PACKET_SIZE] = {0};
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
    struct BoundingBoxData
    {
        std::vector<BoundingBox2D> bbox_2d;
        std::vector<BoundingBox3D> bbox_3d;
        std::vector<BoundingBoxClass> classes;
        double timestamp;
    };

    union BoundingBoxPacketStruct
    {
        // Initialize data to 0
        uint8_t data[MAX_PACKET_SIZE] = {0};
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
    std::deque<CameraData> camera_data_queue_;
    size_t max_camera_data_queue_size_;
    bool is_camera_data_received_;


    // 바운딩 박스 데이터 멤버 변수와 뮤텍스 추가
    std::mutex mutex_bbox_data_;
    BoundingBoxData bbox_data_;
    std::deque<BoundingBoxData> bbox_data_queue_;
    size_t max_bbox_data_queue_size_;
    bool is_bbox_data_received_;

    Camera() = delete;  // Prevent default constructor
    /**
     * @brief Camera 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @note 생성자에서 UDP 수신 스레드가 시작됩니다
     */
    Camera(const std::string& ip_address, uint16_t port);
    /**
     * @brief Camera 클래스의 소멸자
     * @note UDP 수신 스레드를 안전하게 종료합니다
     */
    ~Camera();

    /**
     * @brief 카메라 데이터를 가져옵니다
     * @param[out] data 카메라 데이터를 저장할 구조체
     * @return 데이터 수신 성공 여부
     * @note 이 함수는 새로운 데이터가 수신될 때까지 이전 데이터를 반환하지 않습니다
     */
    bool GetCameraData(CameraData& data);

    /**
     * @brief 바운딩 박스 데이터를 가져옵니다
     * @param[out] data 바운딩 박스 데이터를 저장할 구조체
     * @return 데이터 수신 성공 여부
     * @note 이 함수는 새로운 데이터가 수신될 때까지 이전 데이터를 반환하지 않습니다
     */
    bool GetBoundingBoxData(BoundingBoxData& data);

    /**
     * @brief 동기화된 카메라와 바운딩 박스 데이터를 가져옵니다
     * @param[out] camera_data 카메라 데이터를 저장할 구조체
     * @param[out] bbox_data 바운딩 박스 데이터를 저장할 구조체
     * @return 동기화된 데이터 수신 성공 여부
     * @note 두 데이터의 타임스탬프 차이가 MAX_SYNC_DATA_TIME_MS 이내인 경우에만 true를 반환합니다
     */
    bool GetSyncData(CameraData& camera_data, BoundingBoxData& bbox_data);

    /**
     * @brief 바운딩 박스가 그려진 이미지를 가져옵니다
     * @param[out] image 바운딩 박스가 그려진 이미지
     * @return 이미지 생성 성공 여부
     */
    bool GetBoundingBoxedImage(cv::Mat& image);

    /**
     * @brief 원본 카메라 이미지를 가져옵니다
     * @param[out] image 카메라 이미지
     * @return 이미지 수신 성공 여부
     */
    bool GetImage(cv::Mat& image);

    /**
     * @brief 가장 최근의 카메라 데이터를 가져옵니다
     * @param[out] data 카메라 데이터를 저장할 구조체
     * @return 데이터 존재 여부
     * @note GetCameraData와 달리 항상 최신 데이터를 반환합니다
     */
    bool GetLatestCameraData(CameraData& data);

    /**
     * @brief 가장 최근의 바운딩 박스 데이터를 가져옵니다
     * @param[out] data 바운딩 박스 데이터를 저장할 구조체
     * @return 데이터 존재 여부
     * @note GetBoundingBoxData와 달리 항상 최신 데이터를 반환합니다
     */
    bool GetLatestBoundingBoxData(BoundingBoxData& data);

    /**
     * @brief 가장 최근의 동기화된 데이터를 가져옵니다
     * @param[out] camera_data 카메라 데이터를 저장할 구조체
     * @param[out] bbox_data 바운딩 박스 데이터를 저장할 구조체
     * @return 동기화된 데이터 수신 성공 여부
     * @note 두 데이터의 타임스탬프 차이가 MAX_SYNC_DATA_TIME_MS 이내인 경우에만 true를 반환합니다
     */
    bool GetLatestData(CameraData& camera_data, BoundingBoxData& bbox_data);

    /**
     * @brief 바운딩 박스가 그려진 이미지를 가져옵니다
     * @param[out] image 바운딩 박스가 그려진 이미지
     * @return 이미지 생성 성공 여부
     */
    bool GetLatestBoundingBoxedImage(cv::Mat& image);

    /**
     * @brief 원본 카메라 이미지를 가져옵니다
     * @param[out] image 카메라 이미지
     * @return 이미지 수신 성공 여부
     */
    bool GetLatestImage(cv::Mat& image);

 private:
    void ThreadCameraUdpReceiver();
    bool ParseCameraData(const std::vector<uint8_t>& buffer, CameraData& data);
    bool ParseBoundingBoxData(const char* buffer, size_t size, BoundingBoxData& data);
};

#endif  // __CAMERA_HPP__
