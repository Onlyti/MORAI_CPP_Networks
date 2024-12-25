#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <float.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <iostream>

#include "../network/udp_receiver.hpp"

/**
 * @brief MORAI 시뮬레이터의 IMU 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 IMU 센서의 자세, 각속도, 선가속도 데이터를 수신하고 처리합니다
 */
class IMU : public UDPReceiver
{
    const static size_t PACKET_SIZE = 115;  // 25(header) + 80(data) + 2(tail)

public:
    /**
     * @brief IMU 센서 데이터를 저장하는 구조체
     */
    struct IMUData
    {
        // Orientation (Quaternion)
        double w;
        double x;
        double y;
        double z;

        // Angular Velocity (rad/s)
        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;

        // Linear Acceleration (m/s²)
        double linear_acceleration_x;
        double linear_acceleration_y;
        double linear_acceleration_z;
    };

    // 콜백 함수 타입 정의
    using IMUCallback = std::function<void(const IMUData&)>;

    IMU() = delete;  // Prevent default constructor
    /**
     * @brief IMU 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @note 생성자에서 UDP 수신 스레드가 시작됩니다
     */
    IMU(const std::string& ip_address, uint16_t port);

    /**
     * @brief IMU 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @param callback IMU 데이터를 처리할 콜백 함수
     */
    IMU(const std::string& ip_address, uint16_t port, IMUCallback callback);

    /**
     * @brief IMU 클래스의 소멸자
     * @note UDP 수신 스레드를 안전하게 종료합니다
     */
    ~IMU();

    /**
     * @brief IMU 데이터를 가져옵니다
     * @param[out] data IMU 데이터를 저장할 구조체
     * @return 데이터 수신 성공 여부
     * @note 이 함수는 새로운 데이터가 수신될 때까지 이전 데이터를 반환하지 않습니다
     */
    bool GetIMUData(IMUData& data);

    /**
     * @brief 가장 최근의 IMU 데이터를 가져옵니다
     * @param[out] data IMU 데이터를 저장할 구조체
     * @return 데이터 존재 여부
     * @note GetIMUData와 달리 항상 최신 데이터를 반환합니다
     */
    bool GetLatestIMUData(IMUData& data);

    // 콜백 등록 함수
    void RegisterCallback(IMUCallback callback)
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        imu_callback_ = callback;
    }

private:
    /**
     * @brief UDP 수신 스레드 함수
     * @note 이 함수는 별도의 스레드에서 실행되며, IMU 데이터를 지속적으로 수신합니다
     */
    void ThreadIMUUdpReceiver();

    /**
     * @brief IMU 데이터를 파싱합니다
     * @param buffer 수신된 원본 데이터 버퍼
     * @param size 버퍼의 크기
     * @param[out] data 파싱된 IMU 데이터를 저장할 구조체
     * @return 파싱 성공 여부
     */
    bool ParseIMUData(const char* buffer, size_t size, IMUData& data);

    std::thread thread_imu_udp_receiver_;
    std::atomic<bool> is_running_;
    std::mutex mutex_imu_data_;
    IMUData imu_data_;
    bool is_imu_data_received_;
    std::mutex callback_mutex_;
    IMUCallback imu_callback_{[](const IMUData& data) {
        std::cout << "Note: IMU data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCallback()." << std::endl;
    }};
};

#endif  // __IMU_HPP__
