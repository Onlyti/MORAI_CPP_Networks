/**
 * @file vehicle_state2.hpp
 * @brief VehicleState2 class definition
 * @details This file contains the definition of the VehicleState2 class, which is used to receive and parse the VehicleState2 data from the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-03-03
 */
#ifndef __VEHICLE_STATE2_HPP__
#define __VEHICLE_STATE2_HPP__

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../network/udp_receiver.hpp"

namespace MoraiCppUdp {

/**
 * @class VehicleState2
 * @brief MORAI 시뮬레이터의 차량 상태 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 차량의 상태 정보를 수신하고 파싱합니다.
 *          전체 패킷 크기: 181 Bytes (데이터 크기: 50 Bytes + 102 Bytes)
 */
class VehicleState2 : public UDPReceiver {
    const static size_t PACKET_SIZE = 108; // 8 + 4 + 24 + 18 * 4 = 108

public:

    /**
     * @struct Timestamp
     * @brief Unix timestamp 정보 (1970/01/01부터의 경과 시간)
     */
    struct Timestamp {       //  bytes
        #pragma pack(push, 1)
        int64_t sec;     ///< (8 bytes) 경과된 초 단위 시간
        int32_t nsec; ///< (4 bytes) timestamp의 소수 단위 초 (나노초)
        #pragma pack(pop)
    };

    /**
     * @struct Vector3
     * @brief 3차원 벡터 정보를 저장하는 구조체
     */
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    /**
     * @struct VehicleData
     * @brief 차량 상태 데이터를 저장하는 구조체
     */
    struct VehicleData {       // 108 bytes
        Timestamp timestamp;   ///< (8 bytes) 타임스탬프 정보
        char vehicle_id[24];   ///< (24 bytes) 차량 ID
        Vector3 position;                  ///< (12 bytes) 차량 위치 (m)
        Vector3 rotation;                  ///< (12 bytes) Roll/Pitch/Heading (deg)
        Vector3 velocity;                  ///< (12 bytes) XYZ 속도 (km/h)
        Vector3 acceleration;              ///< (12 bytes) XYZ 가속도 (m/s^2)
        Vector3 angular_velocity;          ///< (12 bytes) 회전 각속도 (deg/s)
        float accel_input;     ///< (4 bytes) 가속 페달 입력값 (0~1)
        float brake_input;     ///< (4 bytes) 브레이크 페달 입력값 (0~1)
        float steering;                    ///< (4 bytes) 조향각 (deg)
    };

    union VehicleStatePacketStruct {
        char data[PACKET_SIZE]; // 108 bytes
        struct {
            VehicleData vehicle_data; // 108 bytes
        } packet;
    };

    /**
     * @brief 콜백 함수 타입 정의
     */
    using VehicleState2Callback = std::function<void(const VehicleData&)>;

    /**
     * @brief VehicleState 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    VehicleState2(const std::string& ip_address, uint16_t port);

    /**
     * @brief VehicleState 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @param callback VehicleState 데이터를 처리할 콜백 함수
     */
    VehicleState2(const std::string& ip_address, uint16_t port, VehicleState2Callback callback);

    /**
     * @brief VehicleState 클래스의 소멸자
     */
    ~VehicleState2();

    /**
     * @brief 차량 상태 데이터를 가져옵니다
     * @param[out] data 차량 상태 데이터를 저장할 구조체
     * @return 데이터 수신 성공 여부
     */
    bool GetVehicleState(VehicleData& data);

    /**
     * @brief 콜백 등록 함수
     * @param callback 콜백 함수
     */
    void RegisterCallback(VehicleState2Callback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        vehicle_state2_callback_ = callback;
    }

public:
    /**
     * @brief UDP 수신 스레드 함수
     */
    void ThreadVehicleStateReceiver();

    /**
     * @brief 수신된 데이터를 파싱합니다
     * @param buffer 수신된 원본 데이터 버퍼
     * @param size 버퍼의 크기
     * @param[out] data 파싱된 차량 상태 데이터를 저장할 구조체
     * @return 파싱 성공 여부
     */
    bool ParseVehicleState(const char* buffer, size_t size, VehicleStatePacketStruct& data);

    std::thread thread_vehicle_state_receiver_; ///< 차량 상태 수신 스레드
    std::mutex callback_mutex_;                 ///< 콜백 함수 보호를 위한 뮤텍스
    std::atomic<bool> is_running_{false};       ///< 스레드 실행 상태
    VehicleStatePacketStruct packet_data_;
    VehicleState2Callback vehicle_state2_callback_{[](const VehicleData& data) {
        std::cout << "Note: Vehicle state data received but no callback is registered.\n"
                  << "Vehicle Info:\n"
                  << "\tPosition: " << data.position.x << ", " << data.position.y << ", " << data.position.z << " m\n"
                  << "\tRotation: " << data.rotation.x << ", " << data.rotation.y << ", " << data.rotation.z << " deg\n"
                  << "\tVelocity: " << data.velocity.x << ", " << data.velocity.y << ", " << data.velocity.z << " km/h\n"
                  << "\tAcceleration: " << data.acceleration.x << ", " << data.acceleration.y << ", " << data.acceleration.z << " m/s^2\n"
                  << "\tAngular Velocity: " << data.angular_velocity.x << ", " << data.angular_velocity.y << ", " << data.angular_velocity.z << " deg/s\n"
                  << "\tAccel Input: " << data.accel_input << "\n"
                  << "\tBrake Input: " << data.brake_input << "\n"
                  << "\tSteering: " << data.steering << " deg\n"
                  << "Consider registering a callback using RegisterCallback()." << std::endl;
    }};
};

} // namespace MoraiCppUdp
#endif // __VEHICLE_STATE2_HPP__
