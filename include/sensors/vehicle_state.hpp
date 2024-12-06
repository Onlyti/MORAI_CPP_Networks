#ifndef __VEHICLE_STATE_HPP__
#define __VEHICLE_STATE_HPP__

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../network/udp_receiver.hpp"

/**
 * @class VehicleState
 * @brief MORAI 시뮬레이터의 차량 상태 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 차량의 상태 정보를 수신하고 파싱합니다.
 *          전체 패킷 크기: 181 Bytes (데이터 크기: 50 Bytes + 102 Bytes)
 */
class VehicleState : public UDPReceiver
{
    const static size_t PACKET_SIZE = 229;  // 11 + 216 + 2

 public:
    /**
     * @brief 차량 제어 모드 열거형
     */
    enum class ControlMode : uint8_t
    {
        MORAI_AI = 0,  ///< MORAI AI 제어 모드
        KEYBOARD = 1,  ///< 키보드 제어 모드
        AUTO = 2,       ///< 자율주행 모드
    };

    /**
     * @brief 기어 상태 열거형
     */
    enum class GearMode : uint8_t
    {
        MANUAL = 0,   ///< 수동
        PARKING = 1,  ///< 주차
        REVERSE = 2,  ///< 후진
        NEUTRAL = 3,  ///< 중립
        DRIVE = 4,    ///< 주행
        LOW = 5       ///< L단
    };

    /**
     * @struct Timestamp
     * @brief Unix timestamp 정보 (1970/01/01부터의 경과 시간)
     */
    struct Timestamp
    {  // 8 bytes
// #pragma pack(push, 1)
        int32_t seconds;      ///< (4 bytes) 경과된 초 단위 시간
        int32_t nanoseconds;  ///< (4 bytes) timestamp의 소수 단위 초 (나노초)
// #pragma pack(pop)
    };

    /**
     * @struct Vector3
     * @brief 3차원 벡터 정보를 저장하는 구조체
     */
    struct Vector3
    {
// #pragma pack(push, 1)
        float x;
        float y;
        float z;
// #pragma pack(pop)
    };

    /**
     * @struct VehicleData
     * @brief 차량 상태 데이터를 저장하는 구조체
     */
    struct VehicleData
    {  // 180 bytes
#pragma pack(push, 1)
        Timestamp timestamp;    ///< (8 bytes) 타임스탬프 정보
        ControlMode ctrl_mode;  ///< (1 byte) 제어 모드
        GearMode gear;          ///< (1 byte) 기어 상태
        float signed_velocity;  ///< (4 bytes) 차량 진행 방향 속도 (km/h)
        int32_t
            map_data_id;  ///< (4 bytes) Map data id 값 (0~9999: DigitalTwin, 10000~19999: Virtual)
        float accel_input;                  ///< (4 bytes) 가속 페달 입력값 (0~1)
        float brake_input;                  ///< (4 bytes) 브레이크 페달 입력값 (0~1)
        // Vector3 size;                       ///< (12 bytes) 차량 크기 (m)
        float size_x;
        float size_y;
        float size_z;
        float overhang;                     ///< (4 bytes) 전방 오버행
        float wheelbase;                    ///< (4 bytes) 축거
        float rear_overhang;                ///< (4 bytes) 후방 오버행
        Vector3 position;                   ///< (12 bytes) 차량 위치 (m)
        Vector3 rotation;                   ///< (12 bytes) Roll/Pitch/Heading (deg)
        Vector3 velocity;                   ///< (12 bytes) XYZ 속도 (km/h)
        Vector3 angular_velocity;           ///< (12 bytes) 회전 각속도 (deg/s)
        Vector3 acceleration;               ///< (12 bytes) XYZ 가속도 (m/s^2)
        float steering;                     ///< (4 bytes) 조향각 (deg)
        char mgeo_link_id[38];              ///< (38 bytes) MGeo Link ID
        float tire_lateral_force_fl;        ///< (4 bytes) 좌측 전방 타이어 측면 힘 (N)
        float tire_lateral_force_fr;        ///< (4 bytes) 우측 전방 타이어 측면 힘 (N)
        float tire_lateral_force_rl;        ///< (4 bytes) 좌측 후방 타이어 측면 힘 (N)
        float tire_lateral_force_rr;        ///< (4 bytes) 우측 후방 타이어 측면 힘 (N)
        float side_slip_angle_fl;           ///< (4 bytes) 좌측 전방 타이어 측면 각도 (deg)
        float side_slip_angle_fr;           ///< (4 bytes) 우측 전방 타이어 측면 각도 (deg)
        float side_slip_angle_rl;           ///< (4 bytes) 좌측 후방 타이어 측면 각도 (deg)
        float side_slip_angle_rr;           ///< (4 bytes) 우측 후방 타이어 측면 각도 (deg)
        float tire_cornering_stiffness_fl;  ///< (4 bytes) 좌측 전방 타이어 코너링 강성 (N/deg)
        float tire_cornering_stiffness_fr;  ///< (4 bytes) 우측 전방 타이어 코너링 강성 (N/deg)
        float tire_cornering_stiffness_rl;  ///< (4 bytes) 좌측 후방 타이어 코너링 강성 (N/deg)
        float tire_cornering_stiffness_rr;  ///< (4 bytes) 우측 후방 타이어 코너링 강성 (N/deg)
#pragma pack(pop)
    };

    union VehicleStatePacketStruct
    {
        char data[PACKET_SIZE];  // 211 bytes
        struct
        {
#pragma pack(push, 1)
            char sharp;                // 1 byte
            char header[9];            // 9 bytes
            char dollar;               // 1 byte
            uint32_t length;           // 4 bytes
            uint8_t AuxData[12];       // 12 bytes
#pragma pack(pop)
            VehicleData vehicle_data;  // 180 bytes
#pragma pack(push, 1)
            uint8_t tail[2];           // 2 bytes
#pragma pack(pop)
        } packet;
    };

    /**
     * @brief VehicleState 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    VehicleState(const std::string& ip_address, uint16_t port);

    /**
     * @brief VehicleState 클래스의 소멸자
     */
    ~VehicleState();

    /**
     * @brief 차량 상태 데이터를 가져옵니다
     * @param[out] data 차량 상태 데이터를 저장할 구조체
     * @return 데이터 수신 성공 여부
     */
    bool GetVehicleState(VehicleData& data);

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

    std::thread thread_vehicle_state_receiver_;  ///< 차량 상태 수신 스레드
    std::atomic<bool> is_running_;               ///< 스레드 실행 상태
    std::mutex mutex_vehicle_data_;              ///< 데이터 보호를 위한 뮤텍스
    VehicleData vehicle_data_;                   ///< 최신 차량 상태 데이터
    VehicleStatePacketStruct packet_data_;
    bool is_data_received_;                      ///< 데이터 수신 여부
};

#endif  // __VEHICLE_STATE_HPP__