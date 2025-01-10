#pragma once
#ifndef __MULTI_EGO_SETTING_HPP__
#define __MULTI_EGO_SETTING_HPP__

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "network/udp_sender.hpp"

namespace MoraiCppUdp {

#pragma pack(push, 1) // 1바이트 정렬 시작

// 차량의 상태를 나타내는 구조체 (32 bytes)
struct EgoState {
    int16_t ego_index; // 차량의 인덱스 번호
    float g_pos_x;     // 차량의 position (m)
    float g_pos_y;     // 차량의 position (m)
    float g_pos_z;     // 차량의 position (m)
    float g_roll;      // 차량의 rotation (deg)
    float g_pitch;     // 차량의 rotation (deg)
    float g_yaw;       // 차량의 rotation (deg)
    float speed;       // 차량의 속력 (km/h)
    uint8_t gear;      // 기어모드 변경 [1: Parking, 2: Rear, 3: Neutral, 4: Drive]
    uint8_t ctrl_mode; // 제어하는 방식 [1: Keyboard, 2: AutoMode]
};

// 전체 패킷 구조체
struct MultiEgoSettingPacket {
    uint8_t start_indicator;          // '#' (0x23)
    char header_name[15];             // "MultiEgoSetting"
    uint8_t doller_indicator;         // '$' (0x24)
    uint32_t data_length;             // 648
    uint8_t aux_data[12];             // 예약된 공간
    int32_t num_of_ego;               // 제어할 차량의 개수
    int32_t camera_index;             // 어떤 차량에 카메라를 고정 시킬 지 정함
    std::vector<EgoState> ego_states; // 차량 상태 데이터 (최대 20개)
    uint8_t tail01;                   // 0x0D
    uint8_t tail02;                   // 0x0A
};

#pragma pack(pop) // 이전 정렬로 복원

class MultiEgoSetting : public UDPSender {
public:
    MultiEgoSetting(const std::string& ip, int port);
    ~MultiEgoSetting() = default;

    // 패킷 전송
    bool SendMultiEgoSetting(const MultiEgoSettingPacket& packet);

    // 패킷 생성 헬퍼 함수
    static MultiEgoSettingPacket CreatePacket(int32_t num_of_ego, int32_t camera_index,
                                              const std::vector<EgoState>& ego_states);
};

// Gear Mode enum
enum class GearMode : uint8_t { Parking = 1, Rear = 2, Neutral = 3, Drive = 4 };

// Control Mode enum
enum class ControlMode : uint8_t { Keyboard = 1, AutoMode = 2 };

} // namespace MoraiCppUdp

#endif // __MULTI_EGO_SETTING_HPP__