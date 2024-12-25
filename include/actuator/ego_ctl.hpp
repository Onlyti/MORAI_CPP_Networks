#ifndef __EGO_CTL_HPP__
#define __EGO_CTL_HPP__

#include <cstdint>
#include <memory>
#include <string>

#include "../network/udp_sender.hpp"

namespace MoraiCppUdp {

/**
 * @brief MORAI 시뮬레이터의 차량 제어를 위한 클래스
 * @details UDP를 통해 차량 제어 명령을 전송합니다
 */
class EgoControl
{
    // Packet Structure Constants
    const static uint32_t HEADER_SIZE = 1;           // '#'
    const static uint32_t CMD_SIZE = 12;             // "MoraiCtrlCmd"
    const static uint32_t DELIMITER_SIZE = 1;        // '$'
    const static uint32_t DATA_LENGTH_SIZE = 4;      // uint32_t
    const static uint32_t AUX_DATA_SIZE = 12;        // Reserved
    const static uint32_t CONTROL_DATA_SIZE = 3;     // Mode, Gear, CmdType
    const static uint32_t VEHICLE_DATA_SIZE = 20;    // 5 * sizeof(float)
    const static uint32_t TAIL_SIZE = 2;             // 0x0D 0x0A

    const static uint32_t PACKET_SIZE = HEADER_SIZE + CMD_SIZE + DELIMITER_SIZE + 
                                      DATA_LENGTH_SIZE + AUX_DATA_SIZE + 
                                      CONTROL_DATA_SIZE + VEHICLE_DATA_SIZE + TAIL_SIZE;  // 55 bytes

public:
    // Control Mode
    enum class ControlMode : uint8_t {
        KEYBOARD = 1,
        AUTO_MODE = 2
    };

    // Gear Command
    enum class GearCommand : uint8_t {
        MANUAL = 0,
        PARKING = 1,    // P
        REVERSE = 2,    // R
        NEUTRAL = 3,    // N
        DRIVE = 4,      // D
        LOW = 5         // L
    };

    // Longitudinal Command Type
    enum class LongitudinalCommandType : uint8_t {
        NONE = 0,
        THROTTLE = 1,      // Throttle control (accel/brake/steering)
        VELOCITY = 2,      // Velocity control (velocity/steering)
        ACCELERATION = 3   // Acceleration control
    };

    // Control Command Structure
#pragma pack(push, 1)
    struct ControlCommand {
        ControlMode ctrl_mode;
        GearCommand gear;
        LongitudinalCommandType long_cmd_type;
        
        float velocity;       // km/h, used when long_cmd_type is VELOCITY
        float acceleration;   // m/s^2, used when long_cmd_type is ACCELERATION
        float accel;         // 0 ~ 1
        float brake;         // 0 ~ 1
        float steering;      // -1 ~ 1 (실제 조향각 = steering * 최대조향각(36.25))
    };
#pragma pack(pop)

    EgoControl() = delete;  // Prevent default constructor
    
    /**
     * @brief EgoControl 클래스의 생성자
     * @param ip_address UDP 전송을 위한 IP 주소
     * @param port UDP 전송을 위한 포트 번호
     */
    EgoControl(const std::string& ip_address, uint16_t dest_port, uint16_t host_port);
    
    /**
     * @brief EgoControl 클래스의 소멸자
     */
    ~EgoControl();

    /**
     * @brief 차량 제어 명령을 전송합니다
     * @param command 전송할 제어 명령
     * @return 전송 성공 여부
     */
    bool SendControlCommand(const ControlCommand& command);

private:
    std::unique_ptr<UDPSender> udp_sender_;
    bool PackControlCommand(const ControlCommand& command, uint8_t* buffer);
};

} // namespace MoraiCppUdp
#endif  // __EGO_CTL_HPP__
