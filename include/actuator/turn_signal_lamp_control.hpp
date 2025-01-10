#ifndef __TURN_SIGNAL_LAMP_CTL_HPP__
#define __TURN_SIGNAL_LAMP_CTL_HPP__

#include <cstdint>
#include <memory>
#include <string>

#include "../network/udp_sender.hpp"

namespace MoraiCppUdp {

class TurnSignalLampControl {
    // Packet Structure Constants
    const static uint32_t HEADER_SIZE = 1;      // '#'
    const static uint32_t CMD_SIZE = 11;        // "LampControl"
    const static uint32_t DELIMITER_SIZE = 1;   // '$'
    const static uint32_t DATA_LENGTH_SIZE = 4; // uint32_t
    const static uint32_t AUX_DATA_SIZE = 12;   // Reserved
    const static uint32_t LAMP_DATA_SIZE = 2;   // turnSignal, emergencySignal
    const static uint32_t TAIL_SIZE = 2;        // 0x0D 0x0A

    const static uint32_t PACKET_SIZE = HEADER_SIZE + CMD_SIZE + DELIMITER_SIZE + DATA_LENGTH_SIZE + AUX_DATA_SIZE +
                                        LAMP_DATA_SIZE + TAIL_SIZE; // 33 bytes

public:
    // Turn Signal Types
    enum class TurnSignal : uint8_t { NONE = 0, LEFT = 1, RIGHT = 2 };

    // Emergency Signal Types
    enum class EmergencySignal : uint8_t { OFF = 0, ON = 1 };

    // Lamp Command Structure
#pragma pack(push, 1)
    struct LampCommand {
        TurnSignal turn_signal; // 0: NONE, 1: LEFT, 2: RIGHT
        EmergencySignal emergency_signal; // 0: OFF, 1: ON
    };
#pragma pack(pop)

    TurnSignalLampControl() = delete; // Prevent default constructor

    /**
     * @brief TurnSignalLampControl 클래스의 생성자
     * @param ip_address UDP 전송을 위한 IP 주소
     * @param dest_port UDP 전송을 위한 목적지 포트 번호
     */
    TurnSignalLampControl(const std::string& ip_address, uint16_t dest_port);

    /**
     * @brief TurnSignalLampControl 클래스의 소멸자
     */
    ~TurnSignalLampControl();

    /**
     * @brief 램프 제어 명령을 전송합니다
     * @param command 전송할 제어 명령
     * @return 전송 성공 여부
     */
    bool SendLampCommand(const LampCommand& command);

private:
    std::unique_ptr<UDPSender> udp_sender_;
    bool PackLampCommand(const LampCommand& command, uint8_t* buffer);
};

} // namespace MoraiCppUdp
#endif // __TURN_SIGNAL_LAMP_CTL_HPP__