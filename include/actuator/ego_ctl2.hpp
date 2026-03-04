/**
 * @file ego_ctl2.hpp
 * @brief EgoControl2 class definition
 * @details This file contains the definition of the EgoControl2 class, which is used to send the EgoControl2 data to the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-03-04
 */
#ifndef __EGO_CTL2_HPP__
#define __EGO_CTL2_HPP__

#include <cstdint>
#include <memory>
#include <string>

#include "../network/udp_sender.hpp"

namespace MoraiCppUdp {

/**
 * @brief MORAI 시뮬레이터의 차량 제어를 위한 클래스
 * @details UDP를 통해 차량 제어 명령을 전송합니다
 */
class EgoControl2
{
    // Packet Structure Constants
    const static uint32_t PACKET_SIZE = 24;  // 24 bytes

public:
    // Control Command Structure
#pragma pack(push, 1)
    struct ControlCommand2 {
        double accel_input;     // 0 ~ 1
        double brake_input;     // 0 ~ 1
        double steering;        // -1 ~ 1 (실제 조향각 = steering * 최대조향각(36.25))
    };
#pragma pack(pop)

    EgoControl2() = delete;  // Prevent default constructor
    
    /**
     * @brief EgoControl 클래스의 생성자
     * @param ip_address UDP 전송을 위한 IP 주소
     * @param dest_port UDP 전송을 위한 목적지 포트 번호
     */
    EgoControl2(const std::string& ip_address, uint16_t dest_port);
    
    /**
     * @brief EgoControl 클래스의 소멸자
     */
    ~EgoControl2();

    /**
     * @brief 차량 제어 명령을 전송합니다
     * @param command 전송할 제어 명령
     * @return 전송 성공 여부
     */
    bool SendControlCommand(const ControlCommand2& command);

private:
    std::unique_ptr<UDPSender> udp_sender_;
    bool PackControlCommand(const ControlCommand2& command, uint8_t* buffer);
};

} // namespace MoraiCppUdp
#endif  // __EGO_CTL2_HPP__
