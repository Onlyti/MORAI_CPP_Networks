/**
 * @file ego_ctl2.cpp
 * @brief EgoControl2 class implementation
 * @details This file contains the implementation of the EgoControl2 class, which is used to send the EgoControl2 data to the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-03-04
 */
#include "actuator/ego_ctl2.hpp"

#include <cstring>
#include <iostream>

using namespace MoraiCppUdp;

EgoControl2::EgoControl2(const std::string& ip_address, uint16_t dest_port)
{
    udp_sender_ = std::make_unique<UDPSender>(ip_address, dest_port);
}

EgoControl2::~EgoControl2()
{
    // UDPSender will be automatically destroyed by unique_ptr
}

bool EgoControl2::SendControlCommand(const ControlCommand2& command)
{
    uint8_t buffer[PACKET_SIZE] = {0};

    if (!PackControlCommand(command, buffer))
    {
        std::cerr << "Failed to pack control command2" << std::endl;
        return false;
    }

    if (!udp_sender_)
    {
        std::cerr << "UDP sender is not initialized" << std::endl;
        return false;
    }

    if (!udp_sender_->Send(reinterpret_cast<char*>(buffer), PACKET_SIZE))
    {
        std::cerr << "Failed to send control command2" << std::endl;
        return false;
    }

    return true;
}

bool EgoControl2::PackControlCommand(const ControlCommand2& command, uint8_t* buffer)
{
    if (buffer == nullptr)
    {
        return false;
    }

    // Vehicle control packet layout (24 bytes total):
    // [0..7]   accel_input  (double)
    // [8..15]  brake_input  (double)
    // [16..23] steering     (double)
    std::memcpy(&buffer[0], &command.accel_input, sizeof(double));
    std::memcpy(&buffer[8], &command.brake_input, sizeof(double));
    std::memcpy(&buffer[16], &command.steering, sizeof(double));

    return true;
}
