#include "actuator/ego_ctl.hpp"

#include <cstring>
#include <iostream>

using namespace MoraiCppUdp;

EgoControl::EgoControl(const std::string& ip_address, uint16_t dest_port)
{
    udp_sender_ = std::make_unique<UDPSender>(ip_address, dest_port);
}

EgoControl::~EgoControl()
{
    // UDPSender will be automatically destroyed by unique_ptr
}

bool EgoControl::SendControlCommand(const ControlCommand& command)
{
    uint8_t buffer[PACKET_SIZE] = {0};
    
    if (!PackControlCommand(command, buffer))
    {
        std::cerr << "Failed to pack control command" << std::endl;
        return false;
    }

    size_t sent_size = 0;
    if (!udp_sender_->Send(reinterpret_cast<char*>(buffer), PACKET_SIZE))
    {
        std::cerr << "Failed to send control command" << std::endl;
        return false;
    }

    return true;
}

bool EgoControl::PackControlCommand(const ControlCommand& command, uint8_t* buffer)
{
    if (buffer == nullptr)
    {
        return false;
    }

    // Header '#' (1 byte)
    buffer[0] = '#';

    // "MoraiCtrlCmd" (12 bytes)
    const char* cmd_str = "MoraiCtrlCmd";
    std::memcpy(&buffer[1], cmd_str, 12);

    // '$' (1 byte)
    buffer[13] = '$';

    // Data Length (4 bytes) - length of remaining data
    uint32_t data_length = 23;  // 12(Aux) + 3(Control) + 20(Data)
    std::memcpy(&buffer[14], &data_length, sizeof(uint32_t));

    // Aux Data (12 bytes) - Reserved for future use
    std::memset(&buffer[18], 0, 12);

    // Control Data (3 bytes)
    buffer[30] = static_cast<uint8_t>(command.ctrl_mode);    // Control Mode
    buffer[31] = static_cast<uint8_t>(command.gear);         // Gear
    buffer[32] = static_cast<uint8_t>(command.long_cmd_type);// Command Type

    // Vehicle Control Data (20 bytes = 5 * 4 bytes)
    std::memcpy(&buffer[33], &command.velocity, sizeof(float));      // Velocity
    std::memcpy(&buffer[37], &command.acceleration, sizeof(float));  // Acceleration
    std::memcpy(&buffer[41], &command.accel, sizeof(float));        // Accelerator
    std::memcpy(&buffer[45], &command.brake, sizeof(float));        // Brake
    std::memcpy(&buffer[49], &command.steering, sizeof(float));     // Steering

    // Tail "0x0D 0x0A" (2 bytes)
    buffer[53] = 0x0D;
    buffer[54] = 0x0A;

    return true;
}
