#include "actuator/turn_signal_lamp_control.hpp"

#include <cstring>
#include <iostream>

using namespace MoraiCppUdp;

TurnSignalLampControl::TurnSignalLampControl(const std::string& ip_address, uint16_t dest_port) {
    udp_sender_ = std::make_unique<UDPSender>(ip_address, dest_port);
}

TurnSignalLampControl::~TurnSignalLampControl() {
    // UDPSender will be automatically destroyed by unique_ptr
}

bool TurnSignalLampControl::SendLampCommand(const LampCommand& command) {
    uint8_t buffer[PACKET_SIZE] = {0};

    if (!PackLampCommand(command, buffer)) {
        std::cerr << "Failed to pack lamp command" << std::endl;
        return false;
    }

    if (!udp_sender_->Send(reinterpret_cast<char*>(buffer), PACKET_SIZE)) {
        std::cerr << "Failed to send lamp command" << std::endl;
        return false;
    }

    return true;
}

bool TurnSignalLampControl::PackLampCommand(const LampCommand& command, uint8_t* buffer) {
    if (buffer == nullptr) {
        return false;
    }

    // Header '#' (1 byte)
    buffer[0] = '#';

    // "LampControl" (11 bytes)
    const char* cmd_str = "LampControl";
    std::memcpy(&buffer[1], cmd_str, 11);

    // '$' (1 byte)
    buffer[12] = '$';

    // Data Length (4 bytes) - length of remaining data
    uint32_t data_length = 14; // 12(Aux) + 2(Lamp Data)
    std::memcpy(&buffer[13], &data_length, sizeof(uint32_t));

    // Aux Data (12 bytes) - Reserved for future use
    std::memset(&buffer[17], 0, 12);

    // Lamp Control Data (2 bytes)
    buffer[29] = static_cast<uint8_t>(command.turn_signal);      // Turn Signal
    buffer[30] = static_cast<uint8_t>(command.emergency_signal); // Emergency Signal

    // Tail "0x0D 0x0A" (2 bytes)
    buffer[31] = 0x0D;
    buffer[32] = 0x0A;

    return true;
}