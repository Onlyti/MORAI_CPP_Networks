#include <chrono>
#include <iostream>
#include <thread>

#include "actuator/turn_signal_lamp_control.hpp"

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7601" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        PrintUsage(argv[0]);
        return -1;
    }

    const std::string ip_address = argv[1];
    const int dest_port = std::stoi(argv[2]);

    try
    {
        MoraiCppUdp::TurnSignalLampControl lamp_control(ip_address, dest_port);
        std::cout << "UDP Client Info - IP: " << ip_address << ", Dest Port: " << dest_port << std::endl;

        MoraiCppUdp::TurnSignalLampControl::LampCommand cmd;
        cmd.turn_signal = MoraiCppUdp::TurnSignalLampControl::TurnSignal::NONE;
        cmd.emergency_signal = MoraiCppUdp::TurnSignalLampControl::EmergencySignal::OFF;

        std::cout << "Starting turn signal lamp control example..." << std::endl;

        // Left signal test
        std::cout << "Testing left turn signal..." << std::endl;
        cmd.turn_signal = MoraiCppUdp::TurnSignalLampControl::TurnSignal::LEFT;
        for (int i = 0; i < 30; i++)
        {
            if (!lamp_control.SendLampCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Right signal test
        std::cout << "Testing right turn signal..." << std::endl;
        cmd.turn_signal = MoraiCppUdp::TurnSignalLampControl::TurnSignal::RIGHT;
        for (int i = 0; i < 30; i++)
        {
            if (!lamp_control.SendLampCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Emergency signal test
        std::cout << "Testing emergency signal..." << std::endl;
        cmd.turn_signal = MoraiCppUdp::TurnSignalLampControl::TurnSignal::NONE;
        cmd.emergency_signal = MoraiCppUdp::TurnSignalLampControl::EmergencySignal::ON;
        for (int i = 0; i < 30; i++)
        {
            if (!lamp_control.SendLampCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Turn off all signals
        std::cout << "Turning off all signals..." << std::endl;
        cmd.turn_signal = MoraiCppUdp::TurnSignalLampControl::TurnSignal::NONE;
        cmd.emergency_signal = MoraiCppUdp::TurnSignalLampControl::EmergencySignal::OFF;
        if (!lamp_control.SendLampCommand(cmd))
        {
            std::cerr << "Failed to send command" << std::endl;
            return -1;
        }

        std::cout << "Example completed successfully!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
} 