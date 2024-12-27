#include <chrono>
#include <iostream>
#include <thread>

#include "actuator/ego_ctl.hpp"

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
        // Create EgoControl instance with command line arguments
        EgoControl ego_control(ip_address, dest_port);

        std::cout << "UDP Client Info - IP: " << ip_address << ", Dest Port: " << dest_port << std::endl;

        // Create control command
        EgoControl::ControlCommand cmd;
        cmd.ctrl_mode = EgoControl::ControlMode::AUTO_MODE;
        cmd.gear = EgoControl::GearCommand::DRIVE;
        cmd.long_cmd_type = EgoControl::LongitudinalCommandType::VELOCITY;

        // Initial values
        cmd.velocity = 0.0f;      // km/h
        cmd.acceleration = 0.0f;   // m/s^2
        cmd.accel = 0.0f;         // 0 ~ 1
        cmd.brake = 0.0f;         // 0 ~ 1
        cmd.steering = 0.0f;      // -1 ~ 1

        std::cout << "Starting ego control example..." << std::endl;
        std::cout << "Will perform: acceleration -> cruise -> turn -> stop" << std::endl;

        // Acceleration phase
        std::cout << "Accelerating to 30 km/h..." << std::endl;
        cmd.velocity = 30.0f;
        for (int i = 0; i < 100; i++)
        {
            if (!ego_control.SendControlCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cruise phase
        std::cout << "Cruising..." << std::endl;
        for (int i = 0; i < 50; i++)
        {
            if (!ego_control.SendControlCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Turn phase
        std::cout << "Turning right..." << std::endl;
        cmd.steering = 0.5f;  // Turn right (약 18도)
        for (int i = 0; i < 30; i++)
        {
            if (!ego_control.SendControlCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Return to straight
        std::cout << "Returning to straight..." << std::endl;
        cmd.steering = 0.0f;
        for (int i = 0; i < 20; i++)
        {
            if (!ego_control.SendControlCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop phase
        std::cout << "Stopping..." << std::endl;
        cmd.velocity = 0.0f;
        for (int i = 0; i < 50; i++)
        {
            if (!ego_control.SendControlCommand(cmd))
            {
                std::cerr << "Failed to send command" << std::endl;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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