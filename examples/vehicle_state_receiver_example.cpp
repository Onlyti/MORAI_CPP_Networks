#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include "sensors/vehicle_state.hpp"

// 기어 상태를 문자열로 변환하는 헬퍼 함수
std::string GearToString(VehicleState::GearMode gear) {
    switch (gear) {
        case VehicleState::GearMode::MANUAL: return "MANUAL";
        case VehicleState::GearMode::PARKING: return "PARKING";
        case VehicleState::GearMode::REVERSE: return "REVERSE";
        case VehicleState::GearMode::NEUTRAL: return "NEUTRAL";
        case VehicleState::GearMode::DRIVE: return "DRIVE";
        case VehicleState::GearMode::LOW: return "LOW";
        default: return "UNKNOWN";
    }
}

// 제어 모드를 문자열로 변환하는 헬퍼 함수
std::string ControlModeToString(VehicleState::ControlMode mode) {
    switch (mode) {
        case VehicleState::ControlMode::MORAI_AI: return "MORAI_AI";
        case VehicleState::ControlMode::KEYBOARD: return "KEYBOARD";
        case VehicleState::ControlMode::AUTO: return "AUTO";
        default: return "UNKNOWN";
    }
}

// Vector3 출력을 위한 헬퍼 함수
void PrintVector3(const std::string& name, const VehicleState::Vector3& vec, 
                 const std::string& unit = "", int precision = 3) {
    std::cout << name << ":" << std::endl
              << "\tx: " << std::fixed << std::setprecision(precision) << vec.x
              << unit << std::endl
              << "\ty: " << std::fixed << std::setprecision(precision) << vec.y
              << unit << std::endl
              << "\tz: " << std::fixed << std::setprecision(precision) << vec.z
              << unit << std::endl;
}

int main(int argc, char** argv)
{
    int port = argc > 1 ? std::stoi(argv[1]) : 7777;  // Vehicle State의 기본 포트는 7777로 설정

    try
    {
        std::cout << "Vehicle State receiver example" << std::endl;
        std::cout << "Port: " << port << std::endl;

        // UDP 수신을 위한 VehicleState 객체 생성
        VehicleState vehicle_state("127.0.0.1", port);

        std::cout << "Waiting for vehicle state data..." << std::endl;

        while (true)
        {
            VehicleState::VehicleData data;

            if (vehicle_state.GetVehicleState(data))
            {
                // Clear screen for better visibility (Windows)
                #ifdef _WIN32
                system("cls");
                #else
                system("clear");
                #endif

                std::cout << "\n=== Vehicle State Data ===" << std::endl;

                // Timestamp
                std::cout << "Timestamp:" << std::endl
                         << "\tSeconds: " << data.timestamp.seconds << std::endl
                         << "\tNanoseconds: " << data.timestamp.nanoseconds << std::endl;

                // Control Mode and Gear
                std::cout << "Control Mode: " << ControlModeToString(data.ctrl_mode) << std::endl;
                std::cout << "Gear: " << GearToString(data.gear) << std::endl;

                // Vehicle Speed and Map Data
                std::cout << "Signed Velocity: " << std::fixed << std::setprecision(2) 
                         << data.signed_velocity << " km/h" << std::endl;
                std::cout << "Map Data ID: " << data.map_data_id 
                         << (data.map_data_id < 10000 ? " (DigitalTwin)" : " (Virtual)") 
                         << std::endl;

                // Pedal Inputs
                std::cout << "Pedal Inputs:" << std::endl
                         << "\tAccel: " << std::fixed << std::setprecision(3) 
                         << data.accel_input << std::endl
                         << "\tBrake: " << std::fixed << std::setprecision(3) 
                         << data.brake_input << std::endl;

                // Vehicle Dimensions
                PrintVector3("Vehicle Size", data.size, "m");
                std::cout << "Overhang: " << std::fixed << std::setprecision(3) 
                         << data.overhang << "m" << std::endl;
                std::cout << "Wheelbase: " << std::fixed << std::setprecision(3) 
                         << data.wheelbase << "m" << std::endl;
                std::cout << "Rear Overhang: " << std::fixed << std::setprecision(3) 
                         << data.rear_overhang << "m" << std::endl;

                // Vehicle State
                PrintVector3("Position", data.position, "m");
                PrintVector3("Rotation", data.rotation, "deg");
                PrintVector3("Velocity", data.velocity, "km/h");
                PrintVector3("Angular Velocity", data.angular_velocity, "deg/s");
                PrintVector3("Acceleration", data.acceleration, "m/s2");

                // Steering and Link ID
                std::cout << "Steering: " << std::fixed << std::setprecision(2) 
                         << data.steering << " deg" << std::endl;

                std::cout << "MGeo Link ID: " << data.mgeo_link_id << std::endl;

                // 화면을 더 읽기 쉽게 하기 위해 잠시 대기
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // 'q' 키를 누르면 종료 (Windows에서만 작동)
                #ifdef _WIN32
                if (GetAsyncKeyState('Q') & 0x8000)
                {
                    break;
                }
                #endif
            }
            else
            {
                std::cout << "Waiting for vehicle state data..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
} 