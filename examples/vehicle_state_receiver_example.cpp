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
    std::cout << name << ":" << "\n"
              << "\tx: " << std::fixed << std::setprecision(precision) << vec.x
              << unit << "\n"
              << "\ty: " << std::fixed << std::setprecision(precision) << vec.y
              << unit << "\n"
              << "\tz: " << std::fixed << std::setprecision(precision) << vec.z
              << unit << "\n";
}

int main(int argc, char** argv)
{
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    int port = argc > 2 ? std::stoi(argv[2]) : 7777;  // Vehicle State의 기본 포트는 7777로 설정

    try
    {
        std::cout << "Vehicle State receiver example" << std::endl;
        std::cout << "Port: " << port << std::endl;

        // UDP 수신을 위한 VehicleState 객체 생성
        VehicleState vehicle_state(ip, port);

        std::cout << "Waiting for vehicle state data..." << std::endl;

        while (true)
        {
            VehicleState::VehicleData data;
            VehicleState::VehicleStatePacketStruct packet_data;

            if (vehicle_state.GetVehicleState(data))
            {
                packet_data = vehicle_state.packet_data_;
                // Clear screen for better visibility (Windows)
                #ifdef _WIN32
                system("cls");
                #else
                system("clear");
                #endif

                std::cout << "\n=== Vehicle State Data ===" << "\n";

                // Timestamp
                int offset = 1;
                offset += 1 + 9 + 1 + 4 + 12; // sharp + header + dollar + length + aux_data
                std::cout << "Timestamp:" << "\n"
                         << "\tSeconds: " << data.timestamp.seconds << "\n"
                         << "\t\tSecondsPacket: 0x" << std::hex << static_cast<int>(packet_data.data[offset+0]) <<", " << static_cast<int>(packet_data.data[offset+1])<< ", " << static_cast<int>(packet_data.data[offset+2]) << ", " << static_cast<int>(packet_data.data[offset+3]) << ", " << std::dec << "\n"
                         << "\tNanoseconds: " << data.timestamp.nanoseconds << "\n"
                         << "\t\tNanosecondsPacket: 0x" << std::hex << static_cast<int>(packet_data.data[offset+4]) <<", " << static_cast<int>(packet_data.data[offset+5]) << ", " << static_cast<int>(packet_data.data[offset+6]) << ", " << static_cast<int>(packet_data.data[offset+7]) << ", " << std::dec << "\n";
                offset += 4; // seconds
                offset += 4; // nanoseconds

                // Control Mode and Gear
                std::cout << "Control Mode: " << ControlModeToString(data.ctrl_mode) << "\n"
                         << "\tCtrlModePacket: 0x" << std::hex << static_cast<int>(packet_data.data[offset]) << std::dec << "\n"
                         << "Gear: " << GearToString(data.gear) << "\n"
                         << "\tGearPacket: 0x" << std::hex << static_cast<int>(packet_data.data[offset+1]) << std::dec << "\n";
                offset += 1; // ctrl_mode
                offset += 1; // gear

                // Vehicle Speed and Map Data
                std::cout << "Signed Velocity: " << std::fixed << std::setprecision(2) 
                         << data.signed_velocity << " km/h" << "\n"
                         << "\tSignedVelocityPacket: 0x" << std::hex 
                         << static_cast<int>(packet_data.data[offset]) << ", "
                         << static_cast<int>(packet_data.data[offset+1]) << ", "
                         << static_cast<int>(packet_data.data[offset+2]) << ", "
                         << static_cast<int>(packet_data.data[offset+3]) 
                         << std::dec << "\n";
                offset += 4; // signed_velocity
                std::cout << "Map Data ID: " << data.map_data_id 
                         << (data.map_data_id < 10000 ? " (DigitalTwin)" : " (Virtual)") 
                         << "\tMapDataIDPacket: 0x" << std::hex 
                         << static_cast<int>(packet_data.data[offset]) << ", "
                         << static_cast<int>(packet_data.data[offset+1]) << ", "
                         << static_cast<int>(packet_data.data[offset+2]) << ", "
                         << static_cast<int>(packet_data.data[offset+3]) 
                         << std::dec << "\n";
                offset += 4; // map_data_id

                // Pedal Inputs
                std::cout << "Pedal Inputs:" << "\n"
                         << "\tAccel: " << std::fixed << std::setprecision(3) 
                         << data.accel_input << "\n"
                         << "\tAccelPacket: 0x" << std::hex 
                         << static_cast<int>(packet_data.data[offset]) << ", "
                         << static_cast<int>(packet_data.data[offset+1]) << ", "
                         << static_cast<int>(packet_data.data[offset+2]) << ", "
                         << static_cast<int>(packet_data.data[offset+3]) 
                         << std::dec << "\n"
                         << "\tBrake: " << std::fixed << std::setprecision(3) 
                         << data.brake_input << "\n"
                         << "\tBrakePacket: 0x" << std::hex 
                         << static_cast<int>(packet_data.data[offset+6]) << ", "
                         << static_cast<int>(packet_data.data[offset+7]) << ", "
                         << static_cast<int>(packet_data.data[offset+8]) << ", "
                         << static_cast<int>(packet_data.data[offset+9]) 
                         << std::dec << "\n";
                offset += 4; // accel
                offset += 4; // brake

                // Vehicle Dimensions
                // PrintVector3("Vehicle Size", data.size, "m");

                char size_x_packet[4];
                memcpy(size_x_packet, &packet_data.data[offset], 4);
                offset += 4; // size_x
                char size_y_packet[4];
                memcpy(size_y_packet, &packet_data.data[offset], 4);
                offset += 4; // size_y
                char size_z_packet[4];
                memcpy(size_z_packet, &packet_data.data[offset], 4);
                offset += 4; // size_z
                std::cout << "Size:" << "\n"
                         << "\tX: " << std::fixed << std::setprecision(3) 
                         << data.size_x << "m" << "\n"
                         << "\tSizeXPacket: 0x" << std::hex << static_cast<int>(*size_x_packet) << std::dec << "\n"
                         << "\tY: " << std::fixed << std::setprecision(3) 
                         << data.size_y << "m" << "\n"
                         << "\tSizeYPacket: 0x" << std::hex << static_cast<int>(*size_y_packet) << std::dec << "\n"
                         << "\tZ: " << std::fixed << std::setprecision(3) 
                         << data.size_z << "m" << "\n"
                         << "\tSizeZPacket: 0x" << std::hex << static_cast<int>(*size_z_packet) << std::dec << "\n";

                char overhang_packet[4];
                memcpy(overhang_packet, &packet_data.data[offset], 4);
                offset += 4; // overhang
                std::cout << "Overhang: " << std::fixed << std::setprecision(3) 
                         << data.overhang << "m" << "\n"
                         << "\tOverhangPacket: 0x" << std::hex << static_cast<int>(*overhang_packet) << std::dec << "\n";
                char wheelbase_packet[4];
                memcpy(wheelbase_packet, &packet_data.data[offset], 4);
                offset += 4; // wheelbase
                std::cout << "Wheelbase: " << std::fixed << std::setprecision(3) 
                         << data.wheelbase << "m" << "\n"
                         << "\tWheelbasePacket: 0x" << std::hex << static_cast<int>(*wheelbase_packet) << std::dec << "\n";
                char rear_overhang_packet[4];
                memcpy(rear_overhang_packet, &packet_data.data[offset], 4);
                offset += 4; // rear_overhang
                std::cout << "Rear Overhang: " << std::fixed << std::setprecision(3) 
                         << data.rear_overhang << "m" << "\n"
                         << "\tRearOverhangPacket: 0x" << std::hex << static_cast<int>(*rear_overhang_packet) << std::dec << "\n";

                // Vehicle State
                PrintVector3("Position", data.position, "m");
                PrintVector3("Rotation", data.rotation, "deg");
                PrintVector3("Velocity", data.velocity, "km/h");
                PrintVector3("Angular Velocity", data.angular_velocity, "deg/s");
                PrintVector3("Acceleration", data.acceleration, "m/s2");

                // Steering and Link ID
                std::cout << "Steering: " << std::fixed << std::setprecision(2) 
                         << data.steering << " deg" << "\n";

                std::cout << "MGeo Link ID: " << data.mgeo_link_id << "\n";

                // tire data
                std::cout << "Tire Data:" << "\n";
                std::cout << "\tLateral Force FL: " << data.tire_lateral_force_fl << " N" << "\n";
                std::cout << "\tLateral Force FR: " << data.tire_lateral_force_fr << " N" << "\n";
                std::cout << "\tLateral Force RL: " << data.tire_lateral_force_rl << " N" << "\n";
                std::cout << "\tLateral Force RR: " << data.tire_lateral_force_rr << " N" << "\n";

                // side slip angle data
                std::cout << "Side Slip Angle:" << "\n";
                std::cout << "\tSide Slip Angle FL: " << data.side_slip_angle_fl << " deg" << "\n";
                std::cout << "\tSide Slip Angle FR: " << data.side_slip_angle_fr << " deg" << "\n";
                std::cout << "\tSide Slip Angle RL: " << data.side_slip_angle_rl << " deg" << "\n";
                std::cout << "\tSide Slip Angle RR: " << data.side_slip_angle_rr << " deg" << "\n";

                // tire cornering stiffness data
                std::cout << "Tire Cornering Stiffness:" << "\n";
                std::cout << "\tTire Cornering Stiffness FL: " << data.tire_cornering_stiffness_fl << " N/deg" << "\n";
                std::cout << "\tTire Cornering Stiffness FR: " << data.tire_cornering_stiffness_fr << " N/deg" << "\n";
                std::cout << "\tTire Cornering Stiffness RL: " << data.tire_cornering_stiffness_rl << " N/deg" << "\n";
                std::cout << "\tTire Cornering Stiffness RR: " << data.tire_cornering_stiffness_rr << " N/deg" << "\n";
                
                std::cout << std::endl;

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