#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "sensors/object_info.hpp"

// Object 타입을 문자열로 변환하는 헬퍼 함수
std::string ObjectTypeToString(int16_t type) {
    switch(type) {
        case ObjectInfo::TYPE_EGO: return "EGO";
        case ObjectInfo::TYPE_PEDESTRIAN: return "PEDESTRIAN";
        case ObjectInfo::TYPE_VEHICLE: return "VEHICLE";
        case ObjectInfo::TYPE_OBJECT: return "OBJECT";
        default: return "UNKNOWN";
    }
}

// Vector3 형식 출력을 위한 헬퍼 함수
void PrintVector3(const std::string& name, float x, float y, float z, 
                 const std::string& unit = "", int precision = 3) {
    std::cout << name << ":\n"
              << "\tx: " << std::fixed << std::setprecision(precision) << x << unit << "\n"
              << "\ty: " << std::fixed << std::setprecision(precision) << y << unit << "\n"
              << "\tz: " << std::fixed << std::setprecision(precision) << z << unit << "\n";
}

void PrintUsage() {
    std::cout << "Usage: object_info_receiver <ip_address> <port>" << std::endl;
    std::cout << "Example: object_info_receiver 127.0.0.1 7777" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        PrintUsage();
        return 1;
    }

    std::string ip_address = argv[1];
    uint16_t port = static_cast<uint16_t>(std::stoi(argv[2]));

    try
    {
        std::cout << "Object Info receiver example" << std::endl;
        std::cout << "Connecting to " << ip_address << ":" << port << std::endl;

        ObjectInfo object_info(ip_address, port);
        std::vector<ObjectInfo::ObjectData> objects;

        std::cout << "Waiting for object data..." << std::endl;

        while (true)
        {
            if (object_info.GetObjectInfo(objects))
            {
                // Clear screen for better visibility
                #ifdef _WIN32
                system("cls");
                #else
                system("clear");
                #endif

                std::cout << "\n=== Object Information ===" << std::endl;
                std::cout << "Number of objects: " << objects.size() << std::endl;

                for (size_t i = 0; i < objects.size(); ++i)
                {
                    const auto& obj = objects[i];
                    std::cout << "\nObject #" << i + 1 << std::endl;
                    std::cout << "ID: " << obj.obj_id << std::endl;
                    std::cout << "Type: " << obj.obj_type 
                              << " (" << ObjectTypeToString(obj.obj_type) << ")" << std::endl;

                    PrintVector3("Position", obj.pos_x, obj.pos_y, obj.pos_z, " m");
                    std::cout << "Heading: " << obj.heading << " deg" << std::endl;
                    PrintVector3("Size", obj.size_x, obj.size_y, obj.size_z, " m");
                    
                    if (obj.obj_type == ObjectInfo::TYPE_VEHICLE) {
                        std::cout << "Vehicle Specific Data:" << std::endl;
                        std::cout << "\tOverhang: " << obj.overhang << " m" << std::endl;
                        std::cout << "\tWheelbase: " << obj.wheelbase << " m" << std::endl;
                        std::cout << "\tRear Overhang: " << obj.rear_overhang << " m" << std::endl;
                    }

                    PrintVector3("Velocity", obj.velocity_x, obj.velocity_y, obj.velocity_z, " km/h");
                    PrintVector3("Acceleration", obj.accel_x, obj.accel_y, obj.accel_z, " m/s²");

                    if (obj.obj_type == ObjectInfo::TYPE_VEHICLE) {
                        std::cout << "MGeo Link ID: " << obj.link_id << std::endl;
                    }
                    
                    std::cout << "------------------------" << std::endl;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 