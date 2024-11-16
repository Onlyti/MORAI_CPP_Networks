#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include "sensors/gps.hpp"

int main(int argc, char** argv)
{
    int port = argc > 1 ? std::stoi(argv[1]) : 7779;  // GPS의 기본 포트는 7779로 설정

    printf("port: %d\n", port);
    try
    {
        std::cout << "GPS receiver example" << std::endl;
        // UDP 수신을 위한 GPS 객체 생성 (IP와 포트 설정)
        GPS gps("127.0.0.1", port);  // 실제 MORAI 시뮬레이터의 IP와 포트로 변경하세요

        std::cout << "Waiting for GPS data..." << std::endl;

        while (true)
        {
            GPS::GPSData gps_data;

            // GPS 데이터 수신
            if (gps.GetGPSData(gps_data))
            {
                // 데이터 출력 (소수점 6자리까지)
                std::cout << "\n=== GPS Data ===" << std::endl;
                std::cout << "Format Type: " << gps_data.sentence_type << std::endl;
                std::cout << "Timestamp: " << std::fixed << std::setprecision(1) 
                         << gps_data.timestamp << std::endl;
                std::cout << "Position:" << std::endl;
                std::cout << "\tLatitude: " << std::fixed << std::setprecision(6) 
                         << gps_data.latitude << std::endl;
                std::cout << "\tLongitude: " << std::fixed << std::setprecision(6) 
                         << gps_data.longitude << std::endl;
                std::cout << "\tAltitude: " << std::fixed << std::setprecision(2) 
                         << gps_data.altitude << " m" << std::endl;

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
                std::cout << "Waiting for GPS data..." << std::endl;
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