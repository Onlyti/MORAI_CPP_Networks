#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "sensors/imu.hpp"

int main(int argc, char** argv)
{
    int port = argc > 1 ? std::stoi(argv[1]) : 7778;  // IMU의 기본 포트는 7778로 설정

    printf("port: %d\n", port);
    try
    {
        std::cout << "IMU receiver example" << std::endl;
        // UDP 수신을 위한 IMU 객체 생성 (IP와 포트 설정2
        IMU imu("127.0.0.1", port);  // 실제 MORAI 시뮬레이터의 IP와 포트로 변경하세요

        std::cout << "Waiting for IMU data..." << std::endl;

        while (true)
        {
            IMU::IMUData imu_data;

            // IMU 데이터 수신
            if (imu.GetIMUData(imu_data))
            {
                // 데이터 출력 (소수점 3자리까지)
                std::cout << "\n=== IMU Data ===" << std::endl;

                // Orientation (Quaternion)
                std::cout << "Orientation (Quaternion):" << std::endl;
                std::cout << "\tw: " << std::fixed << std::setprecision(3) << imu_data.w
                          << std::endl;
                std::cout << "\tx: " << std::fixed << std::setprecision(3) << imu_data.x
                          << std::endl;
                std::cout << "\ty: " << std::fixed << std::setprecision(3) << imu_data.y
                          << std::endl;
                std::cout << "\tz: " << std::fixed << std::setprecision(3) << imu_data.z
                          << std::endl;

                // Angular Velocity
                std::cout << "Angular Velocity (rad/s):" << std::endl;
                std::cout << "\tx: " << std::fixed << std::setprecision(3)
                          << imu_data.angular_velocity_x << std::endl;
                std::cout << "\ty: " << std::fixed << std::setprecision(3)
                          << imu_data.angular_velocity_y << std::endl;
                std::cout << "\tz: " << std::fixed << std::setprecision(3)
                          << imu_data.angular_velocity_z << std::endl;

                // Linear Acceleration
                std::cout << "Linear Acceleration (m/s2):" << std::endl;
                std::cout << "\tx: " << std::fixed << std::setprecision(3)
                          << imu_data.linear_acceleration_x << std::endl;
                std::cout << "\ty: " << std::fixed << std::setprecision(3)
                          << imu_data.linear_acceleration_y << std::endl;
                std::cout << "\tz: " << std::fixed << std::setprecision(3)
                          << imu_data.linear_acceleration_z << std::endl;

                // 화면을 더 읽기 쉽게 하기 위해 잠시 대기
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

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
                // std::cout << "Waiting for IMU data..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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