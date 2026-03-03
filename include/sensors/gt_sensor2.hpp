/**
 * @file gt_sensor2.hpp
 * @brief GT_Sensor2 class definition
 * @details This file contains the definition of the GT_Sensor2 class, which is used to receive and parse the GT_Sensor2 data from the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-02-27
 */
#pragma once

#ifndef __GT_SENSOR2_HPP__
#define __GT_SENSOR2_HPP__

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "network/udp_receiver.hpp"

namespace MoraiCppUdp{

class GT_Sensor2 : public UDPReceiver {
public:
    static constexpr size_t MAX_OBJECTS = 20;
    static constexpr size_t PACKET_SIZE = 2160; // 전체 패킷 크기
    struct Timestamp {
        #pragma pack(push, 1)
        int64_t sec;
        int32_t nsec;
        #pragma pack(pop)
    };
    struct ObjectData {
        #pragma pack(push, 1)
        char entity_id[24];  // Entity ID (24 bytes)
        int32_t obj_type;    // Object Type (4 bytes)
        float pos_x;         // Position X m (4 bytes)
        float pos_y;         // Position Y m (4 bytes)
        float pos_z;         // Position Z m (4 bytes)
        float roll;          // Roll (4 bytes)
        float pitch;         // Pitch (4 bytes)
        float yaw;           // Yaw (4 bytes)
        float length;        // Length m (4 bytes)
        float width;         // Width m (4 bytes)
        float height;        // Height m (4 bytes)
        float front;         // Front (4 bytes)
        float rear;          // Rear (4 bytes)
        float wheel_base;    // Wheel Base (4 bytes)
        float vel_x;         // Velocity X (4 bytes)
        float vel_y;         // Velocity Y (4 bytes)
        float vel_z;         // Velocity Z (4 bytes)
        float acc_x;         // Acceleration X (4 bytes)
        float acc_y;         // Acceleration Y (4 bytes)
        float acc_z;         // Acceleration Z (4 bytes)
        #pragma pack(pop)
    }; // Total: 106 bytes per object

    struct GT_Sensor2Data{
        #pragma pack(push, 1)
        Timestamp timestamp;
        int32_t num_objects;
        ObjectData objects[MAX_OBJECTS];
        #pragma pack(pop)
    };

    union GT_Sensor2PacketStruct {
        char buffer[sizeof(GT_Sensor2Data)];
        GT_Sensor2Data packet;
    };

    // 콜백 함수 타입 정의
    using GT_Sensor2Callback = std::function<void(const GT_Sensor2Data&)>;

    /**
     * @brief GT_Sensor2 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     */
    GT_Sensor2(const std::string& ip_address, uint16_t port);

    /**
     * @brief GT_Sensor2 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @param callback GT_Sensor2 데이터를 처리할 콜백 함수
     */
    GT_Sensor2(const std::string& ip_address, uint16_t port, GT_Sensor2Callback callback);

    /**
     * @brief GT_Sensor2 클래스의 소멸자
     * @note UDP 수신 스레드를 안전하게 종료합니다
     */
    virtual ~GT_Sensor2() = default;

    // 콜백 등록 함수
    void RegisterCallback(GT_Sensor2Callback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        gt_sensor2_callback_ = callback;
    }

private:
    void ThreadGT_Sensor2Receiver();
    bool ParseGT_Sensor2(const char* buffer, size_t size, GT_Sensor2Data& data);

    std::thread thread_gt_sensor2_receiver_;
    std::mutex callback_mutex_;
    std::atomic<bool> is_running_{false};
    bool is_data_received_;

    GT_Sensor2Data packet_data_;
    std::vector<ObjectData> gt_sensor2_data_;
    GT_Sensor2Callback gt_sensor2_callback_{[](const GT_Sensor2Data& data) {
        std::cout << "Note: GT_Sensor2 data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCallback()." << std::endl;
    }};
};

} // namespace MoraiCppUdp
#endif // __GT_SENSOR2_HPP__