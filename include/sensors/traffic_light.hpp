#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <functional>
#include <iostream>
#include <atomic>

#include "network/udp_receiver.hpp"

class TrafficLight : public UDPReceiver {
public:
    static constexpr size_t PACKET_SIZE = 48; // 전체 패킷 크기

    struct TrafficLightData {
        char traffic_light_index[12]; // 신호등 ID
        int16_t traffic_light_type;   // 신호등 종류
        int16_t traffic_light_status; // 신호 상태
    };

    // 콜백 함수 타입 정의
    using TrafficLightCallback = std::function<void(const TrafficLightData&)>;

#pragma pack(push, 1)
    struct TrafficLightPacket {
        uint8_t sharp;                       // '#'
        char header[12];                     // "TrafficLight"
        uint8_t dollar;                      // '$'
        uint32_t length;                     // 데이터 길이
        uint8_t AuxData[12];                 // 보조 데이터
        TrafficLightData traffic_light_data; // 실제 데이터
        uint8_t tail[2];                     // 0x0D, 0x0A
    };
#pragma pack(pop)

    union TrafficLightPacketStruct {
        TrafficLightPacket packet;
        char buffer[sizeof(TrafficLightPacket)];
    };

    TrafficLight(const std::string& ip_address, uint16_t port);
    virtual ~TrafficLight();

    // 콜백 등록 함수
    void RegisterCallback(TrafficLightCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        traffic_light_callback_ = callback;
    }

private:
    void ThreadTrafficLightReceiver();
    bool ParseTrafficLight(const char* buffer, size_t size, TrafficLightPacketStruct& data);

    std::thread thread_traffic_light_receiver_;
    std::mutex callback_mutex_;
    std::atomic<bool> is_running_{false};

    TrafficLightPacketStruct packet_data_;
    TrafficLightCallback traffic_light_callback_{[](const TrafficLightData& data) {
        std::cout << "Note: Traffic light data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCallback()." << std::endl;
    }};
};