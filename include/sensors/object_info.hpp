#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "network/udp_receiver.hpp"

class ObjectInfo : public UDPReceiver {
public:
    static constexpr size_t MAX_OBJECTS = 20;
    static constexpr size_t PACKET_SIZE = 2160; // 전체 패킷 크기

    struct ObjectData {
        int16_t obj_id;      // Object ID (2 bytes)
        int16_t obj_type;    // Object Type -1:Ego, 0:Pedestrian, 1:Vehicle, 2:Object (2 bytes)
        float pos_x;         // Position X m (4 bytes)
        float pos_y;         // Position Y m (4 bytes)
        float pos_z;         // Position Z m (4 bytes)
        float heading;       // Heading deg (4 bytes)
        float size_x;        // Size X m (4 bytes)
        float size_y;        // Size Y m (4 bytes)
        float size_z;        // Size Z m (4 bytes)
        float overhang;      // Overhang m (4 bytes)
        float wheelbase;     // Wheelbase (4 bytes)
        float rear_overhang; // Rear Overhang (4 bytes)
        float velocity_x;    // Velocity X kph(4 bytes)
        float velocity_y;    // Velocity Y kph(4 bytes)
        float velocity_z;    // Velocity Z kph(4 bytes)
        float accel_x;       // Acceleration X m/s^2(4 bytes)
        float accel_y;       // Acceleration Y m/s^2(4 bytes)
        float accel_z;       // Acceleration Z m/s^2(4 bytes)
        char link_id[38];    // MGeo Link ID (38 bytes)
    }; // Total: 106 bytes per object

#pragma pack(push, 1)
    struct ObjectInfoPacket {
        uint8_t sharp;                   // '#' (1 byte)
        char header[12];                 // "MoraiObjInfo" (12 bytes)
        uint8_t dollar;                  // '$' (1 byte)
        uint32_t length;                 // Data Length (4 bytes)
        uint8_t aux_data[12];            // Aux Data (12 bytes)
        uint32_t timestamp_sec;          // Timestamp seconds (4 bytes)
        uint32_t timestamp_nsec;         // Timestamp nanoseconds (4 bytes)
        ObjectData objects[MAX_OBJECTS]; // Object Data Array (106 * 20 = 2120 bytes)
        uint8_t tail[2];                 // 0x0D, 0x0A (2 bytes)
    };
#pragma pack(pop)

    union ObjectInfoPacketStruct {
        ObjectInfoPacket packet;
        char buffer[sizeof(ObjectInfoPacket)];
    };

    ObjectInfo(const std::string& ip_address, uint16_t port);
    virtual ~ObjectInfo();

    bool GetObjectInfo(std::vector<ObjectData>& data);

    // Object type constants
    static constexpr int16_t TYPE_EGO = -1;
    static constexpr int16_t TYPE_PEDESTRIAN = 0;
    static constexpr int16_t TYPE_VEHICLE = 1;
    static constexpr int16_t TYPE_OBJECT = 2;

private:
    void ThreadObjectInfoReceiver();
    bool ParseObjectInfo(const char* buffer, size_t size, ObjectInfoPacketStruct& data);

    std::thread thread_object_info_receiver_;
    std::mutex mutex_object_data_;
    bool is_running_;
    bool is_data_received_;

    ObjectInfoPacketStruct packet_data_;
    std::vector<ObjectData> object_data_;
};