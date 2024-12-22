#include <cstring>
#include <iostream>

#include "sensors/object_info.hpp"

ObjectInfo::ObjectInfo(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false) {
    is_running_ = true;
    object_data_.reserve(MAX_OBJECTS);
    thread_object_info_receiver_ = std::thread(&ObjectInfo::ThreadObjectInfoReceiver, this);
}

ObjectInfo::~ObjectInfo() {
    is_running_ = false;
    if (thread_object_info_receiver_.joinable()) {
        thread_object_info_receiver_.join();
    }
    Close();
}

void ObjectInfo::ThreadObjectInfoReceiver() {
    char packet_buffer[PACKET_SIZE];

    while (is_running_) {
        try {
            size_t received_size = 0;
            if (!Receive(packet_buffer, PACKET_SIZE, received_size)) {
                std::cerr << "Failed to receive object info data" << std::endl;
                continue;
            }

            memset(&packet_data_, 0, sizeof(ObjectInfoPacketStruct));
            if (ParseObjectInfo(packet_buffer, received_size, packet_data_)) {
                std::vector<ObjectData> temp_data;
                temp_data.clear();
                for (size_t i = 0; i < MAX_OBJECTS; ++i) {
                    temp_data.push_back(packet_data_.packet.objects[i]);
                }

                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (object_callback_) {
                    object_callback_(temp_data);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Object info error: " << e.what() << std::endl;
            continue;
        }
    }
}

bool ObjectInfo::ParseObjectInfo(const char* buffer, size_t size, ObjectInfoPacketStruct& data) {
    if (size != PACKET_SIZE) {
        return false;
    }

    size_t offset = 0;

    // Parse Sharp (1 byte) - "#"
    std::memcpy(&data.packet.sharp, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    if (data.packet.sharp != '#') {
        std::cerr << "Invalid sharp character" << std::endl;
        return false;
    }

    // Parse Header (12 bytes) - "MoraiObjInfo"
    std::memcpy(&data.packet.header, buffer + offset, sizeof(char[12]));
    offset += sizeof(char[12]);
    if (strncmp(data.packet.header, "MoraiObjInfo", 12) != 0) {
        std::cerr << "Invalid header" << std::endl;
        return false;
    }

    // Parse Dollar (1 byte) - "$"
    std::memcpy(&data.packet.dollar, buffer + offset, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    if (data.packet.dollar != '$') {
        std::cerr << "Invalid dollar character" << std::endl;
        return false;
    }

    // Parse Length (4 bytes)
    std::memcpy(&data.packet.length, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Parse Aux Data (12 bytes)
    std::memcpy(&data.packet.aux_data, buffer + offset, sizeof(uint8_t[12]));
    offset += sizeof(uint8_t[12]);

    // Parse Timestamp (8 bytes)
    std::memcpy(&data.packet.timestamp_sec, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    std::memcpy(&data.packet.timestamp_nsec, buffer + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Parse Object Data Array (20 objects * 106 bytes each)
    for (size_t i = 0; i < MAX_OBJECTS; ++i) {
        ObjectData& obj = data.packet.objects[i];

        // Parse basic info
        std::memcpy(&obj.obj_id, buffer + offset, sizeof(int16_t));
        offset += sizeof(int16_t);
        std::memcpy(&obj.obj_type, buffer + offset, sizeof(int16_t));
        offset += sizeof(int16_t);

        // Parse position
        std::memcpy(&obj.pos_x, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.pos_y, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.pos_z, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse heading
        std::memcpy(&obj.heading, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse size
        std::memcpy(&obj.size_x, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.size_y, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.size_z, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse vehicle specific data
        std::memcpy(&obj.overhang, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.wheelbase, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.rear_overhang, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse velocity
        std::memcpy(&obj.velocity_x, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.velocity_y, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.velocity_z, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse acceleration
        std::memcpy(&obj.accel_x, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.accel_y, buffer + offset, sizeof(float));
        offset += sizeof(float);
        std::memcpy(&obj.accel_z, buffer + offset, sizeof(float));
        offset += sizeof(float);

        // Parse MGeo Link ID
        std::memcpy(&obj.link_id, buffer + offset, sizeof(char[38]));
        offset += sizeof(char[38]);
    }

    // Parse Tail (2 bytes) - 0x0D 0x0A
    std::memcpy(&data.packet.tail, buffer + offset, sizeof(uint8_t[2]));
    if (data.packet.tail[0] != 0x0D || data.packet.tail[1] != 0x0A) {
        std::cerr << "Invalid tail" << std::endl;
        return false;
    }

    return true;
}