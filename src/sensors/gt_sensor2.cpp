/**
 * @file gt_sensor2.cpp
 * @brief GT_Sensor2 class definition
 * @details This file contains the definition of the GT_Sensor2 class, which is used to receive and parse the GT_Sensor2 data from the Morai simulator.
 * 
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @date 2026-02-27
 */

 #include <cstring>
 #include <iostream>
 
 #include "sensors/gt_sensor2.hpp"
 
 using namespace MoraiCppUdp;
 
 GT_Sensor2::GT_Sensor2(const std::string& ip_address, uint16_t port)
     : UDPReceiver(ip_address, port), is_running_(false), is_data_received_(false) {
     is_running_ = true;
     gt_sensor2_data_.reserve(MAX_OBJECTS);
     thread_gt_sensor2_receiver_ = std::thread(&GT_Sensor2::ThreadGT_Sensor2Receiver, this);
 }
 
 GT_Sensor2::GT_Sensor2(const std::string& ip_address, uint16_t port, GT_Sensor2Callback callback)
     : UDPReceiver(ip_address, port), is_running_(false), is_data_received_(false), gt_sensor2_callback_(callback) {
     std::lock_guard<std::mutex> lock(callback_mutex_);
     gt_sensor2_callback_ = callback;
     is_running_ = true;
     gt_sensor2_data_.reserve(MAX_OBJECTS);
     thread_gt_sensor2_receiver_ = std::thread(&GT_Sensor2::ThreadGT_Sensor2Receiver, this);
 }
 
 void GT_Sensor2::ThreadGT_Sensor2Receiver() {
     char packet_buffer[PACKET_SIZE];
 
     while (is_running_) {
         try {
             size_t received_size = 0;
             if (!Receive(packet_buffer, PACKET_SIZE, received_size)) {
                 std::cerr << "Failed to receive GT_Sensor2 data" << std::endl;
                 continue;
             }
 
             GT_Sensor2Data temp_data{};
             if (ParseGT_Sensor2(packet_buffer, received_size, temp_data)) {
                 is_data_received_ = true;
                 std::lock_guard<std::mutex> lock(callback_mutex_);
                 if (gt_sensor2_callback_) {
                     gt_sensor2_callback_(temp_data);
                 }
             }
         } catch (const std::exception& e) {
             std::cerr << "GT_Sensor2 error: " << e.what() << std::endl;
             continue;
         }
     }
 }
 
 bool GT_Sensor2::ParseGT_Sensor2(const char* buffer, size_t size, GT_Sensor2Data& data) {
     if (buffer == nullptr || size == 0) {
         return false;
     }
 
     std::memset(&data, 0, sizeof(GT_Sensor2Data));
 
     // Parse strictly according to GT_Sensor2Data layout in gt_sensor2.hpp.
     size_t offset = 0;
     const size_t min_required_size = sizeof(Timestamp) + sizeof(int32_t);
     if (size < min_required_size) {
         return false;
     }
 
     std::memcpy(&data.timestamp.sec, buffer + offset, sizeof(int64_t));
     offset += sizeof(int64_t);
     std::memcpy(&data.timestamp.nsec, buffer + offset, sizeof(int32_t));
     offset += sizeof(int32_t);
     std::memcpy(&data.num_objects, buffer + offset, sizeof(int32_t));
     offset += sizeof(int32_t);
 
     if (data.num_objects < 0) {
         data.num_objects = 0;
     }
     if (data.num_objects > static_cast<int32_t>(MAX_OBJECTS)) {
         data.num_objects = static_cast<int32_t>(MAX_OBJECTS);
     }
 
     const size_t objects_required_size = data.num_objects * sizeof(ObjectData);
     if (size < offset + objects_required_size) {
         return false;
     }
    //  offset += 32; // Why 32? I don't know. But it works.
 
     for (size_t i = 0; i < data.num_objects; ++i) {
         const size_t object_base = offset + (i * sizeof(ObjectData));
         ObjectData obj{};
         size_t obj_offset = object_base;
 
         std::memcpy(&obj.entity_id, buffer + obj_offset, sizeof(obj.entity_id));
         obj_offset += sizeof(obj.entity_id);
         std::memcpy(&obj.obj_type, buffer + obj_offset, sizeof(obj.obj_type));
         obj_offset += sizeof(obj.obj_type);
         std::memcpy(&obj.pos_x, buffer + obj_offset, sizeof(obj.pos_x));
         obj_offset += sizeof(obj.pos_x);
         std::memcpy(&obj.pos_y, buffer + obj_offset, sizeof(obj.pos_y));
         obj_offset += sizeof(obj.pos_y);
         std::memcpy(&obj.pos_z, buffer + obj_offset, sizeof(obj.pos_z));
         obj_offset += sizeof(obj.pos_z);
         std::memcpy(&obj.roll, buffer + obj_offset, sizeof(obj.roll));
         obj_offset += sizeof(obj.roll);
         std::memcpy(&obj.pitch, buffer + obj_offset, sizeof(obj.pitch));
         obj_offset += sizeof(obj.pitch);
         std::memcpy(&obj.yaw, buffer + obj_offset, sizeof(obj.yaw));
         obj_offset += sizeof(obj.yaw);
         std::memcpy(&obj.length, buffer + obj_offset, sizeof(obj.length));
         obj_offset += sizeof(obj.length);
         std::memcpy(&obj.width, buffer + obj_offset, sizeof(obj.width));
         obj_offset += sizeof(obj.width);
         std::memcpy(&obj.height, buffer + obj_offset, sizeof(obj.height));
         obj_offset += sizeof(obj.height);
         std::memcpy(&obj.front, buffer + obj_offset, sizeof(obj.front));
         obj_offset += sizeof(obj.front);
         std::memcpy(&obj.rear, buffer + obj_offset, sizeof(obj.rear));
         obj_offset += sizeof(obj.rear);
         std::memcpy(&obj.wheel_base, buffer + obj_offset, sizeof(obj.wheel_base));
         obj_offset += sizeof(obj.wheel_base);
         std::memcpy(&obj.vel_x, buffer + obj_offset, sizeof(obj.vel_x));
         obj_offset += sizeof(obj.vel_x);
         std::memcpy(&obj.vel_y, buffer + obj_offset, sizeof(obj.vel_y));
         obj_offset += sizeof(obj.vel_y);
         std::memcpy(&obj.vel_z, buffer + obj_offset, sizeof(obj.vel_z));
         obj_offset += sizeof(obj.vel_z);
         std::memcpy(&obj.acc_x, buffer + obj_offset, sizeof(obj.acc_x));
         obj_offset += sizeof(obj.acc_x);
         std::memcpy(&obj.acc_y, buffer + obj_offset, sizeof(obj.acc_y));
         obj_offset += sizeof(obj.acc_y);
         std::memcpy(&obj.acc_z, buffer + obj_offset, sizeof(obj.acc_z));
 
         data.objects[i] = obj;
     }
 
     return true;
 }