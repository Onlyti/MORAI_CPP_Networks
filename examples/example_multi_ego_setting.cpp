#include <atomic>
#include <iostream>
#include <thread>
#include <simulator/multi_ego_setting.hpp>

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <ip_address> <port>" << std::endl;
        return 1;
    }

    const std::string ip_address = argv[1];
    const int port = std::stoi(argv[2]);


    MoraiCppUdp::MultiEgoSetting multi_ego_setting(ip_address, port);

    std::vector<MoraiCppUdp::MultiEgoSetting::EgoState> ego_states;
    MoraiCppUdp::MultiEgoSetting::EgoState ego_state;
    ego_state.ego_index = 0;
    ego_state.g_pos_x = 0.0;
    ego_state.g_pos_y = 0.0;
    ego_state.g_pos_z = 0.0;
    ego_state.g_roll = 0.0;
    ego_state.g_pitch = 0.0;
    ego_state.g_yaw = 0.0;
    ego_state.speed = 100.0;
    ego_state.gear = 4;
    ego_state.ctrl_mode = 1;
    ego_states.push_back(ego_state);

    MoraiCppUdp::MultiEgoSetting::MultiEgoSettingPacket packet = multi_ego_setting.CreatePacket(2, 1, ego_states);

    std::cout << "Sending Multi Ego Setting Packet..." << std::endl;
    std::cout << "\t position: " << packet.ego_states[0].g_pos_x << ", " << packet.ego_states[0].g_pos_y << ", " << packet.ego_states[0].g_pos_z << std::endl;
    std::cout << "\t speed: " << packet.ego_states[0].speed << std::endl;
    std::cout << "\t gear: " << static_cast<int>(packet.ego_states[0].gear) << std::endl;
    std::cout << "\t ctrl_mode: " << static_cast<int>(packet.ego_states[0].ctrl_mode) << std::endl;

    multi_ego_setting.SendMultiEgoSetting(packet);
    return 0;
}