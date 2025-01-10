#include "simulator/multi_ego_setting.hpp"

namespace MoraiCppUdp {

MultiEgoSetting::MultiEgoSetting(const std::string& ip, int port) : UDPSender(ip, port) {}

MultiEgoSettingPacket MultiEgoSetting::CreatePacket(int32_t num_of_ego, int32_t camera_index,
                                                    const std::vector<EgoState>& ego_states) {
    if (ego_states.size() > 20) {
        std::cerr << "Warning: Too many ego states provided (" << ego_states.size() << "). Only first 20 will be used."
                  << std::endl;
    }

    MultiEgoSettingPacket packet;

    // 헤더 설정
    packet.start_indicator = 0x23; // '#'
    std::strncpy(packet.header_name, "MultiEgoSetting", 15);
    packet.doller_indicator = 0x24; // '$'

    // 데이터 길이 설정 (4 + 4 + 32 * 20)
    packet.data_length = 648;

    // aux_data는 0으로 초기화
    std::memset(packet.aux_data, 0, sizeof(packet.aux_data));

    // 차량 개수와 카메라 인덱스 설정
    packet.num_of_ego = num_of_ego;
    packet.camera_index = camera_index;

    // ego_states 복사 (최대 20개까지만)
    packet.ego_states.clear();
    for (size_t i = 0; i < std::min(ego_states.size(), size_t(20)); ++i) {
        packet.ego_states.push_back(ego_states[i]);
    }

    // 20개까지 채우기
    while (packet.ego_states.size() < 20) {
        packet.ego_states.push_back(EgoState{});
    }

    // 테일 설정
    packet.tail01 = 0x0D;
    packet.tail02 = 0x0A;

    return packet;
}

bool MultiEgoSetting::SendMultiEgoSetting(const MultiEgoSettingPacket& packet) {
    // 패킷을 바이트 배열로 변환
    char buffer[683]; // 전체 패킷 크기
    char* ptr = buffer;

    // 헤더 복사
    *ptr++ = packet.start_indicator;
    std::memcpy(ptr, packet.header_name, 15);
    ptr += 15;
    *ptr++ = packet.doller_indicator;

    // 데이터 길이 복사
    std::memcpy(ptr, &packet.data_length, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // aux_data 복사
    std::memcpy(ptr, packet.aux_data, 12);
    ptr += 12;

    // num_of_ego와 camera_index 복사
    std::memcpy(ptr, &packet.num_of_ego, sizeof(int32_t));
    ptr += sizeof(int32_t);
    std::memcpy(ptr, &packet.camera_index, sizeof(int32_t));
    ptr += sizeof(int32_t);

    // ego_states 복사 (20개)
    for (const auto& ego : packet.ego_states) {
        std::memcpy(ptr, &ego, sizeof(EgoState));
        ptr += sizeof(EgoState);
    }

    // 테일 복사
    std::memcpy(ptr, &packet.tail01, sizeof(uint8_t));
    ptr += sizeof(uint8_t);
    std::memcpy(ptr, &packet.tail02, sizeof(uint8_t));
    ptr += sizeof(uint8_t);

    // 데이터 전송
    return Send(buffer, 683);
}

} // namespace MoraiCppUdp
