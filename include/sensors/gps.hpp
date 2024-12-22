#ifndef __GPS_HPP__
#define __GPS_HPP__

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../network/udp_receiver.hpp"

/**
 * @class GPS
 * @brief MORAI 시뮬레이터의 GPS 데이터를 수신하고 처리하는 클래스
 * @details UDP를 통해 GPS 센서의 NMEA 문자열을 수신하고 파싱합니다.
 *          NMEA0183 Protocol을 따르고, RMC/GGA sentence 두 가지 format으로 동시에 들어옵니다.
 */
class GPS : public UDPReceiver {
    const static size_t MAX_PACKET_SIZE = 1024;

public:
    /**
     * @struct GPSData
     * @brief GPS 센서 데이터를 저장하는 구조체
     * @details NMEA0183 프로토콜의 GPRMC와 GPGGA 문장에서 추출한 데이터를 저장합니다.
     */
    struct GPSData {
        std::string sentence_type; ///< GPRMC 또는 GPGGA. Format의 종류를 나타냅니다.

        // Time information
        float timestamp; ///< GPS 정보가 들어올 때의 time stamp.

        // Position information
        double latitude;  ///< 위도 (도 단위)
        char lat_dir;     ///< 위도 방향 (N: 북위, S: 남위)
        double longitude; ///< 경도 (도 단위)
        char lon_dir;     ///< 경도 방향 (E: 동경, W: 서경)
        double altitude;  ///< 고도 (미터 단위)

        // Status
        bool is_valid; ///< GPS 데이터 유효��� 여부

        /**
         * @brief GPSData 구조체의 기본 생성자
         * @details 모든 수치 데이터를 0으로, 유효성을 false로 초기화합니다.
         */
        GPSData() : timestamp(0.0), latitude(0.0), longitude(0.0), altitude(0.0), is_valid(false) {}
    };

    // 콜백 함수 타입 정의
    using GPSCallback = std::function<void(const GPSData&)>;

    /**
     * @brief GPS 클래스의 생성자
     * @param ip_address UDP 수신을 위한 IP 주소
     * @param port UDP 수신을 위한 포트 번호
     * @note 생성자에서 UDP 수신 스레드가 시작됩니다
     */
    GPS(const std::string& ip_address, uint16_t port);

    /**
     * @brief GPS 클래스의 소멸자
     * @note UDP 수신 스레드를 안전하게 종료합니다
     */
    ~GPS();

    /**
     * @brief 콜백 등록 함수
     * @param callback 콜백 함수
     */
    void RegisterCallback(GPSCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        gps_callback_ = callback;
    }

private:
    /**
     * @brief UDP 수신 스레드 함수
     * @note 이 함수는 별도의 스레드에서 실행되며, GPS 데이터를 지속적으로 수신합니다
     */
    void ThreadGPSUdpReceiver();
    bool ParseNMEAString(const std::string& nmea_string, GPSData& data);
    bool ParseGPRMC(const std::vector<std::string>& tokens, GPSData& data);
    bool ParseGPGGA(const std::vector<std::string>& tokens, GPSData& data);
    double ConvertNMEAPositionToDegrees(const std::string& nmea_pos);

    std::thread thread_gps_udp_receiver_;
    std::atomic<bool> is_running_;
    std::mutex callback_mutex_;
    GPSCallback gps_callback_{[](const GPSData& data) {
        std::cout << "Note: GPS data received but no callback is registered. "
                  << "Consider registering a callback using RegisterCallback()." << std::endl;
    }};
};

#endif // __GPS_HPP__