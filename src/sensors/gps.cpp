#include "sensors/gps.hpp"
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>

GPS::GPS(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false)
{
    is_running_ = true;
    thread_gps_udp_receiver_ = std::thread(&GPS::ThreadGPSUdpReceiver, this);
}

GPS::~GPS()
{
    is_running_ = false;
    if (thread_gps_udp_receiver_.joinable())
    {
        thread_gps_udp_receiver_.join();
    }
    Close();
}

void GPS::ThreadGPSUdpReceiver()
{
    char packet_buffer[MAX_PACKET_SIZE];

    while (is_running_)
    {
        try 
        {
            size_t received_size = 0;
            if (!Receive(packet_buffer, MAX_PACKET_SIZE, received_size))
            {
                std::cerr << "Failed to receive GPS data" << std::endl;
                continue;
            }

            packet_buffer[received_size] = '\0';
            std::string nmea_string(packet_buffer);

            GPSData temp_data;
            if (ParseNMEAString(nmea_string, temp_data))
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (gps_callback_)
                {
                    gps_callback_(temp_data);
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "GPS position error: " << e.what() << std::endl;
            continue;
        }
    }
}

bool GPS::ParseNMEAString(const std::string& nmea_string, GPSData& data)
{
    // Split NMEA string into tokens
    std::vector<std::string> tokens;
    std::stringstream ss(nmea_string);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        tokens.push_back(token);
    }

    if (tokens.empty()) return false;

    // Check sentence type
    if (tokens[0] == "$GPRMC")
    {
        return ParseGPRMC(tokens, data);
    }
    else if (tokens[0] == "$GPGGA")
    {
        return ParseGPGGA(tokens, data);
    }

    return false;
}

bool GPS::ParseGPRMC(const std::vector<std::string>& tokens, GPSData& data)
{
    if (tokens.size() < 12) return false;

    data.sentence_type = "GPRMC";
    
    // Parse time
    if (!tokens[1].empty())
    {
        data.timestamp = std::stof(tokens[1]);
    }

    // Parse status
    data.is_valid = (tokens[2] == "A");

    // Parse latitude
    if (!tokens[3].empty() && !tokens[4].empty())
    {
        data.latitude = ConvertNMEAPositionToDegrees(tokens[3]);
        data.lat_dir = tokens[4][0];
        if (data.lat_dir == 'S') data.latitude = -data.latitude;
    }

    // Parse longitude
    if (!tokens[5].empty() && !tokens[6].empty())
    {
        data.longitude = ConvertNMEAPositionToDegrees(tokens[5]);
        data.lon_dir = tokens[6][0];
        if (data.lon_dir == 'W') data.longitude = -data.longitude;
    }

    return true;
}

bool GPS::ParseGPGGA(const std::vector<std::string>& tokens, GPSData& data)
{
    if (tokens.size() < 15) return false;

    data.sentence_type = "GPGGA";
    
    // Parse time
    if (!tokens[1].empty())
    {
        data.timestamp = std::stof(tokens[1]);
    }

    // Parse latitude
    if (!tokens[2].empty() && !tokens[3].empty())
    {
        data.latitude = ConvertNMEAPositionToDegrees(tokens[2]);
        data.lat_dir = tokens[3][0];
        if (data.lat_dir == 'S') data.latitude = -data.latitude;
    }

    // Parse longitude
    if (!tokens[4].empty() && !tokens[5].empty())
    {
        data.longitude = ConvertNMEAPositionToDegrees(tokens[4]);
        data.lon_dir = tokens[5][0];
        if (data.lon_dir == 'W') data.longitude = -data.longitude;
    }

    // Parse altitude
    if (!tokens[9].empty())
    {
        data.altitude = std::stod(tokens[9]);
    }

    data.is_valid = true;
    return true;
}

double GPS::ConvertNMEAPositionToDegrees(const std::string& nmea_pos)
{
    if (nmea_pos.empty()) return 0.0;

    double pos = std::stod(nmea_pos);
    int degrees = static_cast<int>(pos / 100);
    double minutes = pos - (degrees * 100);
    return degrees + (minutes / 60.0);
} 