#include "sensors/gps.hpp"
#include <iostream>
#include <thread>
#include <iomanip>

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <ip_address> <port>" << std::endl;
    std::cout << "Example: " << program_name << " 127.0.0.1 7506" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        PrintUsage(argv[0]);
        return -1;
    }

    const std::string ip_address = argv[1];
    const int port = std::stoi(argv[2]);

    try
    {
        GPS gps(ip_address, port);
        std::cout << "UDP Server Info - IP: " << ip_address << ", Port: " << port << std::endl;

        while (true)
        {
            GPS::GPSData data;
            if (gps.GetGPSData(data))
            {
                std::cout << "===== GPS Data =====" << std::endl;
                std::cout << "\tSentence Type: " << data.sentence_type << std::endl;
                std::cout << "\tTimestamp: " << std::fixed << std::setprecision(6) << data.timestamp << std::resetiosflags(std::ios::fixed) << std::endl;
                std::cout << "\tLatitude: " << std::fixed << std::setprecision(6) << data.latitude << std::endl;
                std::cout << "\tLatitude Direction: " << data.lat_dir << std::endl;
                std::cout << "\tLongitude: " << std::fixed << std::setprecision(6) << data.longitude << std::endl;
                std::cout << "\tLongitude Direction: " << data.lon_dir << std::endl;
                std::cout << "\tAltitude: " << std::fixed << std::setprecision(6) << data.altitude << std::resetiosflags(std::ios::fixed) << std::endl;
                std::cout << "===================" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
} 