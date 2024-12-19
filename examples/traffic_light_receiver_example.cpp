#include "sensors/traffic_light.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

void PrintUsage() {
    std::cout << "Usage: traffic_light_receiver <ip_address> <port>" << std::endl;
    std::cout << "Example: traffic_light_receiver 127.0.0.1 7777" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        PrintUsage();
        return 1;
    }

    std::string ip_address = argv[1];
    uint16_t port = static_cast<uint16_t>(std::stoi(argv[2]));

    std::cout << "Connecting to " << ip_address << ":" << port << std::endl;
    
    TrafficLight traffic_light(ip_address, port);
    TrafficLight::TrafficLightData data;

    while (true)
    {
        if (traffic_light.GetTrafficLightState(data))
        {
            std::cout << "Traffic Light ID: " << data.traffic_light_index << std::endl;
            std::string str_type = "";
            if(data.traffic_light_type == 0)
                str_type = "RYG";
            else if(data.traffic_light_type == 1)
                str_type = "RYGLeft";
            else if(data.traffic_light_type == 2)
                str_type = "RYGLeftG";
            else if(data.traffic_light_type == 100)
                str_type = "YYY";

            std::cout << "Type: " << data.traffic_light_type << " " << str_type << std::endl;

            std::string str_status = "";
            if(data.traffic_light_status == 1)
                str_status = "Red";
            else if(data.traffic_light_status == 4)
                str_status = "Yellow";
            else if(data.traffic_light_status == 16)
                str_status = "Green";
            else if(data.traffic_light_status == 32)
                str_status = "GreenLeft";
            else if(data.traffic_light_status == 48)
                str_status = "Green with GreenLeft";
            else if(data.traffic_light_status == 20)
                str_status = "Yellow with Green";
            else if(data.traffic_light_status == 36)
                str_status = "Yellow with GreenLeft";
            else if(data.traffic_light_status == 5)
                str_status = "Red with Yellow";
            else if(data.traffic_light_status == -1)
                str_status = "default";
            std::cout << "Status: " << data.traffic_light_status << " " << str_status << std::endl;
            std::cout << "------------------------" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
} 