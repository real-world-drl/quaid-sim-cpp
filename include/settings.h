//
// Created by bohm on 25/01/23.
//

#ifndef QUAID_SIM_CPP_SETTINGS_H
#define QUAID_SIM_CPP_SETTINGS_H

#include <string>
#include <yaml-cpp/yaml.h>

class MqttSettings {
public:
    MqttSettings(const std::string &path);


    std::string mqtt_queue_no = "10";
    std::string mqtt_server_ip = "192.168.86.202";
    int mocapStreamingDelay = 25;
    int streamingDelay = 25;
    int actingDelay = 25;

    float position_noise, rotation_noise = 0.0;

    int matching_servo_limits = 45;
    int version = 2;

protected:
    void parse(const std::string &path);
};

#endif //QUAID_SIM_CPP_SETTINGS_H
