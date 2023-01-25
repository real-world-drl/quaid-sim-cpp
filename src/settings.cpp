//
// Created by bohm on 25/01/23.
//

#include "settings.h"

MqttSettings::MqttSettings(const std::string &path) {
  parse(path);
}

void MqttSettings::parse(const std::string &path) {
  YAML::Node config = YAML::LoadFile(path);

  if (config["mqtt"]["mqtt_queue_no"]) {
    mqtt_queue_no = config["mqtt"]["mqtt_queue_no"].as<std::string>();
  }

  if (config["mqtt"]["mqtt_server_ip"]) {
    mqtt_server_ip = config["mqtt"]["mqtt_server_ip"].as<std::string>();
  }

  if (config["sim"]["mocap_streaming_delay"]) {
    mocapStreamingDelay = config["sim"]["mocap_streaming_delay"].as<int>();
  }
}