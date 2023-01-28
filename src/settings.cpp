//
// Created by bohm on 25/01/23.
//
#include <fstream>
#include "settings.h"

MqttSettings::MqttSettings(const std::string &path) {
  parse(path);
}

void MqttSettings::parse(const std::string &path) {
  std::ifstream f(path.c_str());
  if (!f.good()) {
    throw std::runtime_error("Config file " + path + " does not exist!");
  }

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

  if (config["sensors"]["position_noise"]) {
    position_noise = config["sensors"]["position_noise"].as<float>();
  }
  if (config["sensors"]["rotation_noise"]) {
    rotation_noise = config["sensors"]["rotation_noise"].as<float>();
  }
}