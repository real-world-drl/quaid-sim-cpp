//
// Created by bohm on 12/18/22.
//

#ifndef QUAID_SIM_CPP_MQTT_CONTROLLER_H
#define QUAID_SIM_CPP_MQTT_CONTROLLER_H

#include <memory>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <mqtt/async_client.h>
#include <mqtt/client.h>
#include <mujoco/mujoco.h>

#include "servo_shield.h"
#include "utils.h"

#include "settings.h"

class MqttController {
public:
  void init(std::shared_ptr<MqttSettings> settings, mjModel* m, mjData* d, mjvCamera* cam);
  ~MqttController();

  bool connect();
  void disconnect() const;

  void reset();
  void sitDown();
  void standUp();

  bool readDataPacket(std::string readline);

  void startStreamingMocapData();
  void stopStreamingMocapData();

  void startStreamingObservations();
  void stopStreamingObservations();

  std::shared_ptr<ServoShield> servoShield;

protected:
  mjModel* m;
  mjData* d;

  std::shared_ptr<MqttSettings> settings;
  std::shared_ptr<mqtt::async_client> client;

  const std::string ACT_TOPIC_BASE { "quaid/act/r" };
  const std::string OBS_TOPIC_BASE { "quaid/obs/r" };
  const std::string OBS_MOCAP_TOPIC_BASE { "quaid/mocap/r" };

  const std::string CLIENT_ID_BASE {"QuaidSimulator"};
  const std::string PERSIST_DIR			{ "./persist" };

  std::string ACT_TOPIC;
  std::string OBS_TOPIC;
  std::string OBS_MOCAP_TOPIC;
  std::string CLIENT_ID;

  const char* LWT_PAYLOAD = "Last will and testament.";
  const int  QOS = 1;

  const std::chrono::seconds TIMEOUT = std::chrono::seconds(10);

  bool isStreamingMocap = false;
  void streamMocapData();

  bool isStreamingObservations = false;
  void streamObservations();

};

struct StateObservations {
    uint8_t header = 0x0A;

    int16_t time_delta;
    float distance;
    float yaw;
    float pitch;
    float roll;

    float voltage;
    float current;

    int16_t position_knee_front_left;
    int16_t position_thigh_front_left;
    int16_t position_knee_front_right;
    int16_t position_thigh_front_right;

    int16_t position_knee_back_left;
    int16_t position_thigh_back_left;
    int16_t position_knee_back_right;
    int16_t position_thigh_back_right;

} __attribute__((packed));

#endif //QUAID_SIM_CPP_MQTT_CONTROLLER_H
