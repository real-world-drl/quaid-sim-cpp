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

struct MqttSettings {
  std::string mqtt_queue_no = "10";
  std::string mqtt_server_ip = "192.168.86.202";
  int mocapStreamingDelay = 25;
  int streamingDelay = 25;
  int actingDelay = 25;
};

struct euler_t {
  float yaw, pitch, roll = 0;
};

const float RAD_TO_DEG = 57.295779513082321;

class MqttController {
public:
  void init(std::shared_ptr<MqttSettings> settings, mjData* d);

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
  mjData* d;

  std::shared_ptr<MqttSettings> settings;
  std::shared_ptr<mqtt::async_client> client;

  const std::string ACT_TOPIC_BASE { "quaid/act/r" };
  const std::string OBS_TOPIC_BASE { "quaid/obs/r" };
  const std::string OBS_MOCAP_TOPIC_BASE { "quaid/obs/mocap" };

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

  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees );

};

#endif //QUAID_SIM_CPP_MQTT_CONTROLLER_H
