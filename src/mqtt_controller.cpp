//
// Created by bohm on 12/18/22.
//

#include "mqtt_controller.h"

void MqttController::init(std::shared_ptr<MqttSettings> settings, mjData* d) {
  this->settings = settings;
  this->d = d;

  CLIENT_ID = (CLIENT_ID_BASE + this->settings->mqtt_queue_no);
  OBS_TOPIC = (OBS_TOPIC_BASE + this->settings->mqtt_queue_no);
  OBS_MOCAP_TOPIC = OBS_MOCAP_TOPIC_BASE;
  ACT_TOPIC = (ACT_TOPIC_BASE + this->settings->mqtt_queue_no);

  client = std::make_shared<mqtt::async_client>(this->settings->mqtt_server_ip, CLIENT_ID, PERSIST_DIR);
  servoShield = std::make_shared<ServoShield>(d);
}

bool MqttController::readDataPacket(std::string payload) {
  std::cout << "Received message:\n" << payload << std::endl;

  char dir = payload.c_str()[0];

  switch(dir) {
    case 'a':
      servoShield->move_servos(payload);
      break;
    case 'b':
      servoShield->move_servos(payload, false);
      break;
    case 'u':
      settings->streamingDelay = atoi(payload.substr(1).c_str());
      break;
    case 'i':
      settings->actingDelay = atoi(payload.substr(1).c_str());
      break;
    case 'f':
      servoShield->EXP_FILTER_C = atof(payload.substr(1).c_str());
      break;
    case 'r':
      servoShield->center_servos();
      break;
    case 'e':
      servoShield->stand_up();
      break;
    case 'x':
      startStreamingObservations();
      break;
    case 'y':
      stopStreamingObservations();
      break;
    case 'm':
      startStreamingMocapData();
      break;
    case 'n':
      stopStreamingMocapData();
      break;

//      case 'p':
//        showText = cmd.substring(1);
//        Lcd::print_short(showText);
//        break;
    default: break; // keep the current course
  }

  return true;
}

void MqttController::startStreamingObservations() {
  if (!isStreamingObservations) {
    isStreamingObservations = true;
    std::thread(&MqttController::streamObservations, this).detach();
    std::cout << "Started streaming observations to " << OBS_TOPIC << std::endl;
  }
}

void MqttController::stopStreamingObservations() {
  isStreamingObservations = false;
}

void MqttController::streamObservations() {
  auto lastSent = std::chrono::high_resolution_clock::now();
  while (isStreamingObservations) {
    auto now = std::chrono::high_resolution_clock::now();
    auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(lastSent - now).count();
    lastSent = now;

    char cmd[50] = "";
    sprintf(cmd, "S%ld", time_delta);
    client->publish(OBS_TOPIC, cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(settings->streamingDelay));
  }
}

void MqttController::startStreamingMocapData() {
  if (!isStreamingMocap) {
    isStreamingMocap = true;
    std::thread(&MqttController::streamMocapData, this).detach();
    std::cout << "Started streaming mocap observations to " << OBS_MOCAP_TOPIC << std::endl;
  }
}

void MqttController::stopStreamingMocapData() {
  isStreamingMocap = false;
}

void MqttController::streamMocapData() {
  while (isStreamingMocap) {
    char cmd[50] = "";
    sprintf(cmd, "S%f,%f,%f", d->sensordata[4], d->sensordata[5], d->sensordata[6]);
    client->publish(OBS_MOCAP_TOPIC, cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(settings->mocapStreamingDelay));
  }
}


bool MqttController::connect() {
//  min_callback mcb{this};
//  client->set_callback(mcb);
  client->set_message_callback([this](const mqtt::const_message_ptr& msg) {
      readDataPacket(msg->get_payload());
  });

  auto connOpts = mqtt::connect_options_builder()
      .clean_session()
      .will(mqtt::message(ACT_TOPIC, LWT_PAYLOAD, QOS))
      .finalize();

  std::cout << "\nConnecting..." << std::endl;
  mqtt::token_ptr conntok = client->connect(connOpts);
  std::cout << "Waiting for the connection..." << std::endl;
  conntok->wait();
  std::cout << "  ...OK" << std::endl;
  client->subscribe(ACT_TOPIC, QOS);

  std::cout << "Subscribed to " << ACT_TOPIC << std::endl;

  return true;
}

void MqttController::disconnect() const {
  std::cout << "\nDisconnecting..." << std::endl;
  client->disconnect()->wait();
  std::cout << "  ...OK" << std::endl;
}
