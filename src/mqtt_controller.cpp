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

  std::cout << "MqttController on thread " << std::this_thread::get_id() << std::endl;
}

bool MqttController::readDataPacket(std::string payload) {
  // std::cout << "Received message:\n" << payload << std::endl;

  char dir = payload.c_str()[0];

  switch(dir) {
    case 'a':
//      std::thread(&ServoShield::move_servos, servoShield, payload, true).detach();
       servoShield->move_servos(payload);
      break;
    case 'b':
//      std::thread(&ServoShield::move_servos, servoShield, payload, false).detach();
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
      std::thread(&ServoShield::center_servos, servoShield).detach();
      // servoShield->center_servos();
      break;
    case 'e':
      std::thread(&ServoShield::stand_up, servoShield).detach();
      // servoShield->stand_up();
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
    auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSent).count();
    lastSent = now;

    char cmd[150] = "";
    sprintf(cmd, "S%ld,%ld,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            time_delta,
            // std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count(), // distance
            0L,
            0.0, //Bno::euler.yaw,
            0.0, //Bno::euler.pitch,
            0.0, //Bno::euler.roll,
            0, // voltage
            0, // current

            servoShield->get_position(0),
            servoShield->get_position(1),

            servoShield->get_position(4),
            servoShield->get_position(5),

            servoShield->get_position(8),
            servoShield->get_position(9),

            servoShield->get_position(12),
            servoShield->get_position(13)

    );

    try {
      mqtt::message_ptr pubmsg = mqtt::make_message(OBS_TOPIC, cmd);
      pubmsg->set_qos(QOS);
      client->publish(OBS_TOPIC, cmd);
    } catch (const mqtt::exception& exc) {
      std::cerr << "Error streaming observations: " << exc.what() << " ["
           << exc.get_reason_code() << "]" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(settings->streamingDelay));
  }
  std::cout << "Observations streaming stopped" << std::endl;
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
  euler_t ypr{};
  while (isStreamingMocap) {
    char cmd[150] = "";
    quaternionToEuler(d->sensordata[0], d->sensordata[1], d->sensordata[2], d->sensordata[3], &ypr, false);
    sprintf(cmd, "S1,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            // in Mocap Y is up so we need to map x, y to x, z (i.e. index 4, 6, 5)
            // it seems that (actual) y is forward so the order is y, z, x
            d->sensordata[5] * 10, d->sensordata[6] * 10, d->sensordata[4] * 10,
            ypr.yaw, ypr.pitch, ypr.roll
            );
    try {
      client->publish(OBS_MOCAP_TOPIC, cmd);
    } catch (const mqtt::exception& exc) {
      std::cerr << "Error streaming mocap: " << exc.what() << " ["
                << exc.get_reason_code() << "]" << std::endl;
    }

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

void MqttController::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees ) {

  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  // pitch and roll are swapped
  ypr->roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}
