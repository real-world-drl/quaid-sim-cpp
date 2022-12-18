//
// Created by bohm on 12/18/22.
//

#include "quaid_controller.h"

MqttController QuaidController::mqtt;

void QuaidController::setup_camera(mjvCamera &cam) {
  cam.azimuth = 45;
  cam.elevation = -20;
  cam.distance = 5;
  cam.lookat[2] = 0;
}

void QuaidController::init_controller(const mjModel *m, mjData *d) {
  std::shared_ptr<MqttSettings> settings = std::make_shared<MqttSettings>();
  mqtt.init(settings, d);
  mqtt.connect();
}

void QuaidController::controller(const mjModel *m, mjData *d) {

}