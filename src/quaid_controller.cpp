//
// Created by bohm on 12/18/22.
//

#include "quaid_controller.h"

MqttController QuaidController::mqtt;

void QuaidController::setup_camera(mjvCamera &cam) {
//  cam.azimuth = 45;
//  cam.elevation = -20;
//  cam.distance = 5;
//  cam.lookat[2] = 0;
}

void QuaidController::init_controller(const mjModel *m, mjData *d, mjvCamera *cam, const std::string& config_path, const std::string& mqtt_queue_no) {
  std::shared_ptr<MqttSettings> settings = std::make_shared<MqttSettings>(config_path);

  if (mqtt_queue_no != "-1") {
    settings->mqtt_queue_no = mqtt_queue_no;
  }

  mqtt.init(settings, d, cam);
  mqtt.connect();

  m->sensor_noise[0] = settings->rotation_noise;
  m->sensor_noise[1] = settings->position_noise;
}

void QuaidController::controller(const mjModel *m, mjData *d) {

}
