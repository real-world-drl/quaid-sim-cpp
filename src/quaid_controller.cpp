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

void QuaidController::init_controller(mjModel *m, mjData *d, mjvCamera *cam, std::shared_ptr<MqttSettings> settings, const std::string& mqtt_queue_no) {
  if (mqtt_queue_no != "-1") {
    settings->mqtt_queue_no = mqtt_queue_no;
  }

  mqtt.init(settings, m, d, cam);
  mqtt.connect();

  m->sensor_noise[0] = settings->rotation_noise;
  m->sensor_noise[1] = settings->position_noise;
}

void QuaidController::controller(const mjModel *m, mjData *d) {

}

void QuaidController::disconnect() {
    mqtt.stopStreamingObservations();
    mqtt.stopStreamingMocapData();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mqtt.disconnect();
}