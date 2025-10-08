//
// Created by bohm on 12/18/22.
//

#include "quaid_controller.h"

QuaidController::QuaidController(mjModel *m, mjData *d, std::shared_ptr<MqttSettings> settings, mjvCamera *cam,
                                 const std::string &mqtt_queue_no) : settings(settings) {
    if (mqtt_queue_no != "-1") {
        settings->mqtt_queue_no = mqtt_queue_no;
    }

    // initModel();

    this->mqtt = std::make_shared<MqttController>(settings, m, d, cam);
    this->mqtt->connect();

    m->sensor_noise[0] = settings->rotation_noise;
    m->sensor_noise[1] = settings->position_noise;
}

QuaidController::~QuaidController() {
    disconnect();
}

void QuaidController::setup_camera(mjvCamera &cam) {
//  cam.azimuth = 45;
//  cam.elevation = -20;
//  cam.distance = 5;
//  cam.lookat[2] = 0;
}

bool QuaidController::isResetting() {
    return mqtt->isResetting;
}

void QuaidController::finishResetting() {
    mqtt->isResetting = false;
}

void QuaidController::initModel() {

}

/*
void QuaidController::controller(const mjModel *m, mjData *d) {

}*/

void QuaidController::disconnect() {
    mqtt->stopStreamingObservations();
    mqtt->stopStreamingMocapData();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mqtt->disconnect();
}