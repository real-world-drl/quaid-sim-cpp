//
// Created by bohm on 12/18/22.
//

#ifndef QUAID_SIM_CPP_QUAID_CONTROLLER_H
#define QUAID_SIM_CPP_QUAID_CONTROLLER_H

#include <memory>
#include <string>
#include <mujoco/mujoco.h>
#include "mqtt_controller.h"

class QuaidController {
protected:
    std::shared_ptr<MqttController> mqtt;
    mjModel* m;
    mjData* d;
    std::shared_ptr<MqttSettings> settings;;

public:
    QuaidController(mjModel* m, mjData* d, std::shared_ptr<MqttSettings> settings, mjvCamera *cam, const std::string& mqtt_queue_no);
    ~QuaidController();

    void setup_camera(mjvCamera &cam);

//    void init_controller(mjModel* m, mjData* d, mjvCamera *cam, std::shared_ptr<MqttSettings> settings, const std::string& mqtt_queue_no);
//    void controller(const mjModel *m, mjData *d);


    void disconnect();

    bool isResetting();
    void finishResetting();

};

#endif //QUAID_SIM_CPP_QUAID_CONTROLLER_H
