//
// Created by bohm on 12/18/22.
//

#ifndef QUAID_SIM_CPP_QUAID_CONTROLLER_H
#define QUAID_SIM_CPP_QUAID_CONTROLLER_H

#include <memory>
#include <string>
#include <mujoco/mujoco.h>
#include "mqtt_controller.h"

namespace QuaidController {
  void setup_camera(mjvCamera &cam);

  void init_controller(mjModel* m, mjData* d, mjvCamera *cam, std::shared_ptr<MqttSettings> settings, const std::string& mqtt_queue_no);

  void controller(const mjModel *m, mjData *d);

  extern MqttController mqtt;

  void disconnect();

}

#endif //QUAID_SIM_CPP_QUAID_CONTROLLER_H
