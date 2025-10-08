//
// Created by bohm on 1/9/25.
//
#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

#include "quaid_controller.h"

#include "CLI11.hpp"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context


int main(int argc, const char** argv) {
    std::string model_file = "";
    std::string config_file = "../config/settings.yaml";
    std::string mqtt_queue_no = "-1";

    CLI::App app{"Quaid-SIM"};
    app.add_option("-m,--model", model_file,
                   "MuJoCo model file path (defaults based on version set in the settings file)");
    app.add_option("-c,--config", config_file, "Config file path (defaults to ../config/settings.yaml)");
    app.add_option("-q,--mqtt_queue_no", mqtt_queue_no, "MQTT queue no (defaults to the one in config file)");

    CLI11_PARSE(app, argc, argv);

    std::shared_ptr<MqttSettings> settings = std::make_shared<MqttSettings>(config_file);
    settings->setModelFile(model_file);

    // load and compile model
    std::cout << "Loading model: " << settings->model_file << std::endl;
    char error[1000] = "Could not load binary model";
    if (settings->model_file.find(".mjb") != std::string::npos) {
        m = mj_loadModel(settings->model_file.c_str(), 0);
    } else {
        m = mj_loadXML(settings->model_file.c_str(), 0, error, 1000);
    }
    if (!m) {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);

    std::shared_ptr<QuaidController> controller = std::make_shared<QuaidController>(m, d, settings, &cam, mqtt_queue_no);

    while (true) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        }

        if (controller->isResetting()) {
            mj_resetData(m, d);
            mj_forward(m, d); // it was in the sample reset method - but it causes issues ERROR: mju_makeFrame: xaxis of contact frame undefined
            controller->finishResetting();
        }

        // without this it doesn't train and any trained policies will perform poorly
        // I guess there is some delay from rendering in the full mode - 10ms seems to work
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 1;
}

void run_simulation() {

}