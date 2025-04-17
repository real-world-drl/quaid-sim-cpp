//
// Created by bohm on 12/18/22.
//

#ifndef QUAID_SIM_CPP_SERVO_SHIELD_H
#define QUAID_SIM_CPP_SERVO_SHIELD_H

#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <utility>
#include <thread>
#include <chrono>
#include <memory>
#include <mujoco/mujoco.h>
#include <vector>
#include <cmath>

#include "utils.h"
#include "settings.h"

struct Limit {
    Limit(int min, int max, int center, int up) : min(min), max(max), center(center), up(up){

    };

    Limit(int min, int max, int center, int up, float control_min, float control_max) :  min(min), max(max), center(center), up(up), control_min(control_min), control_max(control_max) {

    }

    int min, max, center, up;
    float control_min = -1, control_max = 1;
    int range() {
        return max - min;
    }
};

class ServoShield {
public:
    explicit ServoShield(mjModel* m, mjData* d, mjvCamera* cam, std::shared_ptr<MqttSettings> settings);

    float EXP_FILTER_C = 0.5;

    void center_servos();
    void stand_up();

    void move_servos(std::string payload, bool use_set_action = true);

    void set_position_with_filter(int const &new_position_cmd, int const &servonum);

    void set_action(float const &new_position_cmd, int const &servonum);
    void set_position(int const &new_position_cmd, int const &servonum);
    int get_position(const int &servonum);

    void reset_marker(float theta);
    void move_marker(std::string payload);

    void set_sensor_noise(std::string payload);

    void reset_camera();


protected:
    void apply_matching_servo_limits(int const &servonum);
    void write_to_servo(const int &servonum);

    mjData* d;
    mjModel* m;
    mjvCamera* cam;

    std::shared_ptr<MqttSettings> settings;

    std::vector<Limit> limits_v2 = {
            /* quaid-sim-v2 */
            //Back Left
            Limit(185, 430, 308, 430, -1.0, 1.0),
            Limit(308, 430, 369, 308, 0, 1), // the control min/max correspond to joint settings in the XML
            Limit(0, 0, 0, 0), // unused
            Limit(0, 0, 0, 0), // unused
            // back right
            Limit(185, 430, 308, 185, -1.0, 1.0),
            Limit(185, 308, 246, 308, -1, 0),
            Limit(0, 0, 0, 0), // unused
            Limit(0, 0, 0, 0), // unused
            // Front Right
            Limit(185, 430, 308, 185, -1.0, 1.0),
            Limit(185, 308, 246, 308, -1, 0),
            Limit(0, 0, 0, 0), // unused
            Limit(100, 500, 300, 300), // distance sensor up-down
            // Front Left
            Limit(185, 430, 308, 430, -1.0, 1.0),
            Limit(308, 430, 369, 308, 0, 1),
            Limit(0, 0, 0, 0), // unused
            Limit(100, 500, 300, 300) // distance sensor left-right
    };

    std::vector<Limit> limits_v1 = {
            /* quaid-sim-v1 */
            Limit(150, 320, 300, 320),
            Limit(150, 320, 300, 170, 0, 1), // the control min/max correspond to joint settings in the XML
            Limit(0, 0, 0, 0), // unused
            Limit(0, 0, 0, 0), // unused
            Limit(280, 390, 300, 322),
            Limit(280, 390, 300, 390, -1, 0),
            Limit(0, 0, 0, 0), // unused
            Limit(0, 0, 0, 0), // unused
            Limit(280, 390, 300, 322),
            Limit(280, 390, 300, 390, -1, 0),
            Limit(0, 0, 0, 0), // unused
            Limit(100, 500, 300, 300), // distance sensor up-down
            Limit(150, 320, 300, 320),
            Limit(150, 320, 300, 170, 0, 1),
            Limit(0, 0, 0, 0), // unused
            Limit(100, 500, 300, 300) // distance sensor left-right
    };

    std::vector<Limit> limits = limits_v2;

    int positions[16] = {
            limits[0].center, limits[1].center, limits[2].center, limits[3].center,
            limits[4].center, limits[5].center, limits[6].center, limits[7].center,
            limits[8].center, limits[9].center, limits[10].center, limits[11].center,
            limits[12].center, limits[13].center, limits[14].center, limits[15].center
    };

    /*
     * mapping from the no-pcb version to the pcb version
     * in no-pcb 0 & 1 and left back, 4 & 5 are right back,
     *           8 & 9 are right front and 12 & 13 are left front
     * in PCB version 0 & 1 are right front, 6 & 7 are left front,
     *                9 & 8 are left back and 15 & 14 are right back
     * in Simulation 0 & 2 are left front, 4 & 6 are right front
     *                8 & 10 are left back and 12 & 14 are right back
     *
     * The mappings follow the original no-pcb quaid
     *
     * */

    /* */
    int servo_mapping[16] = {
            8, 10, 100, 100,
            12, 14, 100, 100,
            4, 6, 100, 100,
            0, 2, 100, 100
    };
    /* */

    const int MAX_MATCHING_SERVO_DIFFERENCE = 45;

};

#endif //QUAID_SIM_CPP_SERVO_SHIELD_H
