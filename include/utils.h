//
// Created by bohm on 1/16/23.
//

#ifndef QUAID_SIM_CPP_UTILS_H
#define QUAID_SIM_CPP_UTILS_H

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>


struct euler_t {
    float yaw, pitch, roll = 0;
};


struct quat_t {
    float qr, qi, qj, qk = 0;
};

const float RAD_TO_DEG = 57.295779513082321;

class Utils {
public:
    static std::vector<float> parse_csv(std::string readline) {
      readline[0] = ' '; // remove the command code - use later for processing commands to the simulator

      std::vector<float> vect;

      std::stringstream ss(readline);

      while (ss.good()) {
        std::string substr;
        getline(ss, substr, ',');
        //std::cout << "Processing: `" << substr << "`" << std::endl;

        try {
          vect.push_back(stof(substr));
        } catch (const std::invalid_argument &ia) {
          std::cerr << "Couldn't parse number from " << substr << " entire readline: " << readline;
        }
      }

      return vect;
    }

    static float map(float x, float in_min, float in_max, float out_min, float out_max) {
      const float divisor = in_max - in_min;
      if(divisor == 0){
        // log_e("Invalid map input range, min == max");
        std::cout << "Invalid map input range, min == max " << in_max << " == " << in_min << std::endl;
        return in_min; //AVR returns -1, SAM returns 0
        // start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
      }
      return out_min + (out_max - out_min) * ((x - in_min) / (in_max - in_min));
    }

    static
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees ) {
      float sqr = qr * qr;
      float sqi = qi * qi;
      float sqj = qj * qj;
      float sqk = qk * qk;

      ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
      ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
      /*
        cv::Vec4f Q = {qr, qi, qj, qk};
        cv::Quatd q(Q);
        cv::Vec3f euler = q.inv().toEulerAngles(cv::QuatEnum::EulerAnglesType::EXT_YXZ);

        ypr->yaw = euler[0];
        ypr->pitch = euler[1];
        ypr->roll = euler[2];
      */
      if (degrees) {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
      }
    }

    static void eulerToQuaternion(float yaw, float pitch, float roll, quat_t *quat) {
      quat->qi = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
      quat->qj = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
      quat->qk = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
      quat->qr = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    }
};

#endif //QUAID_SIM_CPP_UTILS_H
