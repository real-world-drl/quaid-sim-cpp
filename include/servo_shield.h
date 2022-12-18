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

struct Limit {
  Limit(int min, int max, int center, int up) : min(min), max(max), center(center), up(up){

  };

  int min, max, center, up;
  int range() {
    return max - min;
  }
};

class ServoShield {
public:
  explicit ServoShield(mjData* d);

  float EXP_FILTER_C = 0.5;

  void center_servos();
  void stand_up();

  void move_servos(std::string payload, bool use_set_action = true);

  void set_position_with_filter(int const &new_position_cmd, int const &servonum);

  void set_action(float const &new_position_cmd, int const &servonum);
  void set_position(int const &new_position_cmd, int const &servonum);

  static float map(float value, float start1, float stop1, float start2, float stop2);


protected:
  void apply_matching_servo_limits(int const &servonum);

  mjData* d;

  Limit limits[16] = {
      Limit(170, 320, 300, 320),
      Limit(170, 300, 300, 170),
      Limit(0, 0, 0, 0), // unused
      Limit(0, 0, 0, 0), // unused
      Limit(280, 390, 300, 322),
      Limit(300, 390, 300, 390),
      Limit(0, 0, 0, 0), // unused
      Limit(0, 0, 0, 0), // unused
      Limit(280, 390, 300, 322),
      Limit(300, 390, 300, 390),
      Limit(0, 0, 0, 0), // unused
      Limit(100, 500, 300, 300), // distance sensor up-down
      Limit(250, 320, 300, 320),
      Limit(170, 300, 300, 170),
      Limit(0, 0, 0, 0), // unused
      Limit(100, 500, 300, 300) // distance sensor left-right
  };

  int positions[16] = {
      limits[0].center, limits[1].center, limits[2].center, limits[3].center,
      limits[4].center, limits[5].center, limits[6].center, limits[7].center,
      limits[8].center, limits[9].center, limits[10].center, limits[11].center,
      limits[12].center, limits[13].center, limits[14].center, limits[15].center
  };

  // mapping from the no-pcb version to the pcb version
  // in no-pcb 0 & 1 and left back, 4 & 5 are right back,
  //           8 & 9 are right front and 12 & 13 are left front
  // in PCB version 0 & 1 are right front, 6 & 7 are left front,
  //                9 & 8 are left back and 15 & 14 are right back
  int servo_mapping[16] = {
      9, 8, 2, 3,
      15, 14, 4, 5,
      0, 1, 10, 11,
      6, 7, 12, 13
  };

  const int MAX_MATCHING_SERVO_DIFFERENCE = 45;
};

#endif //QUAID_SIM_CPP_SERVO_SHIELD_H
