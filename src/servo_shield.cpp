//
// Created by bohm on 12/18/22.
//
#include "servo_shield.h"

ServoShield::ServoShield(mjData *d) : d(d) {

}

void ServoShield::center_servos() {
  for (int run = 0; run < 15; ++run) {
    for (int servo = 0; servo < 15; ++servo) {
      set_position_with_filter(limits[servo].center, servo);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void ServoShield::stand_up() {
  std::cout << "Stand up on thread " << std::this_thread::get_id() << std::endl;
  for (int run = 0; run < 15; ++run) {
    for (int servo = 0; servo < 15; ++servo) {
      set_position_with_filter(limits[servo].up, servo);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
}

void ServoShield::set_action(float const &new_position_cmd, int const &servonum) {
  int position = map(
      new_position_cmd * 1000, -1000.0, 1000.0,
      limits[servonum].min, limits[servonum].max
  );
  set_position_with_filter(position, servonum);
}

void ServoShield::set_position(int const &new_position, int const &servonum) {
  if (new_position <= limits[servonum].min) {
    positions[servonum] = limits[servonum].min;
  } else if (new_position >= limits[servonum].max) {
    positions[servonum] = limits[servonum].max;
  } else {
    positions[servonum] = new_position;
  }

  apply_matching_servo_limits(servonum);

  write_to_servo(servonum);
}

int ServoShield::get_position(const int &servonum) {
  return positions[servonum];
}

void ServoShield::apply_matching_servo_limits(int const &servonum) {
  int matching_servo = -1;
  if (servonum == 0 || servonum == 4 || servonum == 8 || servonum == 12) {
    matching_servo = servonum + 1;
  } else if (servonum == 1 || servonum == 5 || servonum == 9 || servonum == 13) {
    matching_servo = servonum - 1;
  }

  if (matching_servo == -1) {
    return;
  }

  int diff = positions[matching_servo] - positions[servonum];
//  char msg[200] = "";
//    sprintf(msg, "Servo: %d, matching servo %d, servo value: %d, matching servo value: %d, diff: %d",
//            servonum, matching_servo, positions[servonum], positions[matching_servo], diff);
//  std::cout << msg << std::endl;

  if (diff > MAX_MATCHING_SERVO_DIFFERENCE) {
    positions[servonum] = positions[matching_servo] - MAX_MATCHING_SERVO_DIFFERENCE;
//      Serial.print("Diff more than MAX adjusting value to ");
//      Serial.println(positions[servonum]);
  } else if (diff < -MAX_MATCHING_SERVO_DIFFERENCE) {
    positions[servonum] = positions[matching_servo] + MAX_MATCHING_SERVO_DIFFERENCE;
//      Serial.print("Diff less than MAX adjusting value to ");
//      Serial.println(positions[servonum]);
  }
}

void ServoShield::move_servos(std::string readline, bool use_set_action) {
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
      return ;
    }
  }

  for (int servoNum = 0; servoNum < vect.size(); ++ servoNum) {
    if (use_set_action) {
      set_action(vect[servoNum], servoNum);
    } else {
      set_position(vect[servoNum], servoNum);
    }
  }
}

void ServoShield::set_position_with_filter(const int &new_position_cmd, const int &servonum) {
  int old_position = positions[servonum];
  int new_position = old_position * EXP_FILTER_C + new_position_cmd * (1.0 - EXP_FILTER_C);
  if (new_position <= limits[servonum].min) {
    positions[servonum] = limits[servonum].min;
  } else if (new_position >= limits[servonum].max) {
    positions[servonum] = limits[servonum].max;
  } else {
    positions[servonum] = new_position;
  }

//    char msg[100] = "";
//    sprintf(msg, "Setting servo %d mapped to %d to filtered value %d (original value %d)",
//            servonum, servo_mapping[servonum], positions[servonum], new_position_cmd);
//    std::cout << msg << std::endl;
  apply_matching_servo_limits(servonum);

  write_to_servo(servonum);
}

void ServoShield::write_to_servo(const int &servonum) {
  int mapped_servo = servo_mapping[servonum];
  if (mapped_servo < 100) {
    //  pwm.setPWM(servo_mapping[servonum], 0, positions[servonum] + offsets[servonum]);
    float mapped_position = map(positions[servonum], limits[servonum].min, limits[servonum].max,
                                limits[servonum].control_min, limits[servonum].control_max);
    d->ctrl[mapped_servo] = mapped_position;
    std::cout << "Setting position of " << servonum << " mapped to " << servo_mapping[servonum] << " to "
              << mapped_position << " mapped from " << positions[servonum] << std::endl;
  }
}

float ServoShield::map(float x, float in_min, float in_max, float out_min, float out_max) {
  const float dividend = out_max - out_min;
  const float divisor = in_max - in_min;
  const float delta = x - in_min;
  if(divisor == 0){
    // log_e("Invalid map input range, min == max");
    return -1; //AVR returns -1, SAM returns 0
    // start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
  }
  // return (delta * dividend + (divisor / 2)) / divisor + out_min;
  return out_min + (out_max - out_min) * ((x - in_min) / (in_max - in_min));
}