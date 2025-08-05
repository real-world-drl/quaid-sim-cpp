//
// Created by bohm on 12/18/22.
//
#include "servo_shield.h"

#include <utility>

ServoShield::ServoShield(mjModel* m, mjData *d,mjvCamera *cam, std::shared_ptr<MqttSettings> settings) : d(d), m(m), cam(cam), settings(settings) {
    if (settings->version == 1) {
        this->limits = this->limits_v1;
    } else {
        this->limits = this->limits_v2;
    }
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
  int position = Utils::map(
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

int16_t ServoShield::get_position(const int &servonum) {
  return (int16_t) positions[servonum];
}

void ServoShield::apply_matching_servo_limits(int const &servonum) {
    if (settings->matching_servo_limits == -1) {
        return;
    }

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

  if (diff > settings->matching_servo_limits) {
    positions[servonum] = positions[matching_servo] - settings->matching_servo_limits;
//      Serial.print("Diff more than MAX adjusting value to ");
//      Serial.println(positions[servonum]);
  } else if (diff < -settings->matching_servo_limits) {
    positions[servonum] = positions[matching_servo] + settings->matching_servo_limits;
//      Serial.print("Diff less than MAX adjusting value to ");
//      Serial.println(positions[servonum]);
  }
}

void ServoShield::move_servos(std::string readline, bool use_set_action) {
  std::vector<float> vect = Utils::parse_csv(std::move(readline));

  for (size_t servoNum = 0; servoNum < vect.size(); ++ servoNum) {
    if (use_set_action) {
      set_action(vect[servoNum], servoNum);
    } else {
      set_position(vect[servoNum], servoNum);
    }
    if (servoNum == 15) {
      break;
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
    float mapped_position = Utils::map(positions[servonum], limits[servonum].min, limits[servonum].max,
                                limits[servonum].control_min, limits[servonum].control_max);
    d->ctrl[mapped_servo] = mapped_position;
    /* * /
     std::cout << "Setting position of " << servonum << " mapped to " << servo_mapping[servonum] << " to "
              << mapped_position << " mapped from " << positions[servonum] << std::endl;
  / * */
    //reset_camera();
  }
}



void ServoShield::reset_body(int body_index, float theta) {
    quat_t quat{};
    Utils::eulerToQuaternion(theta, 0, 0, &quat);
    int position_index = body_index * 3;
    int attitude_index = body_index * 4;

    d->mocap_pos[position_index] = d->sensordata[4];
    d->mocap_pos[position_index + 1] = d->sensordata[5];

    d->mocap_quat[attitude_index] = quat.qr;
    d->mocap_quat[attitude_index + 1] = quat.qi;
    d->mocap_quat[attitude_index + 2] = quat.qj;
    d->mocap_quat[attitude_index + 3] = quat.qk;
}



void ServoShield::reset_marker(float theta) {
    quat_t quat{};
    Utils::eulerToQuaternion(theta, 0, 0, &quat);

    // frame reference marker
    d->mocap_pos[0] = d->sensordata[4];
    d->mocap_pos[1] = d->sensordata[5];

    d->mocap_quat[0] = d->sensordata[0];
    d->mocap_quat[1] = d->sensordata[1];
    d->mocap_quat[2] = d->sensordata[2];
    d->mocap_quat[3] = d->sensordata[3];

    // heading reference marker
    d->mocap_pos[3] = d->sensordata[4];
    d->mocap_pos[4] = d->sensordata[5];

    d->mocap_quat[4] = quat.qr;
    d->mocap_quat[5] = quat.qi;
    d->mocap_quat[6] = quat.qj;
    d->mocap_quat[7] = quat.qk;

}

void ServoShield::move_marker(std::string readline) {
  std::vector<float> pos_orient = Utils::parse_csv(std::move(readline));

  d->mocap_pos[3] = pos_orient[0];
  d->mocap_pos[4] = pos_orient[1];
  d->mocap_pos[5] = pos_orient[2];

  d->mocap_quat[4] = pos_orient[3];
  d->mocap_quat[5] = pos_orient[4];
  d->mocap_quat[6] = pos_orient[5];
  d->mocap_quat[7] = pos_orient[6];
}

void ServoShield::reset_camera() {
  cam->distance = 5;
  cam->azimuth = 45;
  cam->elevation = -20;
  cam->lookat[0] = d->sensordata[4];
  cam->lookat[1] = d->sensordata[5];
  cam->lookat[2] = d->sensordata[6];
}

void ServoShield::set_sensor_noise(std::string payload) {
  std::cout << "Setting sensor noise to " << payload << " This was deprecated in 3.14 and doesn't work anymore!!!\nYou need to provide your own noise" << std::endl;
  std::vector<float> vect = Utils::parse_csv(std::move(payload));

  for (uint i = 0; i < vect.size() && i < (uint) m->nsensor; ++i) {
    m->sensor_noise[i] = vect[i];
  }
}