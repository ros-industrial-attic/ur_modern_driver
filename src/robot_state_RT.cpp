/*
 * robotStateRT.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/robot_state_RT.h"

RobotStateRT::RobotStateRT(std::condition_variable& msg_cond):
  version_(0.0),
  time_(0.0),
  q_target_(6, 0.0),
  qd_target_(6, 0.0),
  qdd_target_(6, 0.0),
  i_target_(6, 0.0),
  m_target_(6, 0.0),
  q_actual_(6, 0.0),
  qd_actual_(6, 0.0),
  i_actual_(6, 0.0),
  i_control_(6, 0.0),
  tool_vector_actual_(6, 0.0),
  tcp_speed_actual_(6, 0.0),
  tcp_force_(6, 0.0),
  tool_vector_target_(6, 0.0),
  tcp_speed_target_(6, 0.0),
  digital_input_bits_(64, false),
  motor_temperatures_(6, 0.0),
  controller_timer_(0.0),
  robot_mode_(0.0),
  joint_modes_(6, 0.0),
  safety_mode_(0.0),
  tool_accelerometer_values_(3, 0.0),
  speed_scaling_(0.0),
  linear_momentum_norm_(0.0),
  v_main_(0.0),
  v_robot_(0.0),
  i_robot_(0.0),
  v_actual_(6, 0.0),
  data_published_(false),
  controller_updated_(false),
  pMsg_cond_(&msg_con)
{

}

RobotStateRT::~RobotStateRT() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	pMsg_cond_->notify_all();
}

void RobotStateRT::setDataPublished() {
	data_published_ = false;
}
bool RobotStateRT::getDataPublished() {
	return data_published_;
}

void RobotStateRT::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated() {
	return controller_updated_;
}

inline double RobotStateRT::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}

std::vector<double> RobotStateRT::unpackVector(uint8_t * buf, unsigned int &offset,
    int nr_of_vals) {
	std::vector<double> ret;
  ret.reserve(nr_of_vals);
	for (int i = 0; i < nr_of_vals; i++) {
    ret.push_back( unpackDouble( buf, offset));
  }
	return ret;
}

inline double RobotStateRT::unpackDouble(uint8_t * buf, unsigned int &offset) {
  uint64_t unpack_to;
  memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
  offset += sizeof(double);
  return ntohd(unpack_to);
}

std::vector<bool> RobotStateRT::unpackDigitalInputBits(int64_t data) {
	std::vector<bool> ret;
	for (int i = 0; i < 64; i++) {
		ret.push_back((data & (1 << i)) >> i);
	}
	return ret;
}

void RobotStateRT::setVersion(double ver) {
  version_.store(ver);
}

double RobotStateRT::getVersion() {
  return version_.load();
}
double RobotStateRT::getTime() {
  return time_.load();
}
std::vector<double> RobotStateRT::getQTarget() {
  std::unique_lock lock(val_lock_);
  return  q_target_;
}
std::vector<double> RobotStateRT::getQdTarget() {
  std::unique_lock lock(val_lock_);
  return  qd_target_;
}
std::vector<double> RobotStateRT::getQddTarget() {
  std::unique_lock lock(val_lock_);
  return qdd_target_;
}
std::vector<double> RobotStateRT::getITarget() {
  std::unique_lock lock(val_lock_);
  return   i_target_;
}
std::vector<double> RobotStateRT::getMTarget() {
  std::unique_lock lock(val_lock_);
  return   m_target_;
}
std::vector<double> RobotStateRT::getQActual() {
  std::unique_lock lock(val_lock_);
  return  q_actual_;
}
std::vector<double> RobotStateRT::getQdActual() {
  std::unique_lock lock(val_lock_);
  return qd_actual_;
}
std::vector<double> RobotStateRT::getIActual() {
  std::unique_lock lock(val_lock_);
  return  i_actual_;
}
std::vector<double> RobotStateRT::getIControl() {
  std::unique_lock lock(val_lock_);
  return  i_control_;
}
std::vector<double> RobotStateRT::getToolVectorActual() {
  std::unique_lock lock(val_lock_);
  return  tool_vector_actual_;
}
std::vector<double> RobotStateRT::getTcpSpeedActual() {
  std::unique_lock lock(val_lock_);
  return tcp_speed_actual_;
}
std::vector<double> RobotStateRT::getTcpForce() {
  std::unique_lock lock(val_lock_);
  return  tcp_force_;
}
std::vector<double> RobotStateRT::getToolVectorTarget() {
  std::unique_lock lock(val_lock_);
  return  tool_vector_target_;
}
std::vector<double> RobotStateRT::getTcpSpeedTarget() {
  std::unique_lock lock(val_lock_);
  return  tcp_speed_target_;
}
std::vector<bool> RobotStateRT::getDigitalInputBits() {
  std::unique_lock lock(val_lock_);
  return  digital_input_bits_;
}
std::vector<double> RobotStateRT::getMotorTemperatures() {
  std::unique_lock lock(val_lock_);
  return  motor_temperatures_;
}
double RobotStateRT::getControllerTimer() {
  return controller_timer_.load();
}
double RobotStateRT::getRobotMode() {
  return robot_mode_.load();
}
std::vector<double> RobotStateRT::getJointModes() {
  std::unique_lock lock(val_lock_);
  return  joint_modes_;
}
double RobotStateRT::getSafety_mode() {
  return safety_mode_.load();
}
std::vector<double> RobotStateRT::getToolAccelerometerValues() {
  std::unique_lock lock(val_lock_);
  return tool_accelerometer_values_;
}
double RobotStateRT::getSpeedScaling() {
  return speed_scaling_.load();
}
double RobotStateRT::getLinearMomentumNorm() {
  return linear_momentum_norm_.load();
}
double RobotStateRT::getVMain() {
  return v_main_.load();
}
double RobotStateRT::getVRobot() {
  return v_robot_.load();
}
double RobotStateRT::getIRobot() {
  return i_robot_.load();
}
std::vector<double> RobotStateRT::getVActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = v_actual_;
	val_lock_.unlock();
	return ret;
}
void RobotStateRT::unpack(uint8_t * buf) {
	int64_t digital_input_bits;
  unsigned offset = 0;
  {// scope of the lock
    std::unique_lock lock(val_lock_);
    int len;

    memcpy(&len, &buf[offset], sizeof(len));
    offset += sizeof(len);
    len = ntohl(len);

    //Check the correct message length is received
    bool len_good = true;
    if (version_ >= 1.6 && version_ < 1.7) { //v1.6
      if (len != 756)
        len_good = false;
    } else if (version_ >= 1.7 && version_ < 1.8) { //v1.7
      if (len != 764)
        len_good = false;
    } else if (version_ >= 1.8 && version_ < 1.9) { //v1.8
      if (len != 812)
        len_good = false;
    } else if (version_ >= 3.0 && version_ < 3.2) { //v3.0 & v3.1
      if (len != 1044)
        len_good = false;
    } else if (version_ >= 3.2 && version_ < 3.3) { //v3.2
      if (len != 1060)
        len_good = false;
    }

    if (!len_good) {
      printf("Wrong length of message on RT interface: %i\n", len);
      return;
    }

    time_ = unpackDouble(buf, offset);
    q_target_ = unpackVector(buf, offset, 6);
    qd_target_ = unpackVector(buf, offset, 6);
    qdd_target_ = unpackVector(buf, offset, 6);
    i_target_ = unpackVector(buf, offset, 6);
    m_target_ = unpackVector(buf, offset, 6);
    q_actual_ = unpackVector(buf, offset, 6);
    qd_actual_ = unpackVector(buf, offset, 6);
    i_actual_ = unpackVector(buf, offset, 6);
    if (version_ <= 1.9) {
      if (version_ > 1.6){
        tool_accelerometer_values_ = unpackVector(buf, offset, 3);
        offset += sizeof(double) * (15); // TODO: double check that this offset is correct
      }
      else{
        offset += sizeof(double) * (3+15); // TODO: double check that this offset is correct
      }
      tcp_force_ = unpackVector(buf, offset, 6);
      tool_vector_actual_ = unpackVector(buf, offset, 6);
      tcp_speed_actual_ = unpackVector(buf, offset, 6);
    } else {
      i_control_ = unpackVector(buf, offset, 6);
      tool_vector_actual_ = unpackVector(buf, offset, 6);
      tcp_speed_actual_ = unpackVector(buf, offset, 6);
      tcp_force_ = unpackVector(buf, offset, 6);
      tool_vector_target_ = unpackVector(buf, offset, 6);
      tcp_speed_target_ = unpackVector(buf, offset, 6);
    }

    memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
    digital_input_bits_ = unpackDigitalInputBits(be64toh(digital_input_bits));
    offset += sizeof(double);
    motor_temperatures_ = unpackVector(buf, offset, 6);
    controller_timer_   = unpackDouble(buf, offset);
    if (version_ > 1.6) {
      offset += sizeof(double); // TODO: double check that this offset is correct
      robot_mode_ = unpackDouble(buf, offset);
      if (version_ > 1.7) {
        joint_modes_ = unpackVector(buf, offset, 6);
      }
      else{
        // TODO: double check that this offset is correct
        offset += sizeof(double) * 6;
      }
    }
    if (version_ > 1.8) {
      safety_mode_ = unpackDouble(buf, offset);
      tool_accelerometer_values_ = unpackVector(buf, offset, 3);
      speed_scaling_ = unpackDouble(buf, offset);
      linear_momentum_norm_ = unpackDouble(buf, offset);
      v_main_ = unpackDouble(buf, offset);
      v_robot_ = unpackDouble(buf, offset);
      i_robot_ = unpackDouble(buf, offset);
      v_actual_ = unpackVector(buf, offset, 6);
    }
    // lock goes out of scope here
  }
	controller_updated_ = true;
	data_published_ = true;
	pMsg_cond_->notify_all();

}

