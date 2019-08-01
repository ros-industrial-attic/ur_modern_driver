/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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

#pragma once

#include <industrial_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <ur_msgs/Analog.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/IOStates.h>
#include <ur_msgs/MasterboardDataMsg.h>
#include <ur_msgs/RobotModeDataMsg.h>

#include "ur_modern_driver/ur/consumer.h"

using namespace ros;

class MBPublisher : public URStatePacketConsumer
{
private:
  NodeHandle nh_;
  Publisher io_pub_;
  Publisher status_pub_;
  Publisher masterboard_state_pub_;
  Publisher robot_mode_state_pub_;

  template <size_t N>
  inline void appendDigital(std::vector<ur_msgs::Digital>& vec, std::bitset<N> bits)
  {
    for (size_t i = 0; i < N; i++)
    {
      ur_msgs::Digital digi;
      digi.pin = static_cast<uint8_t>(i);
      digi.state = bits.test(i);
      vec.push_back(digi);
    }
  }

  void publishIOStates(ur_msgs::IOStates& io_msg, SharedMasterBoardData& data);

  void publishRobotStatus(industrial_msgs::RobotStatus& status, const SharedRobotModeData& data) const;
  void publishRobotStatus(const RobotModeData_V1_X& data) const;
  void publishRobotStatus(const RobotModeData_V3_0__1& data) const;

  // publish 1-to-1 copy of MasterBoardData as a ROS message
  void publishMasterboardData(ur_msgs::MasterboardDataMsg& msg, const SharedMasterBoardData& data) const;
  void publishMasterboardData(const MasterBoardData_V1_X& data) const;
  void publishMasterboardData(const MasterBoardData_V3_0__1& data) const;

  // publish 1-to-1 copy of RobotModeData as a ROS message
  void publishRobotModeData(ur_msgs::RobotModeDataMsg& msg, const SharedRobotModeData& data) const;
  void publishRobotModeData(const RobotModeData_V1_X& data) const;
  void publishRobotModeData(const RobotModeData_V3_0__1& data) const;

public:
  MBPublisher()
    : io_pub_(nh_.advertise<ur_msgs::IOStates>("ur_driver/io_states", 1))
    , status_pub_(nh_.advertise<industrial_msgs::RobotStatus>("ur_driver/robot_status", 1))
    , masterboard_state_pub_(nh_.advertise<ur_msgs::MasterboardDataMsg>("ur_driver/masterboard_state", 1))
    , robot_mode_state_pub_(nh_.advertise<ur_msgs::RobotModeDataMsg>("ur_driver/robot_mode_state", 1))
  {
  }

  virtual bool consume(MasterBoardData_V1_X& data);
  virtual bool consume(MasterBoardData_V3_0__1& data);
  virtual bool consume(MasterBoardData_V3_2& data);

  virtual bool consume(RobotModeData_V1_X& data);
  virtual bool consume(RobotModeData_V3_0__1& data);
  virtual bool consume(RobotModeData_V3_2& data);

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }
};
