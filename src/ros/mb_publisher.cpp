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

#include "ur_modern_driver/ros/mb_publisher.h"

inline void appendAnalog(std::vector<ur_msgs::Analog>& vec, double val, uint8_t pin)
{
  ur_msgs::Analog ana;
  ana.pin = pin;
  ana.state = val;
  vec.push_back(ana);
}

void MBPublisher::publishIOStates(ur_msgs::IOStates& io_msg, SharedMasterBoardData& data)
{
  appendAnalog(io_msg.analog_in_states, data.analog_input0, 0);
  appendAnalog(io_msg.analog_in_states, data.analog_input1, 1);
  appendAnalog(io_msg.analog_out_states, data.analog_output0, 0);
  appendAnalog(io_msg.analog_out_states, data.analog_output1, 1);

  io_pub_.publish(io_msg);
}

void MBPublisher::publishRobotStatus(industrial_msgs::RobotStatus& status, const SharedRobotModeData& data) const
{
  // using ROS time here as the 'timestamp' in RobotModeData is not time-synced
  // with ROS time. Publishing messages using the controller's time will
  // complicate correlating RobotStatus messages with other messages that do
  // use ROS time.
  status.header.stamp = ros::Time::now();

  // note that this is true as soon as the drives are powered,
  // even if the brakes are still closed
  // which is in slight contrast to the comments in the
  // message definition
  status.drives_powered.val = data.robot_power_on;

  status.e_stopped.val = data.emergency_stopped;

  // I found no way to reliably get information if the robot is moving
  // data.programm_running would be true when using this driver to move the robot
  // but it would also be true when another programm is running that might not
  // in fact contain any movement commands
  status.in_motion.val = industrial_msgs::TriState::UNKNOWN;

  // the error code, if any, is not transmitted by this protocol
  // it can and should be fetched seperately
  status.error_code = 0;

  // note that e-stop is handled by a seperate variable
  status.in_error.val = data.protective_stopped;

  status_pub_.publish(status);
}

void MBPublisher::publishRobotStatus(const RobotModeData_V1_X& data) const
{
  industrial_msgs::RobotStatus msg;

  if (data.robot_mode == robot_mode_V1_X::ROBOT_FREEDRIVE_MODE)
    msg.mode.val = industrial_msgs::RobotMode::MANUAL;
  else
    msg.mode.val = industrial_msgs::RobotMode::AUTO;

  // todo: verify that this correct, there is also ROBOT_READY_MODE
  msg.motion_possible.val = (data.robot_mode == robot_mode_V1_X::ROBOT_RUNNING_MODE) ? industrial_msgs::TriState::ON :
                                                                                       industrial_msgs::TriState::OFF;

  publishRobotStatus(msg, data);
}

void MBPublisher::publishRobotStatus(const RobotModeData_V3_0__1& data) const
{
  industrial_msgs::RobotStatus msg;

  msg.motion_possible.val =
      (data.robot_mode == robot_mode_V3_X::RUNNING) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;

  if (data.control_mode == robot_control_mode_V3_X::TEACH)
    msg.mode.val = industrial_msgs::RobotMode::MANUAL;
  else
    msg.mode.val = industrial_msgs::RobotMode::AUTO;

  publishRobotStatus(msg, data);
}

void MBPublisher::publishMasterboardData(ur_msgs::MasterboardDataMsg& msg, const SharedMasterBoardData& data) const
{
  // populate common fields (ie: those that exist in all versions of the protocol)
  msg.analog_input_range0 = data.analog_input_range0;
  msg.analog_input_range1 = data.analog_input_range1;
  msg.analog_input0 = data.analog_input0;
  msg.analog_input1 = data.analog_input1;
  msg.analog_output_domain0 = data.analog_output_domain0;
  msg.analog_output_domain1 = data.analog_output_domain1;
  msg.analog_output0 = data.analog_output0;
  msg.analog_output1 = data.analog_output1;
  msg.masterboard_temperature = data.master_board_temperature;
  msg.robot_voltage_48V = data.robot_voltage_48V;
  msg.robot_current = data.robot_current;
  msg.master_io_current = data.master_IO_current;

  masterboard_state_pub_.publish(msg);
}

void MBPublisher::publishMasterboardData(const MasterBoardData_V1_X& data) const
{
  using ros_msg_dig_inp_t = ur_msgs::MasterboardDataMsg::_digital_input_bits_type;
  using ros_msg_dig_outp_t = ur_msgs::MasterboardDataMsg::_digital_output_bits_type;

  ur_msgs::MasterboardDataMsg msg;

  // according to UR documentation, digital_*_bits fields are 4 byte integers,
  // so this is valid (MasterBoardData ROS msg fields are sufficiently wide).
  // in V1.x, only 10 bits are used
  msg.digital_input_bits = static_cast<ros_msg_dig_inp_t>(data.digital_input_bits.to_ulong());
  msg.digital_output_bits = static_cast<ros_msg_dig_outp_t>(data.digital_output_bits.to_ulong());

  // delegate remainder to common handler
  publishMasterboardData(msg, data);
}

void MBPublisher::publishMasterboardData(const MasterBoardData_V3_0__1& data) const
{
  using ros_msg_dig_inp_t = ur_msgs::MasterboardDataMsg::_digital_input_bits_type;
  using ros_msg_dig_outp_t = ur_msgs::MasterboardDataMsg::_digital_output_bits_type;

  ur_msgs::MasterboardDataMsg msg;

  // according to UR documentation, digital_*_bits fields are 4 byte integers,
  // so this is valid (MasterBoardData ROS msg fields are sufficiently wide)
  // in V3.0, only 18 bits are used
  msg.digital_input_bits = static_cast<ros_msg_dig_inp_t>(data.digital_input_bits.to_ulong());
  msg.digital_output_bits = static_cast<ros_msg_dig_outp_t>(data.digital_output_bits.to_ulong());

  // delegate remainder to common handler
  publishMasterboardData(msg, data);
}

void MBPublisher::publishRobotModeData(ur_msgs::RobotModeDataMsg& msg, const SharedRobotModeData& data) const
{
  // populate common fields  (note: ROS msg structure only contains the common
  // fields right now, so we populate all fields here).
  msg.timestamp = data.timestamp;
  msg.is_robot_connected = data.physical_robot_connected;
  msg.is_real_robot_enabled = data.real_robot_enabled;
  msg.is_power_on_robot = data.robot_power_on;
  msg.is_emergency_stopped = data.emergency_stopped;
  msg.is_protective_stopped = data.protective_stopped;
  msg.is_program_running = data.program_running;
  msg.is_program_paused = data.program_paused;

  robot_mode_state_pub_.publish(msg);
}

void MBPublisher::publishRobotModeData(const RobotModeData_V1_X& data) const
{
  ur_msgs::RobotModeDataMsg msg;
  publishRobotModeData(msg, data);
}

void MBPublisher::publishRobotModeData(const RobotModeData_V3_0__1& data) const
{
  // TODO: add V3 specific fields to RobotModeDataMsg and populate here
  //       for now: delegate to common handler.
  ur_msgs::RobotModeDataMsg msg;
  publishRobotModeData(msg, data);
}

bool MBPublisher::consume(MasterBoardData_V1_X& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publishIOStates(io_msg, data);

  publishMasterboardData(data);

  return true;
}

bool MBPublisher::consume(MasterBoardData_V3_0__1& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publishIOStates(io_msg, data);

  publishMasterboardData(data);

  return true;
}

bool MBPublisher::consume(MasterBoardData_V3_2& data)
{
  consume(static_cast<MasterBoardData_V3_0__1&>(data));
  return true;
}

bool MBPublisher::consume(RobotModeData_V1_X& data)
{
  publishRobotStatus(data);
  publishRobotModeData(data);
  return true;
}

bool MBPublisher::consume(RobotModeData_V3_0__1& data)
{
  publishRobotStatus(data);
  publishRobotModeData(data);
  return true;
}

bool MBPublisher::consume(RobotModeData_V3_2& data)
{
  publishRobotStatus(data);
  // use 3.0 handler for now (as we don't parse version-specific fields
  // for this type of packet yet: would require changes to the ROS msg
  // structure)
  publishRobotModeData(static_cast<RobotModeData_V3_0__1&>(data));
  return true;
}
