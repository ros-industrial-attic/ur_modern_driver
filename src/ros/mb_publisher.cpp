#include "ur_modern_driver/ros/mb_publisher.h"

inline void appendAnalog(std::vector<ur_msgs::Analog>& vec, double val, uint8_t pin)
{
  ur_msgs::Analog ana;
  ana.pin = pin;
  ana.state = val;
  vec.push_back(ana);
}

void MBPublisher::publish(ur_msgs::IOStates& io_msg, SharedMasterBoardData& data)
{
  appendAnalog(io_msg.analog_in_states, data.analog_input0, 0);
  appendAnalog(io_msg.analog_in_states, data.analog_input1, 1);
  appendAnalog(io_msg.analog_out_states, data.analog_output0, 0);
  appendAnalog(io_msg.analog_out_states, data.analog_output1, 1);

  io_pub_.publish(io_msg);

  // --- Publishes the data in robot-specific format.
  ur_msgs::MasterboardDataMsg msg;
  msg.digital_input_bits = 0;   // TODO: Set this.
  msg.digital_output_bits = 0;  // TODO: Set this.
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

void MBPublisher::publishRobotStatus(industrial_msgs::RobotStatus& status, const SharedRobotModeData& data) const
{
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

  // --- Publishes the data in robot-specific format.
  ur_msgs::RobotModeDataMsg msg;
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

bool MBPublisher::consume(MasterBoardData_V1_X& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publish(io_msg, data);
  return true;
}
bool MBPublisher::consume(MasterBoardData_V3_0__1& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publish(io_msg, data);
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
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_0__1& data)
{
  publishRobotStatus(data);
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_2& data)
{
  publishRobotStatus(data);
  return true;
}
