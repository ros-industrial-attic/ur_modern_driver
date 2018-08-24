#pragma once

#include "ros/ros.h"
#include "ur_modern_driver/RobotModeDataMsg.h"
// Do not forget to add ur_modern_driver to your CMakeLists.txt

// Note: topic_namespace needs a leading slash.
bool isProgramRunning(std::string topic_namespace = "")
{
  boost::shared_ptr<ur_modern_driver::RobotModeDataMsg const> sharedPtr;
  ur_modern_driver::RobotModeDataMsg robot_mode_state;
  sharedPtr  = ros::topic::waitForMessage<ur_modern_driver::RobotModeDataMsg>(topic_namespace + "/ur_driver/robot_mode_state", ros::Duration(2));
  if (sharedPtr == NULL)
  {
    ROS_ERROR("No message received from the robot. Is everything running? Is the namespace set correctly? Returning true.");
    return true;  // If the function is used as intended (as a check after sending a custom program) then returning true is safer.
  }
  else
  {
    robot_mode_state = *sharedPtr;
    return robot_mode_state.is_program_running;
  }
}

// Note: topic_namespace needs a leading slash.
bool waitForURProgram(std::string topic_namespace = "", ros::Duration timeout = ros::Duration(60.0))
{
  ROS_INFO("Waiting for UR program to finish. Only run this after sending custom URScripts and not the regular motion commands, or this call will not terminate before the timeout.");
  ros::Time t_now, t_start = ros::Time::now();
  while (isProgramRunning(topic_namespace))
  {
    ros::Duration(.05).sleep();
  }
  ROS_INFO("UR Program has terminated.");
  return true;
}
