#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"

// Note: topic_namespace needs a leading slash.
bool isProgramRunning(std::string topic_namespace = "")
{
  boost::shared_ptr<std_msgs::Bool const> sharedPtr;
  std_msgs::Bool program_running;
  program_running.data = true;
  sharedPtr  = ros::topic::waitForMessage<std_msgs::Bool>(topic_namespace + "/ur_driver/program_running", ros::Duration(2));
  if (sharedPtr == NULL)
  {
    ROS_ERROR("No message received from the robot. Is everything running? Is the namespace set correctly?");
    return false;
  }
  else
  {
    program_running = *sharedPtr;
    return program_running.data;
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
