#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"

// Note: topic_namespace needs a leading slash.
bool waitForURProgram(std::string topic_namespace)
{
  ROS_INFO("Waiting for UR program to finish. Only run this for custom URScripts and not the regular motion commands, or this call will not terminate.");
  boost::shared_ptr<std_msgs::Bool const> sharedPtr;
  std_msgs::Bool program_running;
  program_running.data = true;
  while (program_running.data)
  {
    sharedPtr  = ros::topic::waitForMessage<std_msgs::Bool>(topic_namespace + "/ur_driver/program_running", ros::Duration(2));
    if (sharedPtr == NULL)
    {
        ROS_ERROR("No message received from the robot. Is everything running?");
        return false;
    }
    else
        program_running = *sharedPtr;
  }
  ROS_INFO("UR Program has terminated.");
  return true;
}
