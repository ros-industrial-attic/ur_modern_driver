/*
 * ur_hardware_control_loop.cpp
 *
 * Copyright 2015 Andrew Hundt, Thomas Timm Andersen
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

/* Based on original source from University of Colorado, Boulder. License copied below. */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************

 Author: Andrew Hundt, Dave Coleman
 */

#ifndef UR_ROS_CONTROL_UR_HARDWARE_INTERFACE_H
#define UR_ROS_CONTROL_UR_HARDWARE_INTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <array>
#include "do_output.h"
#include "ur_driver.h"

namespace ur {

// For simulation only - determines how fast a trajectory is followed
static const double POSITION_STEP_FACTOR = 1;
static const double VELOCITY_STEP_FACTOR = 1;

/// \brief Hardware interface for a robot
class UrHardwareInterfaceStandalone {
public:
	/**
	 * \brief Constructor
	 */
	UrHardwareInterfaceStandalone(
                         std::string host
                        ,boost::shared_ptr<std::condition_variable> rt_msg_cond_
	                    ,boost::shared_ptr<std::condition_variable> msg_cond_
                        ,boost::shared_ptr<UrDriver> robot
                        ,bool run_automatically = true
                        );
                        
	UrHardwareInterfaceStandalone(
                        std::string host
                        );

	/// \brief Initialize the hardware interface
	void init();

	/// \brief Read the state from the robot hardware.
	void read();

	/// \brief write the command to the robot hardware.
	void write();
    
    void write_position_command(std::vector<double>){
    }

	void setMaxVelChange(double inp);

	bool canSwitch() const;
	void doSwitch();

    void run(){
       while (velocity_interface_running_ || position_interface_running_) {
            ControlLoop();
        }
    }
    
    void run_one(){
      ControlLoop();
    }
    
    ~UrHardwareInterfaceStandalone(){
        std::unique_lock<std::mutex> locker(usr_access_lock);
        if(control_thread_){
          control_thread_->join();
          control_thread_.reset();
        }
    
    }
    
    void read_joint_position(std::vector<double>& vec){
      getWithLock(vec, joint_position_);
    }
    
    void read_joint_velocity(std::vector<double>& vec){
      getWithLock(vec, joint_velocity_);
    }
    
    void read_joint_effort(std::vector<double>& vec){
      getWithLock(vec, joint_effort_);
    }
    
    void read_tip_force(std::array<double,3> & arr){
      getWithLock(arr, robot_force_);
    }
    
    void read_tip_torque(std::array<double,3> & arr){
      getWithLock(arr, robot_torque_);
    }
    
    
    void write_joint_position(const std::vector<double>& vec){
      setWithLock(vec, joint_position_);
    }
    
    void write_joint_velocity(const std::vector<double>& vec){
      setWithLock(vec, joint_velocity_);
    }
    
    void write_joint_effort(const std::vector<double>& vec){
      setWithLock(vec, joint_effort_);
    }

    std::size_t num_joints(){return num_joints_;}
private:

	// Interfaces
	bool velocity_interface_running_;
	bool position_interface_running_;
    
	std::vector<std::string> joint_names_;
	std::vector<double> joint_position_;
	std::vector<double> joint_velocity_;
	std::vector<double> joint_effort_;
	std::vector<double> joint_position_command_;
	std::vector<double> joint_velocity_command_;
	std::vector<double> prev_joint_velocity_command_;
		std::size_t num_joints_;
	std::array<double,3> robot_force_ = { {0., 0., 0.} };
	std::array<double,3> robot_torque_ = { {0., 0., 0.} };

	double max_vel_change_;

	// Robot API
	boost::shared_ptr<std::condition_variable> rt_msg_cond_;
	boost::shared_ptr<std::condition_variable> msg_cond_;
	boost::shared_ptr<UrDriver> robot_;
    
    std::mutex usr_access_lock;
	boost::scoped_ptr<std::thread> control_thread_;
    
    void ControlLoop(){
		if (velocity_interface_running_ || position_interface_running_) {
            std::mutex dummy_lock;
			std::unique_lock<std::mutex> locker(dummy_lock);
			while (!robot_->rt_interface_->robot_state_->getControllerUpdated()) {
				rt_msg_cond_->wait(locker);
			}
            // clock_gettime(CLOCK_MONOTONIC, &current_time);
            // elapsed_time = ros::Duration(
            //         current_time.tv_sec - last_time.tv_sec
            //                 + (current_time.tv_nsec - last_time.tv_nsec)
            //                         / BILLION);
            // last_time = current_time;
			// Input
			this->read();
			robot_->rt_interface_->robot_state_->setControllerUpdated();
			// Control
            // controller_manager_->update(
            //         ros::Time(current_time.tv_sec, current_time.tv_nsec),
            //         elapsed_time);

			// Output
			this->write();
		}
    }
    
    template<typename T>
    void setWithLock(const T& newVal, T& current){
			std::unique_lock<std::mutex> locker(usr_access_lock);
            current = newVal;
    }
    
    
    template<typename T>
    void getWithLock(T& newVal,const T& current){
			std::unique_lock<std::mutex> locker(usr_access_lock);
            newVal = current;
    }
};
// class

}// namespace

#endif
