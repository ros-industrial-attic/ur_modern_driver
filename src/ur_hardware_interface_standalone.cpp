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

 Authors: Andrew Hundt, Dave Coleman
 */

#include <ur_modern_driver/ur_hardware_interface_standalone.h>
#include <boost/make_shared.hpp>

namespace ur {

UrHardwareInterfaceStandalone::UrHardwareInterfaceStandalone(
                         std::string host
                        ,boost::shared_ptr<std::condition_variable> rt_msg_cond_
	                    ,boost::shared_ptr<std::condition_variable> msg_cond_
                        ,boost::shared_ptr<UrDriver> robot
                        ,bool run_automatically
                        ){
	// Initialize shared memory and interfaces here
	init(); // this implementation loads from rosparam

	max_vel_change_ = 0.12; // equivalent of an acceleration of 15 rad/sec^2

	printf("ur_hardware_interface_standalone:Loaded ur_hardware_interface_standalone.");
    
    
        // start up the driver thread since the io_service_ is internal only
        if(run_automatically)
        {
          control_thread_.reset(new std::thread([&]{ run(); }));
        }
}


UrHardwareInterfaceStandalone::UrHardwareInterfaceStandalone(
                         std::string host
                        ){
                        
                        boost::shared_ptr<std::condition_variable> rt_msg_cond = boost::make_shared<std::condition_variable>();
	                    boost::shared_ptr<std::condition_variable> msg_cond    = boost::make_shared<std::condition_variable>();
                        boost::shared_ptr<UrDriver>                robot       = boost::make_shared<UrDriver>(*rt_msg_cond,*msg_cond,host);
                        UrHardwareInterfaceStandalone(host,rt_msg_cond,msg_cond,robot);
}

void UrHardwareInterfaceStandalone::init() {
	printf("initializing");

	// Get joint names
//	nh_.getParam("hardware_interface/joints", joint_names_);
//	if (joint_names_.size() == 0) {
//		ROS_FATAL_STREAM_NAMED("ur_hardware_interface_standalone",
//				"No joints found on parameter server for controller, did you load the proper yaml file?" << " Namespace: " << nh_.getNamespace());
//		exit(-1);
//	}
//	num_joints_ = joint_names_.size();
    num_joints_ = 6;

	// Resize vectors
	joint_position_.resize(num_joints_);
	joint_velocity_.resize(num_joints_);
	joint_effort_.resize(num_joints_);
	joint_position_command_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);
	prev_joint_velocity_command_.resize(num_joints_);

	// Initialize controller
	for (std::size_t i = 0; i < num_joints_; ++i) {
		std::cout << "ur_hardware_interface_standalone" <<	"Loading joint name: " << joint_names_[i];

		// Create joint state interface
//		joint_state_interface_.registerHandle(
//				hardware_interface::JointStateHandle(joint_names_[i],
//						&joint_position_[i], &joint_velocity_[i],
//						&joint_effort_[i]));

		// Create position joint interface
//		position_joint_interface_.registerHandle(
//				hardware_interface::JointHandle(
//						joint_state_interface_.getHandle(joint_names_[i]),
//						&joint_position_command_[i]));

		// Create velocity joint interface
//		velocity_joint_interface_.registerHandle(
//				hardware_interface::JointHandle(
//						joint_state_interface_.getHandle(joint_names_[i]),
//						&joint_velocity_command_[i]));
		prev_joint_velocity_command_[i] = 0.;
	}

	// Create force torque interface
//	force_torque_interface_.registerHandle(
//			hardware_interface::ForceTorqueSensorHandle("wrench", "",
//					robot_force_, robot_torque_));

//	registerInterface(&joint_state_interface_); // From RobotHW base class.
//	registerInterface(&position_joint_interface_); // From RobotHW base class.
//	registerInterface(&velocity_joint_interface_); // From RobotHW base class.
//	registerInterface(&force_torque_interface_); // From RobotHW base class.
	velocity_interface_running_ = false;
	position_interface_running_ = true;
}

void UrHardwareInterfaceStandalone::read() {
	std::unique_lock<std::mutex> locker(usr_access_lock);
	std::vector<double> pos, vel, current, tcp;
	pos = robot_->rt_interface_->robot_state_->getQActual();
	vel = robot_->rt_interface_->robot_state_->getQdActual();
	current = robot_->rt_interface_->robot_state_->getIActual();
	tcp = robot_->rt_interface_->robot_state_->getTcpForce();
	for (std::size_t i = 0; i < num_joints_; ++i) {
		joint_position_[i] = pos[i];
		joint_velocity_[i] = vel[i];
		joint_effort_[i] = current[i];
	}
	for (std::size_t i = 0; i < 3; ++i) {
		robot_force_[i] = tcp[i];
		robot_torque_[i] = tcp[i + 3];
	}

}

void UrHardwareInterfaceStandalone::setMaxVelChange(double inp) {
	std::unique_lock<std::mutex> locker(usr_access_lock);
	max_vel_change_ = inp;
}

void UrHardwareInterfaceStandalone::write() {
	std::unique_lock<std::mutex> locker(usr_access_lock);
	if (velocity_interface_running_) {
		std::vector<double> cmd;
		//do some rate limiting
		cmd.resize(joint_velocity_command_.size());
		for (unsigned int i = 0; i < joint_velocity_command_.size(); i++) {
			cmd[i] = joint_velocity_command_[i];
			if (cmd[i] > prev_joint_velocity_command_[i] + max_vel_change_) {
				cmd[i] = prev_joint_velocity_command_[i] + max_vel_change_;
			} else if (cmd[i]
					< prev_joint_velocity_command_[i] - max_vel_change_) {
				cmd[i] = prev_joint_velocity_command_[i] - max_vel_change_;
			}
			prev_joint_velocity_command_[i] = cmd[i];
		}
		robot_->setSpeed(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5],  max_vel_change_*125);
	} else if (position_interface_running_) {
		robot_->servoj(joint_position_command_);
	}
}

bool UrHardwareInterfaceStandalone::canSwitch() const {

// we can always stop a controller
	return true;
}

void UrHardwareInterfaceStandalone::doSwitch() {

	if (velocity_interface_running_ == true) {
		velocity_interface_running_ = false;
		printf("Stopping velocity interface");
	} else if (velocity_interface_running_ == false) {
		velocity_interface_running_ = true;
		printf("Starting velocity interface");
	}
    
	if (position_interface_running_ == false) {
		position_interface_running_ = true;
		robot_->uploadProg();
		printf("Starting position interface");
	} else {
		position_interface_running_ = false;
		std::vector<double> tmp;
		robot_->closeServo(tmp);
		printf("Stopping position interface");
	}

}

} // namespace ur

