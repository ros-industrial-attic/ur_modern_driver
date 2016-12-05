/*
 * robot_state.cpp
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

#include "ur_modern_driver/robot_state.h"


RobotState::RobotState(std::condition_variable& msg_cond) {
	version_msg_.major_version = 0;
	version_msg_.minor_version = 0;
	new_data_available_ = false;
	pMsg_cond_ = &msg_cond;
	RobotState::setDisconnected();
	robot_mode_running_ = robotStateTypeV30::ROBOT_MODE_RUNNING;
}
inline double RobotState::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}
void RobotState::unpack(uint8_t* buf, unsigned int buf_length) {
	/* Returns missing bytes to unpack a message, or 0 if all data was parsed */
	unsigned int offset = 0;
	while (buf_length > offset) {
		int len;
		unsigned char message_type;
		unpackVariable(buf,offset, len);
		if (len + offset > buf_length) {
			return;
		}
		unpackVariable(buf,offset, message_type);
		switch (message_type) {
		case messageType::ROBOT_MESSAGE:
			RobotState::unpackRobotMessage(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
			break;
		case messageType::ROBOT_STATE:
			RobotState::unpackRobotState(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
			break;
		case messageType::PROGRAM_STATE_MESSAGE:
			//Don't do anything atm...
		default:
			break;
		}
		offset += (len-5);
	}
	return;
}

void RobotState::unpackRobotMessage(uint8_t* buf, unsigned int offset,
		uint32_t len) {
	uint64_t timestamp;
	int8_t source, robot_message_type;
	
	memcpy(&robot_mode_.timestamp, &buf[offset], sizeof(robot_mode_.timestamp));
	offset += sizeof(robot_mode_.timestamp);
	// NOTE: aren't we supposed to do robot_mode_.timestamp = be64toh(robot_mode_.timestamp) ???
	
	unpackVariable( buf, offset, source);
	unpackVariable( buf, offset, robot_message_type);
	switch (robot_message_type) {
	case robotMessageType::ROBOT_MESSAGE_VERSION:
	{
		std::unique_lock<std::recursive_mutex> lock(val_lock_);
		version_msg_.timestamp = timestamp;
		version_msg_.source = source;
		version_msg_.robot_message_type = robot_message_type;
		RobotState::unpackRobotMessageVersion(buf, offset, len);
	}
	break;
	default:
		break;
	}
	
}

void RobotState::unpackRobotState(uint8_t * buf, unsigned int offset,
		uint32_t len) {
	while (offset < len) {
		int32_t length;
		uint8_t package_type;
		unpackVariable(buf, offset, length);
		unpackVariable(buf, offset, package_type);
		{
			std::unique_lock<std::recursive_mutex> lock(val_lock_);
			
			switch (package_type) {
			case packageType::ROBOT_MODE_DATA:
				RobotState::unpackRobotMode(buf, offset );
				break;
				
			case packageType::MASTERBOARD_DATA:
				RobotState::unpackRobotStateMasterboard(buf, offset );
				break;
			default:
				break;
			}
		}// end of lock
		offset += length-5;
	}
	new_data_available_ = true;
	pMsg_cond_->notify_all();
	
}

void RobotState::unpackRobotMessageVersion(uint8_t * buf, unsigned int offset,
		uint32_t len) {
	unpackVariable( buf, offset, version_msg_.project_name_size);
	
	memcpy(&version_msg_.project_name, &buf[offset],
			sizeof(char) * version_msg_.project_name_size);
	offset += version_msg_.project_name_size;
	version_msg_.project_name[version_msg_.project_name_size] = '\0';
	
	unpackVariable( buf, offset, version_msg_.major_version);
	unpackVariable( buf, offset, version_msg_.minor_version);
	unpackVariable( buf, offset, version_msg_.svn_revision);
	
	memcpy(&version_msg_.build_date, &buf[offset], sizeof(char) * len - offset);
	version_msg_.build_date[len - offset] = '\0';
	if (version_msg_.major_version < 2) {
		robot_mode_running_ = robotStateTypeV18::ROBOT_RUNNING_MODE;
	}
}

void RobotState::unpackRobotMode(uint8_t * buf, unsigned int offset) {
	
	memcpy(&robot_mode_.timestamp, &buf[offset], sizeof(robot_mode_.timestamp));
	offset += sizeof(robot_mode_.timestamp);
	// NOTE: aren't we supposed to do robot_mode_.timestamp = be64toh(robot_mode_.timestamp) ???
	
	unpackVariable( buf, offset, robot_mode_.isRobotConnected);
	unpackVariable( buf, offset, robot_mode_.isRealRobotEnabled);
	unpackVariable( buf, offset, robot_mode_.isPowerOnRobot);
	unpackVariable( buf, offset, robot_mode_.isEmergencyStopped);
	unpackVariable( buf, offset, robot_mode_.isProtectiveStopped);
	unpackVariable( buf, offset, robot_mode_.isProgramRunning);
	unpackVariable( buf, offset, robot_mode_.isProgramPaused);
	unpackVariable( buf, offset, robot_mode_.robotMode);
	
	if (RobotState::getVersion() > 2.) {
		unpackVariable( buf, offset, robot_mode_.controlMode);
		unpackVariable( buf, offset, robot_mode_.targetSpeedFraction);
	}
	unpackVariable( buf, offset, robot_mode_.speedScaling);
}

void RobotState::unpackRobotStateMasterboard(uint8_t * buf,
		unsigned int offset) {
	if (RobotState::getVersion() < 3.0) {
		int16_t digital_input_bits, digital_output_bits;
		unpackVariable( buf, offset, digital_input_bits);
		unpackVariable( buf, offset, digital_output_bits);
		mb_data_.digitalInputBits  = (digital_input_bits);
		mb_data_.digitalOutputBits = (digital_output_bits);
	} else {
		unpackVariable( buf, offset, mb_data_.digitalInputBits);
		unpackVariable( buf, offset, mb_data_.digitalOutputBits);
	}
	
	unpackVariable( buf, offset, mb_data_.analogInputRange0);
	unpackVariable( buf, offset, mb_data_.analogInputRange1);
	unpackVariable( buf, offset, mb_data_.analogInput0);
	unpackVariable( buf, offset, mb_data_.analogInput1);
	unpackVariable( buf, offset, mb_data_.analogOutputDomain0);
	unpackVariable( buf, offset, mb_data_.analogOutputDomain1);
	unpackVariable( buf, offset, mb_data_.analogOutput0);
	unpackVariable( buf, offset, mb_data_.analogOutput1);
	unpackVariable( buf, offset, mb_data_.masterBoardTemperature);
	unpackVariable( buf, offset, mb_data_.robotVoltage48V);
	unpackVariable( buf, offset, mb_data_.robotCurrent);
	unpackVariable( buf, offset, mb_data_.masterIOCurrent);
	unpackVariable( buf, offset, mb_data_.safetyMode);
	unpackVariable( buf, offset, mb_data_.masterOnOffState);
	unpackVariable( buf, offset, mb_data_.euromap67InterfaceInstalled);
	
	if (mb_data_.euromap67InterfaceInstalled != 0) {
		unpackVariable( buf, offset, mb_data_.euromapInputBits);
		unpackVariable( buf, offset, mb_data_.euromapOutputBits);
		if (RobotState::getVersion() < 3.0) {
			int16_t euromap_voltage, euromap_current;
			unpackVariable( buf, offset, euromap_voltage);
			unpackVariable( buf, offset, euromap_current);
			mb_data_.euromapVoltage = (euromap_voltage);
			mb_data_.euromapCurrent = (euromap_current);
		} else {
			unpackVariable( buf, offset, mb_data_.euromapVoltage);
			unpackVariable( buf, offset, mb_data_.euromapCurrent);
		}
	}
}

double RobotState::getVersion() {
	std::unique_lock<std::recursive_mutex> lock(val_lock_);
	return version_msg_.major_version + 0.1 * version_msg_.minor_version
			+ .0000001 * version_msg_.svn_revision;
}

void RobotState::finishedReading() {
	new_data_available_ = false;
}

bool RobotState::getNewDataAvailable() const {
	return new_data_available_;
}

int RobotState::getDigitalInputBits() const {
	return mb_data_.digitalInputBits;
}
int RobotState::getDigitalOutputBits() const {
	return mb_data_.digitalOutputBits;
}
double RobotState::getAnalogInput0() const {
	return mb_data_.analogInput0;
}
double RobotState::getAnalogInput1() const {
	return mb_data_.analogInput1;
}
double RobotState::getAnalogOutput0() const {
	return mb_data_.analogOutput0;
	
}
double RobotState::getAnalogOutput1() const {
	return mb_data_.analogOutput1;
}
bool RobotState::isRobotConnected() const {
	return robot_mode_.isRobotConnected;
}
bool RobotState::isRealRobotEnabled() const {
	return robot_mode_.isRealRobotEnabled;
}
bool RobotState::isPowerOnRobot() const {
	return robot_mode_.isPowerOnRobot;
}
bool RobotState::isEmergencyStopped() const {
	return robot_mode_.isEmergencyStopped;
}
bool RobotState::isProtectiveStopped() const {
	return robot_mode_.isProtectiveStopped;
}
bool RobotState::isProgramRunning() const {
	return robot_mode_.isProgramRunning;
}
bool RobotState::isProgramPaused() const {
	return robot_mode_.isProgramPaused;
}
unsigned char RobotState::getRobotMode() const {
	return robot_mode_.robotMode;
}
bool RobotState::isReady() const {
	return (robot_mode_.robotMode == robot_mode_running_);
}

void RobotState::setDisconnected() {
	robot_mode_.isRobotConnected = false;
	robot_mode_.isRealRobotEnabled = false;
	robot_mode_.isPowerOnRobot = false;
}
