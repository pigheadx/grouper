/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#ifndef DYNAMIXEL_CONTROL_H
#define DYNAMIXEL_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_control_msgs/MotorState.h>
#include <dynamixel_control_msgs/MotorControl.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <diagnostic_updater/diagnostic_updater.h>

//#define PAN_TILT_MOTOR 0
//#define PAN_MOTOR      0
//#define TILT_MOTOR     1

#define VELOCITY      2
#define ACCELERATION  10

namespace dynamixel_control
{
class DynamixelControl
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dynamixel_state_pub_;
  // ROS Service Server
  ros::ServiceServer dynamixel_control_server;
  // ROS Diagnostic Updater
  diagnostic_updater::Updater diagnostic_;
  // Parameters
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string device_name_;
  int baud_rate_;
  float protocol_version_;
  int motor_count_;

  std::map<std::string, int> motor_name_index_;
  std::vector<std::string> motor_name_;
  std::vector<std::string> motor_model_;
  std::vector<int> motor_id_;

  std::map<uint8_t, std::map<std::string, int64_t> > motor_state_;

  std::vector<uint32_t > state_msg_seq;

 public:
  DynamixelControl();
  ~DynamixelControl();
  bool dynamixelControlLoop(void);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

 private:
  bool initDynamixelControl(void);
  bool shutdownDynamixelControl(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);

  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readMotorState(uint8_t motor, std::string addr_name);

  bool syncWriteDynamixels(uint8_t motor, uint16_t addr, uint8_t length, int64_t value);
  bool writeTorqueEnable(bool onoff);
  bool writeProfile();
  bool writePosition(uint8_t motor, int64_t pos);
  bool writeSpeed(uint8_t motor, int64_t spd);

  int64_t convertRadian2Value(uint8_t motor, double radian);

  bool getPublishedMsg(void);
  bool motorControlCallback(dynamixel_control_msgs::MotorControl::Request &req,
                                   dynamixel_control_msgs::MotorControl::Response &res);
};
}

#endif //DYNAMIXEL_CONTROL_H
