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

#include "dynamixel_control/dynamixel_control.h"

using namespace dynamixel_control;

DynamixelControl::DynamixelControl()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0),
     protocol_version_(0.0),
     motor_count_(0)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
//  nh_priv_.getParam("motor_model_", motor_model_);
  nh_priv_.getParam("protocol_version_", protocol_version_);
  nh_priv_.getParam("motor_count_", motor_count_);

  // Init target name
  ROS_ASSERT(initDynamixelControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // init ROS Publish
  dynamixel_state_pub_ = nh_.advertise<dynamixel_control_msgs::MotorState>("dynamixel_control/motor_state",10);

  // init ROS Server
  dynamixel_control_server = nh_.advertiseService("dynamixel_control/motor_control", &DynamixelControl::motorControlCallback, this);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)\n!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
  }

  char buf[20];
  std::string motorParam;
  std::string paramTemp;
  int idTemp;
  for(int i=0;i<motor_count_;i++)
  {
    sprintf(buf,"motor_%d_/",i);
    motorParam = buf;
    nh_priv_.getParam((motorParam+"motor_name_"), paramTemp);
    motor_name_.push_back(paramTemp);
    motor_name_index_[paramTemp]=i;
    nh_priv_.getParam((motorParam+"motor_model_"), paramTemp);
    motor_model_.push_back(paramTemp);
    nh_priv_.getParam((motorParam+"motor_id_"), idTemp);
    motor_id_.push_back(idTemp);
    ROS_INFO("motor[%d]:", i);
    ROS_INFO("motor_name: %s", motor_name_[i].c_str());
    ROS_INFO("motor_model: %s", motor_model_[i].c_str());
    ROS_INFO("motor_id: %d", motor_id_[i]);
    ROS_INFO("motor_protocol_version_: %.1f\n", protocol_version_);
    initMotor(motor_model_[i], motor_id_[i], protocol_version_);
    state_msg_seq.push_back(0);
  }

//  nh_priv_.getParam("pan_motor_/motor_id_", motor_id_);
//  ROS_INFO("pan_motor_id: %d", motor_id_);
//  ROS_INFO("pan_motor_model: %s", motor_model_.c_str());
//  ROS_INFO("pan_motor_protocol_version_: %.1f\n", protocol_version_);

//  initMotor(motor_model_, motor_id_, protocol_version_);

//  nh_priv_.getParam("tilt_motor_/motor_id_", motor_id_);
//  ROS_INFO("tilt_motor_id: %d", motor_id_);
//  ROS_INFO("tilt_motor_model: %s", motor_model_.c_str());
//  ROS_INFO("tilt_motor_protocol_version_: %.1f", protocol_version_);

//  initMotor(motor_model_, motor_id_, protocol_version_);
  writeTorqueEnable(true);
  writeProfile();

  diagnostic_.add("DynamixelStatus", this, &DynamixelControl::diagnostics);
  std::string hardwareID;
  for(int i=0;i<motor_count_;i++){
    hardwareID=hardwareID+motor_name_[i].c_str();
    hardwareID=hardwareID+",";
  }
  diagnostic_.setHardwareID(hardwareID);
}

DynamixelControl::~DynamixelControl()
{
  ROS_ASSERT(shutdownDynamixelControl());
}

void DynamixelControl::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  uint8_t motorLED = 0;
  for(int i=0;i<motor_count_;i++)
  {
    motorLED = motorLED|motor_state_[i]["alarm_led"];
  }
  if (motorLED == 0)
    status.summary(0, "ok");
  else
    status.summary(2, "error");
  for(int i=0;i<motor_count_;i++)
  {
    status.add(motor_name_[i]+"_led", motor_state_[i]["alarm_led"]);
//    status.add(motor_name_[i]+"_vol", motor_state_[i]["present_voltage"]);
  }
}

bool DynamixelControl::initDynamixelControl(void)
{
  ROS_INFO("dynamixel_control : Init OK!");
  return true;
}

bool DynamixelControl::shutdownDynamixelControl(void)
{
  writeTorqueEnable(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelControl::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dynamixel_motor);
}

bool DynamixelControl::syncWriteDynamixels(uint8_t motor, uint16_t addr, uint8_t length, int64_t value)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr,length);

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[motor]->id_, (uint8_t*)&value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[motor]->id_);
    return false;
  }
  dynamixel_comm_result_ = groupSyncWrite.txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWrite.clearParam();
  return true;
}


/*
bool DynamixelControl::syncWriteDynamixels(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr,length);

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[PAN_MOTOR]->id_, (uint8_t*)&pan_motor_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[PAN_MOTOR]->id_);
    return false;
  }

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[TILT_MOTOR]->id_, (uint8_t*)&tilt_motor_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[TILT_MOTOR]->id_);
    return false;
  }

  dynamixel_comm_result_ = groupSyncWrite.txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWrite.clearParam();
  return true;
}
*/

bool DynamixelControl::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  uint8_t dynamixel_error = 0;
  int8_t dynamixel_comm_result_;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dynamixel_comm_result_ = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result_ = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result_ = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
  }

  if (dynamixel_comm_result_ == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
    }

    if (length == 1)
    {
      *value = value_8_bit;
      return true;
    }
    else if (length == 2)
    {
      *value = value_16_bit;
      return true;
    }
    else if (length == 4)
    {
      *value = value_32_bit;
      return true;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelControl::writeTorqueEnable(bool onoff)
{
  if (onoff == true)
  {
    for(int i=0;i<motor_count_;i++)
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["torque_enable"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, true);
    }
  }
  else
  {
    for(int i=0;i<motor_count_;i++)
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["torque_enable"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, false);
    }
  }
}

/*
bool DynamixelControl::writeTorqueEnable(bool onoff)
{
  dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["torque_enable"];
  if (onoff == true)
  {
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, true, true);
  }
  else
  {
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, false, false);
  }
}
*/

bool DynamixelControl::writeProfile()
{
  for(int i=0;i<motor_count_;i++){
    if (!strncmp(motor_model_[i].c_str(), "AX", 2) || !strncmp(motor_model_[i].c_str(), "RX", 2) || !strncmp(motor_model_[i].c_str(), "EX", 2))
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["moving_speed"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, VELOCITY);
    }
    else if (!strncmp(motor_model_[i].c_str(), "MX", 2))
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["moving_speed"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, VELOCITY);
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["goal_acceleration"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, ACCELERATION);
    }
    else if (!strncmp(motor_model_[i].c_str(), "PRO", 3))
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["goal_velocity"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, VELOCITY);
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["goal_acceleration"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, ACCELERATION);
    }
    else
    {
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["profile_velocity"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, VELOCITY);
      dynamixel_[i]->item_ = dynamixel_[i]->ctrl_table_["profile_acceleration"];
      syncWriteDynamixels(i, dynamixel_[i]->item_->address, dynamixel_[i]->item_->data_length, ACCELERATION);
    }
  }
}

/*
bool DynamixelControl::writeProfile()
{
  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
  }
  else if (!strncmp(motor_model_.c_str(), "MX", 2))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
  else if (!strncmp(motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_velocity"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
  else
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_velocity"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_acceleration"];
    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
}
*/

bool DynamixelControl::writePosition(uint8_t motor, int64_t pos)
{
  dynamixel_[motor]->item_ = dynamixel_[motor]->ctrl_table_["goal_position"];
  syncWriteDynamixels(motor, dynamixel_[motor]->item_->address, dynamixel_[motor]->item_->data_length, pos);
}

bool DynamixelControl::writeSpeed(uint8_t motor, int64_t spd)
{
  dynamixel_[motor]->item_ = dynamixel_[motor]->ctrl_table_["moving_speed"];
  syncWriteDynamixels(motor, dynamixel_[motor]->item_->address, dynamixel_[motor]->item_->data_length, spd);
}

bool DynamixelControl::readMotorState(uint8_t motor, std::string addr_name)
{
//  ROS_INFO("motor %u, addr %s", motor, addr_name.c_str());

  int64_t read_value;

  dynamixel_[motor]->item_ = dynamixel_[motor]->ctrl_table_[addr_name];

  readDynamixelRegister(dynamixel_[motor]->id_, dynamixel_[motor]->item_->address, dynamixel_[motor]->item_->data_length, &read_value);

  motor_state_[motor][addr_name] = read_value;
  return true;
}

bool DynamixelControl::getPublishedMsg(void)
{
  for(int i=0;i<motor_count_;i++){
    readMotorState(i, "alarm_led");
//    readMotorState(i, "present_voltage");
    readMotorState(i, "torque_enable");
    readMotorState(i, "moving");
    readMotorState(i, "goal_position");
    readMotorState(i, "present_position");
    readMotorState(i, "present_velocity");

    if (!strncmp(motor_model_[i].c_str(), "AX", 2) || !strncmp(motor_model_[i].c_str(), "RX", 2) || !strncmp(motor_model_[i].c_str(), "MX", 2) || !strncmp(motor_model_[i].c_str(), "EX", 2))
    {
      readMotorState(i,"moving_speed");
    }
    else
    {
      readMotorState(i, "goal_velocity");
    }

    if (!strncmp(motor_model_[i].c_str(), "MX", 2) || !strncmp(motor_model_[i].c_str(), "PRO", 3))
    {
      readMotorState(i, "goal_acceleration");
    }

    if (!strncmp(motor_model_[i].c_str(), "XM", 2))
    {
      readMotorState(i, "profile_velocity");
      readMotorState(i, "profile_acceleration");
    }

    if (!strncmp(motor_model_[i].c_str(), "XM", 2) || !strncmp(motor_model_[i].c_str(), "PRO", 3))
    {
      readMotorState(i, "min_position_limit");
      readMotorState(i, "max_position_limit");
    }
    else
    {
      readMotorState(i, "cw_angle_limit");
      readMotorState(i, "ccw_angle_limit");
    }
  }
}

int64_t DynamixelControl::convertRadian2Value(uint8_t motor, double radian)
{
  int64_t value = 0;
  if (radian > 0)
  {
    if (dynamixel_[motor]->value_of_max_radian_position_ <= dynamixel_[motor]->value_of_0_radian_position_)
      return dynamixel_[motor]->value_of_max_radian_position_;

    value = (radian * (dynamixel_[motor]->value_of_max_radian_position_ - dynamixel_[motor]->value_of_0_radian_position_) / dynamixel_[motor]->max_radian_)
                + dynamixel_[motor]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_[motor]->value_of_min_radian_position_ >= dynamixel_[motor]->value_of_0_radian_position_)
      return dynamixel_[motor]->value_of_min_radian_position_;

    value = (radian * (dynamixel_[motor]->value_of_min_radian_position_ - dynamixel_[motor]->value_of_0_radian_position_) / dynamixel_[motor]->min_radian_)
                + dynamixel_[motor]->value_of_0_radian_position_;
  }
  else
    value = dynamixel_[motor]->value_of_0_radian_position_;

  if (value > dynamixel_[motor]->value_of_max_radian_position_)
    return dynamixel_[motor]->value_of_max_radian_position_;
  else if (value < dynamixel_[motor]->value_of_min_radian_position_)
    return dynamixel_[motor]->value_of_min_radian_position_;

  return value;
}

bool DynamixelControl::dynamixelControlLoop(void)
{
  getPublishedMsg();

  diagnostic_.update();

  dynamixel_control_msgs::MotorState dynamixel_response[motor_count_];

  for (int i = 0; i < motor_count_; i++)
  {
    dynamixel_response[i].header.seq = state_msg_seq[i];
    state_msg_seq[i]++;
    dynamixel_response[i].header.stamp = ros::Time::now();
    dynamixel_response[i].header.frame_id = motor_name_[i];

    dynamixel_response[i].motor_model = dynamixel_[i]->model_name_;
    dynamixel_response[i].id = dynamixel_[i]->id_;
    dynamixel_response[i].torque_enable = (motor_state_[i]["torque_enable"] == 1);
    dynamixel_response[i].present_position = motor_state_[i]["present_position"];
    dynamixel_response[i].present_velocity = motor_state_[i]["present_velocity"];
    dynamixel_response[i].goal_position = motor_state_[i]["goal_position"];
    dynamixel_response[i].moving = (motor_state_[i]["moving"] == 1);

    if (!strncmp(motor_model_[i].c_str(), "AX", 2) || !strncmp(motor_model_[i].c_str(), "RX", 2) || !strncmp(motor_model_[i].c_str(), "MX", 2) || !strncmp(motor_model_[i].c_str(), "EX", 2))
    {
      dynamixel_response[i].moving_speed = motor_state_[i]["moving_speed"];
    }
    else
    {
      dynamixel_response[i].goal_velocity = motor_state_[i]["goal_velocity"];
    }

    if (!strncmp(motor_model_[i].c_str(), "MX", 2) || !strncmp(motor_model_[i].c_str(), "PRO", 3))
    {
      dynamixel_response[i].goal_acceleration = motor_state_[i]["goal_acceleration"];
    }

    if (!strncmp(motor_model_[i].c_str(), "XM", 2))
    {
      dynamixel_response[i].profile_velocity = motor_state_[i]["profile_velocity"];
      dynamixel_response[i].profile_acceleration = motor_state_[i]["profile_acceleration"];
    }

    if (!strncmp(motor_model_[i].c_str(), "XM", 2) || !strncmp(motor_model_[i].c_str(), "PRO", 3))
    {
      dynamixel_response[i].max_position_limit = motor_state_[i]["max_position_limit"];
      dynamixel_response[i].min_position_limit = motor_state_[i]["min_position_limit"];
    }
    else
    {
      dynamixel_response[i].cw_angle_limit = motor_state_[i]["cw_angle_limit"];
      dynamixel_response[i].ccw_angle_limit = motor_state_[i]["ccw_angle_limit"];
    }

    dynamixel_state_pub_.publish(dynamixel_response[i]);
  }
}

bool DynamixelControl::motorControlCallback(dynamixel_control_msgs::MotorControl::Request &req,
                                            dynamixel_control_msgs::MotorControl::Response &res)
{
  int8_t motor = motor_name_index_[req.motor_name];
  int64_t pos = 0;
  if (motor_name_[motor]!=req.motor_name)
  {
    ROS_ERROR("Illegal motor name.");
    return false;
  }
  else
  {
    if (req.control_type == "position") {
      if (req.unit == "rad")
      {
        pos = convertRadian2Value(motor, req.value);
      }
      else if (req.unit == "raw")
      {
        pos = req.value;
      }
      else
      {
        pos = req.value;
      }

      writePosition(motor, pos);
      res.value = pos;
      return true;
    }
      else if (req.control_type == "getposition") 
    {
      if (req.unit == "rad")
      {
        pos = convertRadian2Value(motor, req.value);
        res.value = pos;
        return true;
      }
      else
      {
        ROS_ERROR("Illegal control unit.");
        return false;
      }
    }
      else if (req.control_type == "velocity") 
    {
      ROS_ERROR("Velocity not supported yet.");
      return false;
    }
    else if (req.control_type == "torque")
    {
      ROS_ERROR("Torque not supported yet.");
      return false;
    }
    else 
    {
      ROS_ERROR("Illegal control type.");
      return false;
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_control");
  DynamixelControl dynamixel_pos_ctrl;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    dynamixel_pos_ctrl.dynamixelControlLoop();
    ros::spinOnce();
//    loop_rate.sleep();
  }

  return 0;
}
