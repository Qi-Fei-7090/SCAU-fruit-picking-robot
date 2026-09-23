/**************************************************************************
 * Copyright (C), 2020-2025, Robot++ Co., Ltd.
 * Author: Damon
 * version: 0.1
 * Date: 2020-12-03
 * 
 * Description: Command 
 **************************************************************************/ 

#include "command.h"
#include "ros/ros.h"

template<typename T>
void buildBytes(const T & V, vector<unsigned char> & buffer)
{
  unsigned int size_value(sizeof(T));
  for (unsigned int i = 0; i < size_value; i++)
  {
    buffer.push_back(static_cast<unsigned char>((V >> (i * 8)) & 0xff));
  }
}

unsigned short getChecksum(vector<uint8_t> & cmdStream) {
  unsigned short checksum = 0xFFFF;
  unsigned short tmp = 0;
  for (unsigned int i = 0; i < cmdStream.size(); i++) {
    checksum ^= (cmdStream[i]);
    for(unsigned int j =0; j< 8; j++) {
     if(checksum & 0x01)
       checksum = (checksum >> 1) ^ 0xa001;
     else
       checksum = checksum >> 1;
    }
  }
  
  return checksum;
}


vector<uint8_t> Command::disableError(const uint8_t mask[], const int len) {
  //ROS_INFO("disableError: mask(%ld)", mask);
  vector<uint8_t> mask_cmd; 

  buildBytes(static_cast<uint8_t>(0xff), mask_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), mask_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), mask_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x26), mask_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x08), mask_cmd); // len

  buildBytes(static_cast<uint8_t>(mask[0]), mask_cmd); // 电机异常使能位
  buildBytes(static_cast<uint8_t>(mask[1]), mask_cmd); // 系统异常使能位
  buildBytes(static_cast<uint8_t>(mask[2]), mask_cmd); // 限位异常使能位
  buildBytes(static_cast<uint8_t>(mask[3]), mask_cmd); // 通信异常使能位
  buildBytes(static_cast<uint32_t>(0xffffffff), mask_cmd); 

  buildBytes(getChecksum(mask_cmd), mask_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), mask_cmd);
   
  return mask_cmd;
}

vector<uint8_t> Command::setVelocityControl(const float &linear, const float &angular) {
  //ROS_INFO("setVelocityControl: linear(%lf), angular(%lf)", linear, angular);
  vector<uint8_t> vel_cmd;

  buildBytes(static_cast<uint8_t>(0xff), vel_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), vel_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), vel_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x0f), vel_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x08), vel_cmd); // len
  buildBytes(static_cast<uint32_t>((int)(linear * 1000)), vel_cmd);
  buildBytes(static_cast<uint32_t>((int)(angular * 1000)), vel_cmd);

  buildBytes(getChecksum(vel_cmd), vel_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), vel_cmd);

  return vel_cmd;
}

vector<uint8_t> Command::setVelocityControl(const float &linear_x, const float &linear_y, const float &angular) {
  //ROS_INFO("setVelocityControl: linear(%lf), angular(%lf)", linear, angular);
  vector<uint8_t> vel_cmd;

  buildBytes(static_cast<uint8_t>(0xff), vel_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), vel_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), vel_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x0e), vel_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x0c), vel_cmd); // 12Bytes
  buildBytes(static_cast<uint32_t>(int(linear_x * 1000)), vel_cmd);
  buildBytes(static_cast<uint32_t>(int(linear_y * 1000)), vel_cmd);
  buildBytes(static_cast<uint32_t>(int(angular * 1000)), vel_cmd);

  buildBytes(getChecksum(vel_cmd), vel_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), vel_cmd);
   
  return vel_cmd;
}

vector<uint8_t> Command::setSpeedSteering(uint8_t model, const float &linear_x, const float &steering_speed, const float &steering_angular) {
  ROS_INFO("setSpeedSteering: linear_x(%lf), steering_speed(%lf), steering_angular(%lf)", linear_x, steering_speed, steering_angular);
  vector<uint8_t> speed_steering_cmd;

  buildBytes(static_cast<uint8_t>(0xff), speed_steering_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), speed_steering_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), speed_steering_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x0c), speed_steering_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x11), speed_steering_cmd); // 17Bytes
  buildBytes(model, speed_steering_cmd);
  buildBytes(static_cast<uint16_t>(0x0), speed_steering_cmd); // 00
  buildBytes(static_cast<uint16_t>(0x0), speed_steering_cmd); // 00
  buildBytes(static_cast<uint32_t>(int(linear_x * 1000)), speed_steering_cmd);
  buildBytes(static_cast<uint32_t>(int(steering_speed * 1000)), speed_steering_cmd);
  buildBytes(static_cast<uint32_t>(int(steering_angular * 1000)), speed_steering_cmd);

  buildBytes(getChecksum(speed_steering_cmd), speed_steering_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), vel_cmd);
   
  return speed_steering_cmd;
}

vector<uint8_t> Command::setThrottleSteering(const uint8_t gear_position, const uint16_t throttle, const uint16_t brake_strength, const float &constant_speed, const float &steering_speed, const float &steering_angular) {
 // ROS_INFO("setThrottleSteering: gear_position(0x%02x), throttle(%d), brake_strength(%d), constant_speed(%lf), steering_speed(%lf), steering_angular(%lf)",gear_position, throttle, brake_strength, constant_speed, steering_speed, steering_angular);
  vector<uint8_t> throttle_steering_cmd;

  buildBytes(static_cast<uint8_t>(0xff), throttle_steering_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), throttle_steering_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), throttle_steering_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x0c), throttle_steering_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x11), throttle_steering_cmd); // 17Bytes
  buildBytes(gear_position, throttle_steering_cmd);
  buildBytes(static_cast<uint16_t>(throttle), throttle_steering_cmd);
  buildBytes(static_cast<uint16_t>(brake_strength), throttle_steering_cmd);
  buildBytes(static_cast<uint32_t>(int(constant_speed * 1000)), throttle_steering_cmd);
  buildBytes(static_cast<uint32_t>(int(steering_speed * 1000)), throttle_steering_cmd);
  buildBytes(static_cast<uint32_t>(int(steering_angular * 1000)), throttle_steering_cmd);

  buildBytes(getChecksum(throttle_steering_cmd), throttle_steering_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), vel_cmd);
  //for(int i=0; i<throttle_steering_cmd.size(); i++) {
  //   printf(" 0x%02x,",throttle_steering_cmd[i]);
  //}
  // printf("\n");
  return throttle_steering_cmd;
}

vector<uint8_t> Command::setDeviceStatus(uint8_t model, uint8_t motor) {
  ROS_INFO_THROTTLE(10, "setDeviceStatus: model(%d), motor(%d)", model, motor);
  vector<uint8_t> control_cmd;

  buildBytes(static_cast<uint8_t>(0xff), control_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), control_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), control_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x04), control_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x08), control_cmd); // len
  buildBytes(model, control_cmd);
  buildBytes(static_cast<uint8_t>(0xff), control_cmd); // power 电源管理位，暂时固定写为 0xff，即:全部上电
  buildBytes(motor, control_cmd);
  buildBytes(static_cast<uint8_t>(0x00), control_cmd); // vel ratio
  buildBytes(static_cast<uint32_t>(0x00), control_cmd); // power

  buildBytes(getChecksum(control_cmd), control_cmd);
  //buildBytes(static_cast<uint16_t>(0xffff), control_cmd);

  // buildBytes(static_cast<uint8_t>('\n'), control_cmd); // \n 0x0a
   
  return control_cmd;
}   

vector<uint8_t> Command::setLightStatus(const uint32_t front, const uint32_t rear) {
  ROS_INFO("setLightStatus: front(%d), rear(%d)", front, rear);
  vector<uint8_t> light_control_cmd;

  buildBytes(static_cast<uint8_t>(0xff), light_control_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), light_control_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), light_control_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x15), light_control_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x08), light_control_cmd); // len
  buildBytes(static_cast<uint32_t>(front), light_control_cmd); 
  buildBytes(static_cast<uint32_t>(rear), light_control_cmd); 
  //buildBytes(static_cast<uint16_t>(0xffff), light_control_cmd);
  buildBytes(getChecksum(light_control_cmd), light_control_cmd);

  // buildBytes(static_cast<uint8_t>('\n'), light_control_cmd); // \n 0x0a
 
  return light_control_cmd;
}

vector<uint8_t> Command::setHearts() {
  vector<uint8_t> hearts_cmd;

  buildBytes(static_cast<uint8_t>(0xff), hearts_cmd); // ff
  buildBytes(static_cast<uint8_t>(0xaa), hearts_cmd); // aa
  buildBytes(static_cast<uint8_t>(0x7f), hearts_cmd); // addr
  buildBytes(static_cast<uint8_t>(0x70), hearts_cmd); // cmd
  buildBytes(static_cast<uint16_t>(0x08), hearts_cmd); // len
  buildBytes(static_cast<uint32_t>(0x0), hearts_cmd); 
  buildBytes(static_cast<uint32_t>(0x0), hearts_cmd); 
  //buildBytes(static_cast<uint16_t>(0xffff), hearts_cmd);
  buildBytes(getChecksum(hearts_cmd), hearts_cmd);

  return hearts_cmd;
}

