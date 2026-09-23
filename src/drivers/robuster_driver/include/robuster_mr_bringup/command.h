/**************************************************************************
 * Copyright (C), 2020-2025, Robot++ Co., Ltd.
 * Author: Damon
 * version: 0.1
 * Date: 2020-12-03
 * 
 * Description: Command 
 **************************************************************************/ 

#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <iostream>
#include <vector>

using namespace std;

#define PI                  3.1415926

#define VOLTAGE_MAX         6000     // max voltage value is 60V *100
#define CURRENT_MAX         5000     // max current value is 50A *100

/**
 * robuster 系列产品通信协议复用, 2WD/4WD 在协议层均按 4WD 传输数据
 */
#define MOTOR_COUNTS        4

// 电机 ID
#define LF_MOTOR_ID         0
#define RF_MOTOR_ID         1
#define LB_MOTOR_ID         2
#define RB_MOTOR_ID         3

/// Error state
// 1. communication error
#define ERR_HC_DISC         0x01                // host computer disconnected
#define ERR_JOY_DISC        (ERR_HC_COMMU + 1)  // Joystick disconnected
#define ERR_DR_BUS_DISC     (ERR_HC_COMMU + 2)  // Drive bus which use to communicate with motors disconnected
#define ERR_EX_BUS_DISC     (ERR_HC_COMMU + 3)  // Expansion bus which use to extend other sensors disconnected
#define ERR_BATTERY_DISC    (ERR_HC_COMMU + 4)  // Battery disconnected
#define ERR_DISPLAY_DISC    (ERR_HC_COMMU + 5)  // display panel of the base disconnected


/// TODO: Protocol is not yet clear, to be determined
// Control command
#define SET_ACC_COMMAND     0x03    // set acceleration
#define SET_DEV_STATUS      0x04    // set model and motor(enable/disable)
#define SET_POWER_STATUE    0x05    // power manage    
#define SET_VEL_COMMAND     0x0f    // set velocity
#define SET_LIGHT_STATUS    0x15    // set front and rear light

// Config command
#define CLR_ERROR           0x24    // Clear error status
#define DIS_ERROR           0x26    // Mask error status
//#define                   0x40    // 配置字
#define WHEEL_DATA          0x42    // 
#define LINEAR_VEL_LIMIT    0x44    // 速度上限
#define ROTATE_VEL_LIMIT    0x46    // 转速上限
#define ENCODE_RATIO        0x48    // 编码器减速比
#define JOYSTICK_MID        0x4a    // 摇杆中位、遥控行程
#define JOYSTICK_CH_MAP     0x4f    // 遥控通道映射

#define SYNC_COMMAND        0x70    // 同步
#define S_FIREWARE_UPDATE   0x7a    // 固件升级设置
#define FIREWARE_UPDATE     0x7b    // 固件数据

// Status command
/* 电机相关状态 */
#define MOTOR_TICK_STATUS   0x90  // 电机位置(脉冲)
#define MOTOR_RPM_STATUS    0x91  // 电机转速
#define ODOMETRY_DATA       0x9b  // odometry data
//#define ODOM_THELTA         0x9d  // 里程计 θ
#define VEL_STATUS          0x9f  // cur veloticy and angle status
#define MOTOR_V_STATUS      0xa0  // 电机电压
#define MOTOR_A_STATUS      0xa1  // 电机电流
#define MOTOR_T_STATUS      0xa2  // 电机温度
#define MOTOR_E_STATUS      0xa3  // 电机异常

#define VERSION             0xbf

/* IMU 数据 0xd0~0xda */

/* Ultrasonic 数据 */
#define ULTRA_DATA          0xC5  

/* 电池相关状态 */
#define BATTERY_STATUS      0xed  // battery percentage and battery temperature
#define BATTERY_CAP_STATUS  0xee  // battery capacity(容量). Contains nominal capacity and remaining capacity
#define BATTERY_VA_STATUS   0xef  // Voltage and current status of the battery

#define CMD_STATUS  0xfc  

#define ERROR_STATUS        0xf0
#define DEVICE_STATUS       0xf4
#define LIGHT_STATUS        0xf5

unsigned short getChecksum(vector<uint8_t> & cmdStream);

class Command {
 public:
  
  struct Data {
    int8_t      addr;
    int8_t      cmd; 
    uint16_t    len;  

    int32_t     linear_vel;
    int32_t     angular_vel;
  };

  /** 清除报警/错误状态.
   * 
   *  \param:
   *  \return: clear error command
   */
  static vector<uint8_t> clearError(uint64_t mask);

  /** 清屏蔽报警/错误状态.
   * 
   *  \param: mask 与错误字指令相对应；0 - 表示关闭， 1 - 表示开启
   *      byte[1]: 通信异常使能位
   *      byte[2]: 系统异常使能位
   *      byte[3]: 限位异常使能位
   *      byte[4]: 电机异常使能位
   *      byte[5-8]: 保留
   *  \return: disable error command
   */
  static vector<uint8_t> disableError(const uint8_t mask[], const int len);

  /** 运动控制.
   * 
   *  \param:
   *      linear - 线速度. 单位：mm/s
   *      angular - 角速度. 单位: m_rad/s
   *  \return: command vector
   */
  static vector<uint8_t> setVelocityControl(const float &linear, const float &angular);

  /** 运动控制. 支持横向移动.
   * 
   *  \param:
   *      linear_x - x 轴线速度. 单位：mm/s
   *      linear_y - y 轴线速度(横向速度), 单位: mm/s
   *      angular - 角速度. 单位: m_rad/s
   *  \return: command vector
   */
  static vector<uint8_t> setVelocityControl(const float &linear_x, const float &linear_y, const float &angular);

  /** 阿克曼模型运动控制. 速度-转向控制.
   * 
   *  \param:
   *      model   指令模式，0x06当前指令有效，0x00 当前指令无效 使用指令模式1
   *      linear_x - x 轴线速度. 单位：毫米/秒（mm/s），前进为＋，后退为－
   *      steering_speed 转向速度, 单位：毫弧度/秒（mrad/s）
   *      steering_angular - 转向角度. 单位：毫弧度（mrad），中点左侧为＋，右侧为－
   *  \return: command vector
   */
  static vector<uint8_t> setSpeedSteering(uint8_t model, const float &linear_x, const float &steering_speed, const float &steering_angular);

  /** 阿克曼模型运动控制. 油门-转向控制.
   * 
   *  \param:
   *      gear_position 档位. 0x88：驻车（P）0x55：倒车（R）0x00：空挡（N）0xAA：前进（D）
   *      throttle 油门. 单位：千分比（‰）
   *      brake_strength   刹车强度. 单位：千分比（‰）
   *      constant_speed  定速   单位：毫米/秒（mm/s），前进为＋，后退为－
   *      steering_speed 转向速度, 单位：毫弧度/秒（mrad/s）
   *      steering_angular - 转向角度. 单位：毫弧度（mrad），中点左侧为＋，右侧为－
   *  \return: command vector
   */
  static vector<uint8_t> setThrottleSteering(const uint8_t gear_position, const uint16_t throttle, const uint16_t brake_strength, const float &constant_speed, const float &steering_speed, const float &steering_angular);
 
  /** 设置控制状态.
   * 
   *  \param:
   *      model - 模式. 0x01
   *      motor - 动力. 
   *              0x06: 电机关闭
   *              0x0f: 电机开启 
   *  \return: command vector
   */
  static vector<uint8_t> setDeviceStatus(const uint8_t model, const uint8_t motor);

  /** 前后灯光控制.
   * 
   *  \param:
   *      front - 前灯亮度. 0~4999 
   *      rear - 后灯亮度.  0~4999
   *  \return: command vector
   */
  static vector<uint8_t> setLightStatus(const uint32_t front, const uint32_t rear);

  /** 心跳包.
   *  \return: command vector
   */
  static vector<uint8_t> setHearts();

 private:
  // Data data;
  // vector<uint8_t> buffer_;
};

#endif