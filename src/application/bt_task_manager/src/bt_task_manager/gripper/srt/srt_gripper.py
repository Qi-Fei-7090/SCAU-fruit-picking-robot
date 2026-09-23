#!/usr/bin/python
# -*- coding:utf-8 -*-

'''
软体机器人柔性夹爪控制，适用SCB-18IAS驱动器。
适用串口通信
调试串口号为 /usb/ttyUSB0
'''
'''
夹爪控制状态代码
FREE_TRG（空闲） 0x00
POS_TRG（正压）  0x01
NEG_TRG（负压）  0x02
RES_TRG（泄压）  0x03
CYC_TRG（循环）  0x04
'''

import serial
import time
import modbus_tk.defines as cst
import modbus_tk
from modbus_tk import modbus_rtu
import rospy


class SRTGripper:
  def __init__(self):
    self._retry_times = 100  #通信失败时最多尝试的次数
    self._devport = "/dev/ttyUSB0"   # 如果是真实串口请根据实际串口设备号进行修改
    self.open_control()
    #开闭测试 
    self.close()
    self.open()


  def open_control(self):
    try:
      self._master = modbus_rtu.RtuMaster(serial.Serial(port=self._devport, baudrate=38400, bytesize=8,parity='N', stopbits=1, xonxoff=0))
      self._master.set_timeout(0.5)
    except serial.serialutil.SerialException as ex:
      rospy.logerr(ex)

  def open(self):
    index = 0
    while(index < self._retry_times):
      try:
        #RTU_ID=1 ,写入保持寄存器=0x06,0,写入数据0x02
        ret = self._master.execute(1,0x06,0x1A, 0,0x02)
        time.sleep(0.8)
        break
      except modbus_tk.exceptions.ModbusInvalidResponseError as ex:
              error_msg = "Grasp controler is No response. "
              error_msg += "Please check the physical connection between computer and grasp controler,and power."
              rospy.logerr(error_msg)
              rospy.logwarn("Try reconnect grasp controler({})! retry times:{}/{}".format(self._devport,index,self._retry_times))
              self.open_control()
              index += 1  #重试次数+1
    if(index == self._retry_times):
        rospy.logerr("grasp open fail!!!!!")
    else:
      rospy.loginfo("grasp open success!!!!!")


  def close(self):
    index = 0
    while(index < self._retry_times):
      try:
        #RTU_ID=1 ,写入保持寄存器=0x06,0,写入数据0x01
        ret = self._master.execute(1,0x06,0x1A, 0,0x01)
        time.sleep(0.8)
        break
      except modbus_tk.exceptions.ModbusInvalidResponseError as ex:
              error_msg = "Grasp controler is No response. "
              error_msg += "Please check the physical connection between computer and grasp controler,and power."
              rospy.logerr(error_msg)
              rospy.logwarn("Try reconnect grasp controler({})! retry times:{}/{}".format(self._devport,index,self._retry_times))
              self.open_control()
              index += 1  #重试次数+1
    if(index == self._retry_times):
        rospy.logerr("grasp close fail!!!!!")
    else:
      rospy.loginfo("grasp close success!!!!!")

if __name__ == '__main__':
  rospy.init_node('SRT-gripper')
  gripper = SRTGripper()

  gripper.close()
  time.sleep(1)
  gripper.open()
