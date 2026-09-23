#!/usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import time
from jaka_msgs.srv import SetIO

class JakaGripper:
  def __init__(self):
    rospy.wait_for_service('/jaka_driver/set_io')
    self.setIOService = rospy.ServiceProxy('/jaka_driver/set_io', SetIO)

  def setCabinetDigitalIO(self, index, value):
    # 代表 IO 类型,控制柜面 IO= 0、工具 IO= 1、拓展 IO= 2;
    #2024 06 26适配:控制柜面-->工具 IO= 1
    return self.setIOService.call('digital',1, index, value)

  def open(self):
    resp = self.setCabinetDigitalIO(1, 1)
    if resp.ret == 1:
      resp = self.setCabinetDigitalIO(2, 0)
      if resp.ret == 1:
        time.sleep(2)
        rospy.loginfo("gripper open success!")
        return
    rospy.loginfo("gripper open failed!")

    

  def close(self):
    resp = self.setCabinetDigitalIO(1, 0)
    if resp.ret == 1:
      resp = self.setCabinetDigitalIO(2, 1)
      if resp.ret == 1:
        time.sleep(2)
        rospy.loginfo("gripper close success!")
        return
    rospy.loginfo("gripper close failed!")

if __name__ == '__main__':
  rospy.init_node('jaka_gripper')
  gripper = JakaGripper()

  gripper.close()
  time.sleep(1)
  gripper.open()
