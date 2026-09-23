#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
@File    :  dh_gripper.py
@Time    :  2022/07/08 14:29:00
@Author  :  Damon
@Version :  1.0
@Contact :  support@163.com
@License :  (C)Copyright 2021-2022, Damon
@Desc:   Responsible for task scheduling.
'''


import rospy
import time
from dh_gripper_msgs.msg import GripperCtrl

class DHGripper:
  def __init__(self):
    self._gripper_ctrl_pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=1)

    self.goal = GripperCtrl()
    self.goal.initialize = False
    self.goal.position = 0.00
    self.goal.force = 100.0
    self.goal.speed = 100.0

  def open(self):
    self.goal.position = 900.0
    self._gripper_ctrl_pub.publish(self.goal)
    # rospy.loginfo(self.position(0.085))
    time.sleep(2)

  def close(self):
    self.goal.position = 10.0
    self._gripper_ctrl_pub.publish(self.goal)
    # rospy.loginfo(self.position(0.0))
    time.sleep(2)

if __name__ == '__main__':
  rospy.init_node('dh-gripper')
  gripper = DHGripper()

  gripper.close()
  time.sleep(1)
  gripper.open()