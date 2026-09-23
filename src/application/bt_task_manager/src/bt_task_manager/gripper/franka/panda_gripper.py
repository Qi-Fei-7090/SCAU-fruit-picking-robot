#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
@File    :  robotiq_gripper.py
@Time    :  2022/11/14 14:18:00
@Author  :  Damon
@Version :  1.0
@Contact :  support@163.com
@License :  (C)Copyright 2021-2022, Damon
@Desc:   Responsible for task scheduling.
'''

import rospy
import time
# Brings in the SimpleActionClient
import actionlib

from franka_gripper.msg import GraspAction, GraspGoal

class PandaGripper:
  def __init__(self):
    action_name = rospy.get_param('~action_name', '/franka_gripper/grasp')
    self.panda_gripper_client = actionlib.SimpleActionClient(action_name, GraspAction)

    rospy.loginfo('Wait {}'.format(action_name))
    self.panda_gripper_client.wait_for_server()
    rospy.loginfo('Gripper action server exist')

    self.goal = GraspGoal()
    self.goal.speed = 0.1
    self.goal.force = 10
    self.goal.epsilon.inner = 0.05
    self.goal.epsilon.outer = 0.05
    self.goal.width = 0.05

  def open(self):
    self.position(0.078)
    # rospy.loginfo(self.position(0.085))

  def close(self):
    self.position(0.05) # 0.0
    # rospy.loginfo(self.position(0.0))

  def position(self, position):
    rospy.loginfo("Set gripper position: %f" % position)
    self.goal.width = position
    # Sends the goal to the gripper.
    self.panda_gripper_client.send_goal(self.goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    self.panda_gripper_client.wait_for_result()

    return self.panda_gripper_client.get_result() 

if __name__ == '__main__':
  rospy.init_node('panda_gripper_client')
  gripper = PandaGripper()

  gripper.close()
  time.sleep(1)
  gripper.open()
  time.sleep(1)
  gripper.position(0.05)