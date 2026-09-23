#!/usr/bin/env python
# -*- coding:utf-8 -*-

'''
@File    :  robotiq_gripper.py
@Time    :  2022/04/13 08:48:17
@Author  :  Damon
@Version :  1.0
@Contact :  support@163.com
@License :  (C)Copyright 2021-2022, Damon
@Desc:   Responsible for task scheduling.
'''

'''
依赖 Robotiq-2f-85: https://github.com/KevinGalassi/Robotiq-2f-85
'''

import rospy
import time
# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal

class RobotiqGripper:
  def __init__(self):
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    self.robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

    rospy.loginfo('Wait {}'.format(action_name))
    self.robotiq_client.wait_for_server()
    rospy.loginfo('Gripper action server exist')

    self.goal = CommandRobotiqGripperGoal()
    self.goal.emergency_release = False
    self.goal.stop = False
    self.goal.position = 0.00
    self.goal.speed = 0.1
    self.goal.force = 0.001

  def open(self):
    self.position(0.085)
    # rospy.loginfo(self.position(0.085))

  def close(self):
    self.position(0.0)
    # rospy.loginfo(self.position(0.0))

  def position(self, position):
    self.goal.position = position
    # Sends the goal to the gripper.
    self.robotiq_client.send_goal(self.goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    self.robotiq_client.wait_for_result()

    return self.robotiq_client.get_result() 

if __name__ == '__main__':
  rospy.init_node('robotiq_2f_client')
  gripper = RobotiqGripper()

  gripper.close()
  time.sleep(1)
  gripper.open()
  time.sleep(1)
  gripper.position(0.05)