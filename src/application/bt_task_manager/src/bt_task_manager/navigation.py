#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import tf
import rospy
import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def getRobotPose():
    try:
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        _, _, yaw = tf.listener.transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], yaw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Error getting transformation. Make sure the 'map' and 'base_link' frames are being published.")
        return None


def create_move_base_goal(x, y, theta):
    """ Creates a MoveBaseGoal message from a 2D navigation pose """
    angle = np.mod(theta + 180.0, 360) - 180
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = tf.listener.transformations.quaternion_from_euler(0, 0, np.radians(angle))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal

class Navigation(py_trees.behaviour.Behaviour):

    def __init__(self, name, goal, action_namespace="/move_base"):
        super(Navigation, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = MoveBaseAction
        self.goal = goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = "moving"
        self.logger.info("task name is [{}], goal is [{}]".format(name, goal))

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(self.action_namespace, self.action_spec)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            (x, y, theta)  =(self.goal[0], self.goal[1], self.goal[2])
            self.logger.info("Going to [{:.4f} {:.4f} {:.4f}]".format(x,y,theta))
            goal = create_move_base_goal(x, y, theta)
            self.action_client.send_goal(goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()

        status = self.action_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        else:
            pose = getRobotPose()
            if pose == None:
                return py_trees.common.Status.FAILURE
            if (abs(pose[0] - self.goal[0]) < 0.05 and abs(pose[1] - self.goal[1]) < 0.05 and abs(pose[2] - self.goal[2]) < 5):
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass
