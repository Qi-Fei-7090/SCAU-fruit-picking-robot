#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
ROS_VERSION = os.getenv('ROS_DISTRO')

import copy
import math

import rospy
import py_trees
import py_trees_ros
from py_trees.common import Status
import moveit_commander
import moveit_msgs.msg
import threading
import tf
import geometry_msgs.msg
import numpy as np

g_commanders = dict()

def get_commander(group_name):
    if group_name not in g_commanders:
        # 如果字典中没有指定的键，进行初始化      
        g_commanders[group_name] = moveit_commander.move_group.MoveGroupCommander(group_name)
        
        ####################################################################################################################################
        g_commanders[group_name].set_planning_time(3.0)  # 规划时间增加到10秒
        g_commanders[group_name].set_num_planning_attempts(5)  # 规划尝试次数增加到5次
        ####################################################################################################################################
        
        g_commanders[group_name].set_max_acceleration_scaling_factor(1.0)
        g_commanders[group_name].set_max_velocity_scaling_factor(1.0)
        g_commanders[group_name].set_goal_position_tolerance(0.001)
        g_commanders[group_name].set_goal_orientation_tolerance(0.017)
        g_commanders[group_name].allow_replanning(True)
        g_commanders[group_name].get_current_pose()

    return g_commanders[group_name]


class ArmMovetoPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, group_name="xarm6", goal=None, goal_name=None):
        super(ArmMovetoPose, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.group_name = group_name
        self.goal_name = goal_name
        self.goal = goal
        if goal is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self._commander = get_commander(self.group_name)
        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        if not self.goal_name:
            return
        try:
            self.goal = self.bb.get(self.goal_name)
            self.bb.set(self.goal_name, None)
        except Exception as e:
            self.logger.error("[Ex] get xarm_target_pose exception: {}".format(e))

    def update(self):
        if self.goal == None:
            self.logger.error("no goal [{}]".format(self.goal_name))
            return Status.FAILURE
        if self.status != Status.RUNNING:
            self.move_down = False
            self.move_success = False
            self.move_thread = threading.Thread(target=self.async_move)
            self.move_thread.start()
            return Status.RUNNING
        if self.move_down == False:
            return Status.RUNNING
        if self.move_success == True:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def async_move(self):
        goal = self.goal[:]
        goal[3:] = [math.radians(i) for i in goal[3:]]
        self._commander.set_start_state_to_current_state()
        self._commander.set_pose_target(goal)
        plan = None
        if ROS_VERSION == "noetic":
            ret, plan, planning_time, error_code = self._commander.plan()
            self.logger.info("plan finish, ret={} planning_time={} error_code={}".format(ret, planning_time, error_code))
        else:
            plan = self._commander.plan() # only retrun plan in melodic
        if not plan or not plan.joint_trajectory.points:
            self.move_down = True
            self.move_success = False
            return
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.logger.info("move start, goal={}".format(goal))
        try:
            # ret = self._commander.go(wait=True)
            ret = self._commander.execute(plan, wait=True)
        except Exception as e:
            self.logger.error("[Ex] arm move exception: {}".format(e))
            self.move_down = True
            self.move_success = False
            return
        self.logger.info("move finish, ret={}".format(ret))
        self.move_down = True
        if ret:
            self.move_success = True
            return
        else:
            self.move_success = False
            return

    def terminate(self, new_status):
        if not self.static_goal:
            self.goal = None
        self._commander.stop()
        self._commander.clear_pose_targets()


class ArmMoveJoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, group_name="xarm6", goal=None, goal_name=None):
        super(ArmMoveJoints, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.group_name = group_name
        self.goal_name = goal_name
        self.goal = goal
        if goal is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self._commander = get_commander(self.group_name)
        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        if not self.goal_name:
            return
        try:
            self.goal = self.bb.get(self.goal_name)
            self.bb.set(self.goal_name, None)
        except Exception as e:
            self.logger.error("[Ex] get xarm_target_pose exception: {}".format(e))

    def update(self):
        if self.goal is None:
            self.logger.error("no goal [{}]".format(self.goal_name))
            return Status.FAILURE
        if self.status != Status.RUNNING:
            self.move_down = False
            self.move_success = False
            self.move_thread = threading.Thread(target=self.async_move)
            self.move_thread.start()
            return Status.RUNNING
        if self.move_down == False:
            return Status.RUNNING
        if self.move_success == True:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def async_move(self):
        goal = self._commander.get_current_joint_values()
        for i in range(len(goal)):
            if i >= len(self.goal):
                break
            if self.goal[i] is not None:
                goal[i] = math.radians(self.goal[i])
        # joint_target = [math.radians(i) for i in self.goal]
        self._commander.set_joint_value_target(goal)
        plan = None
        if ROS_VERSION == "noetic":
            ret, plan, planning_time, error_code = self._commander.plan()
            self.logger.info("plan finish, ret={} planning_time={} error_code={}".format(ret, planning_time, error_code))
        else:
            plan = self._commander.plan() # only retrun plan in melodic
        if not plan or not plan.joint_trajectory.points:
            self.move_down = True
            self.move_success = False
            return
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.logger.info("move start, goal={}".format(goal))
        try:
            # ret = self._commander.go(wait=True)
            ret = self._commander.execute(plan, wait=True)
        except Exception as e:
            self.logger.error("[Ex] arm move exception: {}".format(e))
            self.move_down = True
            self.move_success = False
            return
        self.logger.info("move to finish, ret={}".format(ret))
        self.move_down = True
        if ret:
            self.move_success = True
        else:
            self.move_success = False

    def terminate(self, new_status):
        if not self.static_goal:
            self.goal = None
        self._commander.stop()
        self._commander.clear_pose_targets()


class ArmMoveToName(py_trees.behaviour.Behaviour):
    def __init__(self, name, group_name="xarm6", goal=None, goal_name=None):
        super(ArmMoveToName, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.group_name = group_name
        self.goal_name = goal_name
        self.goal = goal
        if goal is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self._commander = get_commander(self.group_name)
        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        if not self.goal_name:
            return
        try:
            self.goal = self.bb.get(self.goal_name)
            self.bb.set(self.goal_name, None)
        except Exception as e:
            self.logger.error("[Ex] get xarm_target_pose exception: {}".format(e))

    def update(self):
        if self.goal is None:
            return Status.FAILURE
        if self.status != Status.RUNNING:
            self.move_down = False
            self.move_success = False
            self.move_thread = threading.Thread(target=self.async_move)
            self.move_thread.start()
            return Status.RUNNING
        if self.move_down == False:
            return Status.RUNNING
        if self.move_success == True:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def async_move(self):
        self._commander.set_named_target(self.goal)
        plan = None
        if ROS_VERSION == "noetic":
            ret, plan, planning_time, error_code = self._commander.plan()
            self.logger.info("plan finish, ret={} planning_time={} error_code={}".format(ret, planning_time, error_code))
        else:
            plan = self._commander.plan() # only retrun plan in melodic
        if not plan or not plan.joint_trajectory.points:
            self.move_down = True
            self.move_success = False
            return
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.logger.info("move start, goal={}".format(self.goal))
        try:
            # ret = self._commander.go(wait=True)
            ret = self._commander.execute(plan, wait=True)
        except Exception as e:
            self.logger.error("[Ex] arm move exception: {}".format(e))
            self.move_down = True
            self.move_success = False
            return
        self.logger.info("move to finish, ret={}".format(ret))
        self.move_down = True
        if ret:
            self.move_success = True
            return
        else:
            self.move_success = False
            return

    def terminate(self, new_status):
        if not self.static_goal:
            self.goal = None
        self._commander.stop()
        self._commander.clear_pose_targets()


class ArmMovetoPoseInCartesian(py_trees.behaviour.Behaviour):
    def __init__(self, name, group_name="xarm6", goal=None, goal_name=None):
        super(ArmMovetoPoseInCartesian, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.group_name = group_name
        self.goal_name = goal_name
        self.goal = goal
        if goal is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self._commander = get_commander(self.group_name)
        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        if not self.goal_name:
            return
        try:
            self.goal = self.bb.get(self.goal_name)
            self.bb.set(self.goal_name, None)
        except Exception as e:
            self.logger.info("[Ex] get xarm_move_line_goal exception: {}".format(e))

    def update(self):
        if self.goal is None:
            return Status.FAILURE
        if self.status != Status.RUNNING:
            self.move_down = False
            self.move_success = False
            self.tcp_line_move_thread = threading.Thread(target=self.async_move, args=(self.goal,))
            self.tcp_line_move_thread.start()
            return Status.RUNNING
        if self.move_down == False:
            return Status.RUNNING
        if self.move_success == True:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def async_move(self, goal):
        wpose = self._commander.get_current_pose().pose

        quaternion = tf.listener.transformations.quaternion_from_euler(self.goal[3], self.goal[4], self.goal[5])

        # 构造Pose消息
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = self.goal[0]
        target_pose.position.y = self.goal[1]
        target_pose.position.z = self.goal[2]
        target_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
        target_pose.orientation = wpose.orientation

        waypoints = []
        waypoints.append(copy.deepcopy(target_pose))
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数
        while fraction < 1.0 and attempts < maxtries:
            self._commander.set_start_state_to_current_state()
            (plan, fraction) = self._commander.compute_cartesian_path(waypoints, 0.01, 2)
            attempts += 1
        self.logger.info("get plan : {} with {} times".format(fraction, attempts))
        if fraction < 1.0:
            self.move_down = True
            self.move_success = False
            return

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.logger.info("move start, goal={}".format(goal))
        try:
            # ret = self._commander.go(wait=True)
            ret = self._commander.execute(plan, wait=True)
        except Exception as e:
            self.logger.error("[Ex] arm move exception: {}".format(e))
            self.move_down = True
            self.move_success = False
            return
        self.logger.info("move finish, ret={}".format(ret))
        self.move_down = True
        if ret == False:
            self.logger.error("plan succeed but move failed!!!")
            self.move_success = False
        else:
            self.move_success = True

    def terminate(self, new_status):
        if not self.static_goal:
            self.goal = None


class ArmMoveEndInCartesian(py_trees.behaviour.Behaviour):
    def __init__(self, name, group_name="xarm6", goal=None, goal_name=None):
        super(ArmMoveEndInCartesian, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.group_name = group_name
        self.goal_name = goal_name
        self.goal = goal
        if goal is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self._commander = get_commander(self.group_name)
        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        if not self.goal_name:
            return
        try:
            self.goal = self.bb.get(self.goal_name)
            self.bb.set(self.goal_name, None)
        except Exception as e:
            self.logger.info("[Ex] get xarm_move_line_goal exception: {}".format(e))

    def update(self):
        if self.goal is None:
            return Status.FAILURE
        if self.status != Status.RUNNING:
            self.move_down = False
            self.move_success = False
            self.tcp_line_move_thread = threading.Thread(target=self.async_move, args=(self.goal,))
            self.tcp_line_move_thread.start()
            return Status.RUNNING
        if self.move_down == False:
            return Status.RUNNING
        if self.move_success == True:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def async_move(self, goal):
        # only x y z move not r p y
        wpose = self._commander.get_current_pose().pose
        original_trans = tf.listener.transformations.translation_matrix([wpose.position.x, wpose.position.y, wpose.position.z])
        original_orient = tf.listener.transformations.quaternion_matrix(
            [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
        )
        delata_matrix = tf.listener.transformations.translation_matrix(self.goal[:3])
        goal_matrix = np.dot(original_trans, original_orient)
        goal_matrix = np.dot(goal_matrix, delata_matrix)
        new_xyz = tf.listener.transformations.translation_from_matrix(goal_matrix)

        # 构造Pose消息
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = new_xyz[0]
        target_pose.position.y = new_xyz[1]
        target_pose.position.z = new_xyz[2]
        target_pose.orientation = wpose.orientation

        waypoints = []
        waypoints.append(copy.deepcopy(target_pose))
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数
        while fraction < 1.0 and attempts < maxtries:
            self._commander.set_start_state_to_current_state()
            (plan, fraction) = self._commander.compute_cartesian_path(waypoints, 0.01, 2)
            attempts += 1
        self.logger.info("get plan : {} with {} times".format(fraction, attempts))
        if fraction < 1.0:
            self.move_down = True
            self.move_success = False
            return

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)
        self.logger.info("move start, goal={}".format(goal))
        try:
            # ret = self._commander.go(wait=True)
            ret = self._commander.execute(plan, wait=True)
        except Exception as e:
            self.logger.error("[Ex] arm move exception: {}".format(e))
            self.move_down = True
            self.move_success = False
            return
        self.logger.info("move finish, ret={}".format(ret))
        self.move_down = True
        if ret == False:
            self.logger.error("plan succeed but move failed!!!")
            self.move_success = False
        else:
            self.move_success = True

    def terminate(self, new_status):
        if not self.static_goal:
            self.goal = None

def create_queue_tree():
    """
    A better tree implementation which uses a queue of location names stored in
    the blackboard to iterate through visiting locations
    """
    bb = py_trees.blackboard.Blackboard()

    bb.set("joint_goal1", [None, None, None, None, None, 90], True)
    bb.set("joint_goal2", [None, None, None, None, None, 0], True)
    bb.set("dadfa", [0.3, 0, 0.3, 180, 0, 180], True)

    main_sequence = py_trees.composites.Sequence(name="seq_xarm")
    main_sequence.add_children(
        [
            ArmMoveJoints("ArmMoveJoints1", group_name="left_arm", goal=[0, 0, 0, 180, 0, 0,0]),
            ArmMovetoPose("ddddd", group_name="left_arm", goal=[0.4, 0.2, 1.2, 0, 0, 0]),
            ArmMoveEndInCartesian("ArmMoveEndInCartesian", group_name="left_arm", goal=[0.1, 0, 0, 0, 0, 0]),
            ArmMovetoPoseInCartesian("ArmMovetoPoseInCartesian", group_name="left_arm", goal=[0.4, 0.4, 1.2, 0, 0, 0]),
            ArmMoveToName("ArmMoveToName", group_name="left_arm",goal= "reset_pose"),
        ]
    )
    return main_sequence


def post_tick_handler(tree):
    if tree.root.status == py_trees.common.Status.SUCCESS:
        rospy.loginfo("Success!")
        # tree.interrupt()
    elif tree.root.status == py_trees.common.Status.FAILURE:
        tree.interrupt()
        rospy.loginfo("Failed!")


if __name__ == "__main__":
    # Start ROS node
    rospy.init_node("test_xarm_node")

    root = create_queue_tree()
    ros_tree = py_trees_ros.trees.BehaviourTree(root)
    ros_tree.setup(timeout=10.0)
    py_trees.logging.level = py_trees.logging.Level.INFO

    ros_tree.tick_tock(
        100,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        pre_tick_handler=None,
        post_tick_handler=post_tick_handler,
    )
