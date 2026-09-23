#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import py_trees
import py_trees_ros
from py_trees.common import Status
import moveit_commander
import threading


class Gripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, gripper_name="", goal=""):
        super(Gripper, self).__init__(name)
        self.gripper_name = gripper_name
        self.gripper = None
        self.goal = goal
        self.move_down = False
        self.move_success = False

    def setup(self, timeout):
        self.logger.info("gripper name: %s" % self.gripper_name)
        if self.gripper_name == "":
            self.logger.error("No gripper used.")
            return False
        elif self.gripper_name == "debug_gripper":
            pass
        elif self.gripper_name == "robotiq":
            from bt_task_manager.gripper.robotiq.robotiq_gripper import RobotiqGripper

            self.gripper = RobotiqGripper()
        elif self.gripper_name == "dh-gripper":
            from bt_task_manager.gripper.dh.dh_gripper import DHGripper

            self.gripper = DHGripper()
        elif self.gripper_name == "srt-gripper":
            from bt_task_manager.gripper.srt.srt_gripper import SRTGripper

            self.gripper = SRTGripper()
        elif self.gripper_name == "panda_hand":
            from bt_task_manager.gripper.franka.panda_gripper import PandaGripper

            self.gripper = PandaGripper()
        elif self.gripper_name == "jaka_gripper":
            from bt_task_manager.gripper.jaka_io.jaka_gripper import JakaGripper

            self.gripper = JakaGripper()
        else:
            self.logger.info("Use moveit control gripper: {}".format(self.gripper_name))
            self.gripper = moveit_commander.MoveGroupCommander(self.gripper_name)
        return True

    def initialise(self):
        return

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
        ret = False
        if self.goal == "open":
            self.open()
            ret = True
        elif self.goal == "close":
            self.close()
            ret = True
        self.move_down = True
        if ret:
            self.move_success = True
            return
        else:
            self.move_success = False
            return

    def terminate(self, new_status):
        return

    def open(self):
        self.logger.info("open_gripper.")
        if (
            self.gripper_name == "robotiq"
            or self.gripper_name == "dh-gripper"
            or self.gripper_name == "srt-gripper"
            or self.gripper_name == "panda_hand"
            or self.gripper_name == "jaka_gripper"
        ):
            if self.gripper:
                self.logger.info("Call gripper open command.")
                self.gripper.open()
        elif self.gripper_name == "debug_gripper":
            self.logger.info("Call gripper open command in [debug]")
        else:
            if self.gripper:
                self.gripper.set_named_target("open")
                self.gripper.go(wait=True)

    def close(self):
        self.logger.info("close_gripper.")
        if (
            self.gripper_name == "robotiq"
            or self.gripper_name == "dh-gripper"
            or self.gripper_name == "srt-gripper"
            or self.gripper_name == "panda_hand"
            or self.gripper_name == "jaka_gripper"
        ):
            if self.gripper:
                self.logger.info("Call gripper close command.")
                self.gripper.close()
        elif self.gripper_name == "debug_gripper":
            self.logger.info("Call gripper close command in [debug]")
        else:
            if self.gripper:
                self.gripper.set_named_target("close")
                self.gripper.go(wait=True)


def create_queue_tree():
    """
    A better tree implementation which uses a queue of location names stored in
    the blackboard to iterate through visiting locations
    """
    bb = py_trees.blackboard.Blackboard()

    main_sequence = py_trees.composites.Sequence(name="seq_xarm")
    main_sequence.add_children(
        [
            Gripper("open gripper", "left_gripper", "open"),
            Gripper("close gripper", "left_gripper", "close"),
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
