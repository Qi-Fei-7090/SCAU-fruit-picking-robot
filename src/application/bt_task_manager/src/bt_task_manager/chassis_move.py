#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import py_trees
import py_trees_ros
import math
import time
from geometry_msgs.msg import Twist

def get_cmd_msg(direction, speed):
    """
    Generate a Twist message based on the given direction and speed.

    Parameters:
    - direction (str): The direction in which the robot should move ("forward", "backward", "rotate_left", "rotate_right").
    - speed (float): The speed at which the robot should move.

    Returns:
    - Twist: A Twist message containing the linear and angular velocities based on the input direction and speed.
    """
    cmd_vel_msg = Twist()
    if direction == "forward":
        cmd_vel_msg.linear.x = speed
    elif direction == "backward":
        cmd_vel_msg.linear.x = -speed
    elif direction == "rotate_left":
        cmd_vel_msg.angular.z = math.radians(speed)
    elif direction == "rotate_right":
        cmd_vel_msg.angular.z = -math.radians(speed)
    return cmd_vel_msg

class ChassisMove(py_trees.behaviour.Behaviour):
    """
    A behaviour to move chassis with given speed and direction.
    Args:
        direction: move direction, can be forward, backward, rotate_left, rotate_right
        distance: move distance, unit m or °
        speed: move speed, unit m/s or °/s
        cmd_topic: move cmd topic
    """
    def __init__(self, name, direction=None, distance=None, speed = 0.1, cmd_topic="/cmd_vel"):
        """
        Initialize the ChassisMove class.

        Parameters:
            name (str): The name of the object.
            direction (str, optional): The direction to move in. Defaults to None.
            distance (float, optional): The distance to move. Defaults to None.
            speed (float, optional): The speed of movement. Defaults to 0.1.
            cmd_topic (str, optional): The topic to publish movement commands to. Defaults to "/cmd_vel".

        Returns:
            None
        """
        super(ChassisMove, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.cmd_topic = cmd_topic
        self.direction = direction
        self.distance = distance
        self.speed = speed
        if distance is None:
            self.static_goal = False
        else:
            self.static_goal = True
        self.cmd_vel_msg = Twist()

    def setup(self, timeout):
        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        return True

    def initialise(self):
        if self.static_goal:
            return
        try:
            self.direction = self.bb.get("direction")
            self.distance = self.bb.get("distance")
            self.speed =  self.bb.get("speed")
        except Exception as e:
            self.logger.warning("[Ex] get param exception: {}".format(e))

    def update(self):
        if self.direction is None or self.distance is None or self.speed is None :
            return py_trees.common.Status.FAILURE
        if self.status != py_trees.common.Status.RUNNING:
            self.cmd_vel_msg = get_cmd_msg(self.direction,self.speed)
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            self.start_time = time.time()
            return py_trees.common.Status.RUNNING

        elapsed_time = time.time() - self.start_time
        self.distance_moved = self.speed * elapsed_time
        self.logger.info("Distance moved: {} ".format(self.distance_moved))
        if self.distance_moved >= self.distance:
            self.cmd_vel_msg = get_cmd_msg("stop",0)
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            return py_trees.common.Status.SUCCESS

        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if not self.static_goal:
            self.distance = None
            self.direction = None
            self.speed = None

def create_queue_tree():
    bb = py_trees.blackboard.Blackboard()

    main_sequence = py_trees.composites.Sequence(name="main_sequence")
    main_sequence.add_children(
        [
            # ChassisMove("chassis_move", "backward",    0.5, 0.1),
            # ChassisMove("chassis_move", "rotate_left", 180, 30),
            # ChassisMove("chassis_move", "forward",       1, 0.1),
            # ChassisMove("chassis_move", "rotate_left", 180, 30)
        ]
    )
    root = py_trees.decorators.OneShot(main_sequence, "one_shot")
    return main_sequence

def print_tree(tree):
    print(py_trees.display.ascii_tree(tree.root))
    if(tree.root.status == py_trees.common.Status.SUCCESS):
        rospy.loginfo("Success!")
        tree.interrupt()

if __name__ == "__main__":
    # Start ROS node
    rospy.init_node("test_navigation_node")

    root = create_queue_tree()
    ros_tree = py_trees_ros.trees.BehaviourTree(root)
    ros_tree.setup(timeout=10.0)
    py_trees.logging.level = py_trees.logging.Level.INFO


    ros_tree.tick_tock(10,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        # 100,
        pre_tick_handler=None,
        post_tick_handler=print_tree
    )
    # rospy.spin()
    rospy.loginfo("Done!")
