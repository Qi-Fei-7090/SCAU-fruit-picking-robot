#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import numpy as np
import rospy
import py_trees
import py_trees_ros
import tf


class GetGraspPoseByTF(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        base_frame="base_link",
        target_frame="",
        out_frame="grasp_pose",
        feed_depth=0.1,
        offset_x=0,
        offset_y=0,
        offset_z=0,
        use_target_orient=False,
        grasp_angles=[90, 0, 90],
    ):
        super(GetGraspPoseByTF, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.base_frame = base_frame
        self.target_frame = target_frame
        self.out_frame = out_frame
        self.feed_depth = feed_depth
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        self.use_target_orient = use_target_orient
        self.grasp_angles = np.radians(grasp_angles)

    def setup(self, timeout):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        return True

    def initialise(self):
        self.stable_count = 0
        self.last_xyz = None
        self.last_quat = None
        pass

    def update(self):
        try:
            self.tf_listener.waitForTransform(self.base_frame, self.target_frame, rospy.Time(), rospy.Duration(0.5))
            xyz, quat = self.tf_listener.lookupTransform(self.base_frame, self.target_frame, rospy.Time(0))

            if self.last_xyz is None:
                self.last_xyz = xyz
                self.last_quat = quat
                return py_trees.Status.RUNNING

            if (
                math.fabs(xyz[0] - self.last_xyz[0]) > 0.01
                or math.fabs(xyz[1] - self.last_xyz[1]) > 0.01
                or math.fabs(xyz[2] - self.last_xyz[2]) > 0.01
            ):
                self.last_xyz = xyz
                self.last_quat = quat
                return py_trees.Status.RUNNING

            # TODO 增加多个采样融合估计平均位姿
            self.stable_count += 1
            if self.stable_count < 10:
                return py_trees.Status.RUNNING

            grasp_transform = tf.listener.transformations.compose_matrix()
            if self.use_target_orient:
                target_trans = tf.listener.transformations.translation_matrix(self.last_xyz)
                target_orient = tf.listener.transformations.quaternion_matrix(self.last_quat)
                delta_trans = tf.listener.transformations.translation_matrix([self.offset_x, self.offset_y, self.offset_z + self.feed_depth])
                grasp_orient = tf.listener.transformations.euler_matrix(math.pi, 0, 0)  # reserve 180 degree szyx
                grasp_transform = np.dot(target_trans, target_orient)
                grasp_transform = np.dot(grasp_transform, delta_trans)  # offset in tcp coordinate
                grasp_transform = np.dot(grasp_transform, grasp_orient)  # reserve 180 degree
            else:
                target_trans = tf.listener.transformations.translation_matrix(self.last_xyz)
                grasp_orient = tf.listener.transformations.euler_matrix(self.grasp_angles[0], self.grasp_angles[1], self.grasp_angles[2])
                delta_trans = tf.listener.transformations.translation_matrix([self.offset_x, self.offset_y, self.offset_z - self.feed_depth])
                grasp_transform = np.dot(target_trans, grasp_orient)
                grasp_transform = np.dot(grasp_transform, delta_trans)  # offset in tcp coordinate

            translate = tf.listener.transformations.translation_from_matrix(grasp_transform)
            angles = tf.listener.transformations.euler_from_matrix(grasp_transform)
            rotation = tf.listener.transformations.quaternion_from_matrix(grasp_transform)
            angles = [math.degrees(a) for a in angles]
            self.logger.info("target {} {}".format(translate, angles))
            self.bb.set(self.out_frame, list(translate) + list(angles))
            self.tf_broadcaster.sendTransform(translate, rotation, rospy.Time.now(), self.out_frame, self.base_frame)
            return py_trees.Status.SUCCESS

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            self.logger.error("tf error. {}".format(ex))
            self.stable_count = 0
            self.last_xyz = None
            self.last_quat = None
            return py_trees.Status.RUNNING


def create_queue_tree():
    """
    A better tree implementation which uses a queue of location names stored in
    the blackboard to iterate through visiting locations
    """
    bb = py_trees.blackboard.Blackboard()

    main_sequence = py_trees.composites.Sequence(name="seq_xarm")
    main_sequence.add_children(
        [
            GetGraspPoseByTF("test", base_frame="base_link", target_frame="goal_link", out_frame="grasp_pose", grasp_angles=[0, 90, 0]),
        ]
    )
    return main_sequence


def post_tick_handler(tree):
    if tree.root.status == py_trees.common.Status.SUCCESS:
        rospy.loginfo("Success!")
        # tree.interrupt()
    elif tree.root.status == py_trees.common.Status.FAILURE:
        # tree.interrupt()
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
