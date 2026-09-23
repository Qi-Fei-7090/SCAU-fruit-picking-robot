#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import numpy as np
import rospy
import py_trees
import py_trees_ros
import tf
from sensor_msgs.msg import Image,CompressedImage,CameraInfo,PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32


class GetGraspPoseByTopic(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        base_frame="base_link",
        camera_frame="camera_color_optical_frame",
        target_type="",
        out_frame="grasp_pose",
        feed_depth=0,
        offset_x=0,
        offset_y=0,
        offset_z=-0.15, #0.12
        grasp_angles=[0,90,0],
    ):
        super(GetGraspPoseByTopic, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.target_type = target_type
        if self.target_type == "":
            self.get_target_type_from_param = True
        self.logger.info("target  error")
        self.out_frame = out_frame
        self.feed_depth = feed_depth
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        self.grasp_angles = np.radians(grasp_angles)
        self.targets = None
        self.stable_count = 0

    def setup(self, timeout):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.objects_sub = rospy.Subscriber("/yolo5/camera/detected_objects", PointCloud, self.objects_callback)
        return True

    def initialise(self):
        self.targets = None
        self.stable_count = 0
        if self.get_target_type_from_param:
            self.target_type = rospy.get_param("/pick_target_type","")
        pass

    def update(self):
        if self.status != py_trees.Status.RUNNING:
            self.status = py_trees.Status.RUNNING
        
        if self.stable_count <= 10:
            return py_trees.Status.RUNNING

        # space in camera link
        space_x = 0.0
        space_y = 0.0
        space_z = 0.4
        targets = []
        target = None
        for i in range(len(self.targets)):
            if self.target_type != "":
                if self.targets[i][1] != self.target_type:
                    continue
            xyz = self.targets[i][0]
            # if (xyz[0] < space_x-0.2 or xyz[0] > space_x+0.2 or
            #     xyz[1] < space_y-0.1 or xyz[1] > space_y+0.1 or
            #     xyz[2] < space_z-0.1 or xyz[2] > space_z+0.1):
            #     continue
            if target is None:
                target = self.targets[i]
            if xyz[2] < target[0][2]:
                target = self.targets[i]
            targets.append(self.targets[i])
        if target is None:
            self.targets = None
            self.stable_count = 0
            return py_trees.Status.RUNNING
        self.logger.info("{}".format(target))

        # transfor tf 

        try:
            self.tf_listener.waitForTransform(self.base_frame, self.camera_frame, rospy.Time(), rospy.Duration(0.5))
            camera_xyz, camera_quat = self.tf_listener.lookupTransform(self.base_frame, self.camera_frame, rospy.Time(0))
            
            camera_trans = tf.listener.transformations.translation_matrix(camera_xyz)
            camera_orient = tf.listener.transformations.quaternion_matrix(camera_quat)
            target_trans_in_camera = tf.listener.transformations.translation_matrix([target[0][0],target[0][1],target[0][2]+0.03])# buchang yuanzhuti banjing 3cm
            target_transform = np.dot(camera_trans, camera_orient)
            target_transform = np.dot(target_transform, target_trans_in_camera)  # offset in tcp coordinate
            target_translation = tf.listener.transformations.translation_from_matrix(target_transform)
            translation_matrix = tf.listener.transformations.translation_matrix(target_translation)

            grasp_orient = tf.listener.transformations.euler_matrix(self.grasp_angles[0], self.grasp_angles[1], self.grasp_angles[2])

            delta_trans = tf.listener.transformations.translation_matrix([self.offset_x, self.offset_y, self.offset_z - self.feed_depth])   
            grasp_transform = np.dot(translation_matrix, grasp_orient)
            grasp_transform = np.dot(grasp_transform, delta_trans)  # offset in tcp coordinate   
        
            translate = tf.listener.transformations.translation_from_matrix(grasp_transform)
            angles = tf.listener.transformations.euler_from_matrix(grasp_transform)
            rotation = tf.listener.transformations.quaternion_from_matrix(grasp_transform)
            angles = [math.degrees(a) for a in angles]
            self.logger.info("target {} {}".format(translate, angles))
            self.bb.set(self.out_frame, list(translate) + list(angles))
            self.tf_broadcaster.sendTransform(translate, rotation, rospy.Time.now(), self.out_frame, self.base_frame)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:

            self.logger.error("tf error. {}".format(ex))
            self.stable_count = 0
            self.last_xyz = None
            self.last_quat = None
            return py_trees.Status.RUNNING

        return py_trees.Status.SUCCESS


    def objects_callback(self, msg):
        if self.status != py_trees.Status.RUNNING:
            return
        objects = []
        obj_sum = len(msg.points)
        for i in range(obj_sum):
            xyz = [msg.points[i].x,msg.points[i].y,msg.points[i].z]
            type = msg.channels[i].values[0]
            id =  msg.channels[i].values[1]
            prob = msg.channels[i].values[2]
            objects.append([xyz,type,id,prob])
        if self.targets is None or len(self.targets) != len(objects):
            self.targets = objects
            self.stable_count = 0
            return
        for i in range(obj_sum):
            if objects[i][2] != self.targets[i][2]:
                self.targets = objects
                self.stable_count = 0
                return
            if ( math.fabs(objects[i][0][0] - self.targets[i][0][0]) > 0.01
                or math.fabs(objects[i][0][1] - self.targets[i][0][1]) > 0.01
                or math.fabs(objects[i][0][2] - self.targets[i][0][2]) > 0.01):
                self.targets = objects
                self.stable_count = 0
                return
        self.stable_count += 1



def create_queue_tree():
    """
    A better tree implementation which uses a queue of location names stored in
    the blackboard to iterate through visiting locations
    """
    bb = py_trees.blackboard.Blackboard()

    main_sequence = py_trees.composites.Sequence(name="seq_xarm")
    main_sequence.add_children(
        [
            GetGraspPoseByTopic("test" ),
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
