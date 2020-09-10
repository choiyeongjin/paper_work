#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
import math
import tf
import tf2_ros
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, TransformStamped
from ar_track_alvar_msgs.msg import *
from tf.transformations import *

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

MM2M = 1.0 / 1000.0
M2MM  = 1000.0

AR_MARKER_FRAME_PREFIX_ = 'ar_marker_'
AR_TARGET_FRAME_PREFIX_ = 'ar_target_'
CAMERA_FRAME_PREFIX_    = 'camera_link'

OFFSET_FROM_TARGET_X  = 0.0   * MM2M # 보정 [mm]
OFFSET_FROM_TARGET_Y  = 0.0   * MM2M # 250.0 [mm]
OFFSET_FROM_TARGET_Z  = 175.0 * MM2M # [mm]
OFFSET_FROM_TARGET_RX = 180.0
OFFSET_FROM_TARGET_RY = 0.0
OFFSET_FROM_TARGET_RZ = 90.0


class snu_object_tracker():
    def __init__(self):
        rospy.init_node('snu_obejct_tracker', anonymous=True)
        
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        ### Class Variables ###
        self.reference_frame_name  = rospy.get_param("snu_object_tracker/reference_frame",      'base_0')
        self.object_frame_name     = rospy.get_param("snu_object_tracker/object_frame",         'ar_marker_0')
        self.target_frame_name     = rospy.get_param("snu_object_tracker/target_frame",         'ar_target_0')
        self.offset_from_target_x  = rospy.get_param("snu_object_tracker/offset_from_target/x",  OFFSET_FROM_TARGET_X)
        self.offset_from_target_y  = rospy.get_param("snu_object_tracker/offset_from_target/y",  OFFSET_FROM_TARGET_Y)
        self.offset_from_target_z  = rospy.get_param("snu_object_tracker/offset_from_target/z",  OFFSET_FROM_TARGET_Z)
        self.offset_from_target_rx = rospy.get_param("snu_object_tracker/offset_from_target/rx", OFFSET_FROM_TARGET_RX)
        self.offset_from_target_ry = rospy.get_param("snu_object_tracker/offset_from_target/ry", OFFSET_FROM_TARGET_RY)
        self.offset_from_target_rz = rospy.get_param("snu_object_tracker/offset_from_target/rz", OFFSET_FROM_TARGET_RZ)
        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

        ### Topics to Subscribe ###
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1) # Trigger Topic
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_sub_cb, queue_size=1) # AR Marker Topic
        rospy.Subscriber('objects', Float32MultiArray, self.object_sub_cb, queue_size=1) # Object Pose Topic
        
        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked
    

    '''
        변환: (trans, rot) -> geometry_msgs/Pose
    '''
    def update_cmd_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1] # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]
    

    '''
        Update ROS Parameters
    '''    
    def update_ros_param(self):
        self.object_frame_name     = rospy.get_param("snu_object_tracker/object_frame")
        self.target_frame_name     = rospy.get_param("snu_object_tracker/target_frame")
        self.offset_from_target_x  = rospy.get_param("snu_object_tracker/offset_from_target/x")
        self.offset_from_target_y  = rospy.get_param("snu_object_tracker/offset_from_target/y")
        self.offset_from_target_z  = rospy.get_param("snu_object_tracker/offset_from_target/z")
        self.offset_from_target_rx = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rx")
        self.offset_from_target_ry = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/ry")
        self.offset_from_target_rz = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rz")
        

    '''
        AR_Marker -> Subscribe -> Target Pose 계산 -> TF Broadcast (새로운 Frame: TARGET_FRAME)
    '''
    def ar_sub_cb(self, msg):
        self.update_ros_param()

        n_tags = len(msg.markers)
        print "Number of detected tags: %d"%n_tags
        
        if n_tags is not 0:
            idx = 0
            for x in msg.markers:
                # self.static_transformStamped.header.stamp    = msg.header.stamp
                # self.static_transformStamped.header.frame_id = CAMERA_FRAME_PREFIX_
                # self.static_transformStamped.child_frame_id  = AR_MARKER_FRAME_PREFIX_ + str(msg.markers[idx].id)
                # self.static_transformStamped.transform.translation.x = msg.markers[idx].pose.pose.position.x
                # self.static_transformStamped.transform.translation.y = msg.markers[idx].pose.pose.position.y
                # self.static_transformStamped.transform.translation.z = msg.markers[idx].pose.pose.position.z
                # self.static_transformStamped.transform.rotation.x = msg.markers[idx].pose.pose.orientation.x
                # self.static_transformStamped.transform.rotation.y = msg.markers[idx].pose.pose.orientation.y
                # self.static_transformStamped.transform.rotation.z = msg.markers[idx].pose.pose.orientation.z
                # self.static_transformStamped.transform.rotation.w = msg.markers[idx].pose.pose.orientation.w
                # self.broadcaster.sendTransform(self.static_transformStamped)

                self.static_transformStamped.header.stamp    = msg.header.stamp
                self.static_transformStamped.header.frame_id = AR_MARKER_FRAME_PREFIX_ + str(msg.markers[idx].id)
                self.static_transformStamped.child_frame_id  = AR_TARGET_FRAME_PREFIX_ + str(msg.markers[idx].id)
                self.static_transformStamped.transform.translation.x = 0.0 + self.offset_from_target_x
                self.static_transformStamped.transform.translation.y = 0.0 + self.offset_from_target_y
                self.static_transformStamped.transform.translation.z = 0.0 + self.offset_from_target_z
                quat = tf.transformations.quaternion_from_euler(self.offset_from_target_rx, self.offset_from_target_ry, self.offset_from_target_rz)
                self.static_transformStamped.transform.rotation.x = quat[0]
                self.static_transformStamped.transform.rotation.y = quat[1]
                self.static_transformStamped.transform.rotation.z = quat[2]
                self.static_transformStamped.transform.rotation.w = quat[3]
                self.broadcaster.sendTransform(self.static_transformStamped)

                idx += 1
            return 1
        else:
            return -1


    def object_sub_cb(self, msg):
        self.update_ros_param()
        self.static_transformStamped.header.stamp    = rospy.Time.now()
        self.static_transformStamped.header.frame_id = self.object_frame_name
        self.static_transformStamped.child_frame_id  = self.target_frame_name
        
        self.static_transformStamped.transform.translation.x = self.offset_from_target
        self.static_transformStamped.transform.translation.y = 0.0
        self.static_transformStamped.transform.translation.z = 0.0
        
        quat = tf.transformations.quaternion_from_euler(0.0, -math.pi/2.0, 0.0)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]
        #print "%s"%self.static_transformStamped
        
        self.broadcaster.sendTransform(self.static_transformStamped)

    '''
        "/ur_pnp" Subscribe -> if) data == "search" -> TF Subscribe -> "cmd_pose" Publish
    '''
    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        print "Subscribed '/ur_pnp' -> '%s'"%msg.data
        if msg.data == 'search':
            # Update the Target Frame Name (임시로 rosparam 이용해 target_frame_name 수정 - SMACH 구성 후 수정 필요)
            self.target_frame_name = rospy.get_param("snu_object_tracker/target_frame")
            try:
                print "Trying to search the target: %s ..."%self.object_frame_name
                self.listener.waitForTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(), rospy.Duration(0.5))
                (trans,rot) = self.listener.lookupTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(0))

                self.update_cmd_pose(trans, rot)
                self.pub1.publish(self.cmd_pose)
                print(self.cmd_pose)

            except (tf.Exception):
                print "[ERROR]: The Target(TF) is not Detected !!!"
                pass
            return 1
        else:
            return -1

    
if __name__ == "__main__":
    ObjectTracker = snu_object_tracker()

    #test = String()
    #test.data = '1'
    #ar.pnp_cb(test)

    while not rospy.is_shutdown():
        #ar.pnp_cb(test)
        pass

    print 'good bye!'
