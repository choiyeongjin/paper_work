#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################################################################################
'''
    Python Libraries
'''
import sys, os, threading, time
import timeit
import math
import numpy as np
import cv2 as cv2
import pyrealsense2 as rs
from copy import deepcopy
from math import pi
from time import sleep
import copy
##################################################################################################################################################


##################################################################################################################################################
'''
    ROS Libraries
'''
import rospy
import tf, tf2_ros
import moveit_commander
import moveit_msgs.msg
from syscon_msgs.msg import URStatus
from dsr_msgs.msg import RobotState
from tf.transformations import *
from std_msgs.msg import String, Header, Int32, Float32MultiArray
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Joy, Image
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
##################################################################################################################################################


##################################################################################################################################################
'''
    Doosan-Robot Libraries
'''
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/doosan-robot/common/imp"%HOME_DIR)) )
NS_           = "R_001"
ROBOT_ID_     = "dsr"
ROBOT_MODEL_  = ""
import DR_init
DR_init.__dsr__id = NS_+'/'+ROBOT_ID_
DR_init.__dsr__model = ROBOT_MODEL_
from DSR_ROBOT import *

##################################################################################################################################################