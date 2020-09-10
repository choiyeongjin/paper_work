#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, os, numpy as np
import moveit_commander
import timeit
from math import pi
from time import sleep
from copy import deepcopy

import moveit_msgs.msg
from std_msgs.msg import String, Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from syscon_msgs.msg import URStatus
from dsr_msgs.msg import RobotState
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import *

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

TASK_POINTING  = 'pointing'
TASK_ARM_PICK  = '3.0'
TASK_ARM_PLACE = '4.0'
PICK_3DP = '3dp'

# Some of Contstants
DISTANCE_AWAY_FROM_TARGET = 0.2

EPSILON = 0.0000001
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
M2MM = 1000.0
MM2M = 1.0/1000.0

ROLL = 180.0 * DEG2RAD
PITCH = 0.0 * DEG2RAD
YAW = 0.0 * DEG2RAD

Q0 = [0.0*DEG2RAD,    0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD]
Q1 = [90.0*DEG2RAD,   0.0*DEG2RAD,    -90*DEG2RAD,      0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD]
Q2 = [-90.0*DEG2RAD,  0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD]
Q3 = [0.0*DEG2RAD,    15.0*DEG2RAD,   -70.0*DEG2RAD,    0.0*DEG2RAD,    -125.0*DEG2RAD,   0.0*DEG2RAD]
Q4 = [0.0*DEG2RAD,    0.0*DEG2RAD,    -90.0*DEG2RAD,    0.0*DEG2RAD,    -90.0*DEG2RAD,    180.0*DEG2RAD]
Q5 = [-5.993828130492754e-05, 0.3384384062003861, -1.574394656901861, 9.300767452392647e-05, -1.9054921594259837, 0.00014209506981415032]
Q_SEARCH_RIGHT = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT  = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]

#Q_SEARCH_RIGHT = [-0.021912608043065707, 0.3745233068645807, -2.515318099008636, -0.0016689710660107685, -0.9671584417422292, 0.00014467861565142695]


def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('snu_moveit_commander', anonymous=True)

    group_name = "arm"
    #reference_frame = "/base_link"
    reference_frame = "base_0"
    

    self.joints_state = None
    self.robotus = 'waiting'
    self.dsr_flag = None
    self.start_pose = Pose()
    self.target_pose = Pose()
    self.artag = AlvarMarkers()
    self.target_pose_dsr = [-600.0, 0.0, 600.0, -180.0, 0.0, 180.0]
    
    self.dsr_status = rospy.Publisher('ur_status', URStatus, queue_size=1)
    self.pnp_pub = rospy.Publisher('ur_pnp', String, queue_size=1)

    rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
    rospy.Subscriber('cmd_moveit', Pose, self.cmd_moveit_cb, queue_size=1)
    rospy.Subscriber('dsr/state', RobotState, self.dsr_state_cb, queue_size=1)
    rospy.Subscriber('dsr/joint_states',JointState, self.current_status_cb, queue_size=1)

    self.robot = moveit_commander.RobotCommander(robot_description="dsr/robot_description", ns="dsr") # outer-level interface to the robot
    self.scene = moveit_commander.PlanningSceneInterface(ns="dsr") # world surrounding the robot
    self.group = moveit_commander.MoveGroupCommander(group_name, robot_description="dsr/robot_description", ns="dsr") # interface to one group of joints (plan & execute)
    self.group.set_pose_reference_frame(reference_frame)
    
    #self.default_joint_states = self.group.get_current_joint_values()
    self.default_joint_states = Q5

    # Allow replanning to increase the odds of a solution
    self.group.allow_replanning(True) 
    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.1)
    self.group.set_planning_time(0.1)
    self.group.set_max_acceleration_scaling_factor(0.5)
    self.group.set_max_velocity_scaling_factor(0.65)

    # We can also print the name of the end-effector link for this group:
    self.end_effector_link = self.group.get_end_effector_link()
   # print "============ End effector: %s" % self.end_effector_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()

   # print "============ Moveit_Commander Setup Initialized !!!"
   # print "Robot Pose Initialized!"
    self.group.set_joint_value_target(self.default_joint_states)
    self.group.set_start_state_to_current_state()
    plan = self.group.plan()
    self.robotus = "running"
    self.dsr_status.publish(URStatus(status=self.robotus, arm_status = self.joints_state))
    self.group.execute(plan)
    self.robotus = "done"
    self.dsr_status.publish(URStatus(status=self.robotus, arm_status = self.joints_state))
    self.robotus = "waiting"

##############################################################################################################
  def dsr_state_cb(self, data):
    self.dsr_flag = data.robot_state

  def current_status_cb(self, data):
	  self.joints_state = data

  def cmd_moveit_cb(self, msg):
    self.start_pose  = self.group.get_current_pose(self.end_effector_link).pose
    self.target_pose = self.start_pose
    
    self.target_pose.position.x = msg.position.x
    self.target_pose.position.y = msg.position.y
    self.target_pose.position.z = msg.position.z
    self.target_pose.orientation.x = msg.orientation.x
    self.target_pose.orientation.y = msg.orientation.y
    self.target_pose.orientation.z = msg.orientation.z
    self.target_pose.orientation.w = msg.orientation.w
    
    self.Pose_to_DSR()
    ##q1 = []
    ##q1.append(self.target_pose.orientation.x)
    ##q1.append(self.target_pose.orientation.y)
    ##q1.append(self.target_pose.orientation.z)
    ##q1.append(self.target_pose.orientation.w)
    ##
    ##q2 = []
    ##q2.append(msg.orientation.x)
    ##q2.append(msg.orientation.y)
    ##q2.append(msg.orientation.z)
    ##q2.append(msg.orientation.w)
    ##
    ##q3 = [] 
    ##q3 = quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))
    ##self.target_pose.orientation.x = q3[0]
    ##self.target_pose.orientation.y = q3[1]
    ##self.target_pose.orientation.z = q3[2]
    ##self.target_pose.orientation.w = q3[3]
  
  def Pose_to_DSR(self):

    pos = [self.target_pose.position.x *M2MM, self.target_pose.position.y *M2MM, self.target_pose.position.z *M2MM]
    q = (self.target_pose.orientation.x, self.target_pose.orientation.y, self.target_pose.orientation.z, self.target_pose.orientation.w)
    euler = euler_from_quaternion(q)
    rot =  [euler[0] * RAD2DEG, euler[1] * RAD2DEG, euler[2] * RAD2DEG]

    self.target_pose_dsr = [pos[0], pos[1], pos[2], rot[2], rot[1], rot[0]]
    print(self.target_pose_dsr)
    print(self.target_pose_dsr)
    print(self.target_pose_dsr)

  def wait_for_complete_motion(self):
    rospy.sleep(1)
    while self.dsr_flag == 2:
      print "Robot is Running!"

  def joints_plan_and_execute(self, joints):
    self.group.stop()
    self.group.clear_pose_targets()
    self.default_joint_states = joints
    self.group.set_joint_value_target(self.default_joint_states)
    self.group.set_start_state_to_current_state()
    plan = self.group.plan()
    #self.group.aysncExecute(plan)
    self.group.execute(plan, wait=False)

  def List_to_Pose(self, list):
    pose = Pose()
    pose.position.x    = list[0]
    pose.position.y    = list[1]
    pose.position.z    = list[2]
    pose.orientation.x = list[3]
    pose.orientation.y = list[4]
    pose.orientation.z = list[5]
    pose.orientation.w = list[6]
    return pose

  def moveit_joint_cmd(self, target_joint):
    self.group.set_joint_value_target(target_joint)
    self.group.set_start_state_to_current_state()
    plan = self.group.plan()
    self.group.execute(plan)

  def pnp_cb(self,msg):
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    print(msg.data)
    self.start_flag = msg.data
    ### Init Pose ###
    if(self.start_flag=="0.0"):
      self.moveit_joint_cmd(Q0)

    ### Left Swing ###
    if(self.start_flag=="1.0"):
      self.moveit_joint_cmd(Q1)

    ### Right Swing ###
    if(self.start_flag=="-1.0"):
      self.moveit_joint_cmd(Q2)

    ### Pointing the Object (Target pose) ###
    if(self.start_flag==TASK_POINTING):
      self.moveit_joint_cmd(Q_SEARCH_FRONT)
      self.wait_for_complete_motion()
      
      self.pnp_pub.publish('search')
      self.wait_for_complete_motion()

      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      waypoints.append(deepcopy(self.target_pose))
      self.group.set_start_state_to_current_state()
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()

      self.moveit_joint_cmd(Q5)
      self.wait_for_complete_motion()

    ### Picking the 3D Printer Workpiece ###
    if(self.start_flag == PICK_3DP):
      self.moveit_joint_cmd(Q_SEARCH_FRONT)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('open')
      self.wait_for_complete_motion()
      
      self.pnp_pub.publish('search')
      self.wait_for_complete_motion()

      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      self.target_pose.position.x += 0.2
      waypoints.append(deepcopy(self.target_pose))
      self.target_pose.position.x -= 0.2
      waypoints.append(deepcopy(self.target_pose))
      self.group.set_start_state_to_current_state()
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('close')
      self.wait_for_complete_motion()
      
      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      self.start_pose.position.z += 0.05
      waypoints.append(deepcopy(self.start_pose))
      self.start_pose.position.x += 0.25
      waypoints.append(deepcopy(self.start_pose))
      self.group.set_start_state_to_current_state()
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()

      self.moveit_joint_cmd(Q5)
      self.wait_for_complete_motion()

      #movel(self.target_pose_dsr, vel=30, acc=60, ref=DR_BASE)


    ### Picking the object in the right side ###
    if(self.start_flag == TASK_ARM_PICK):
      self.moveit_joint_cmd(Q_SEARCH_RIGHT)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('open')
      self.wait_for_complete_motion()
      
      self.pnp_pub.publish('search')
      self.wait_for_complete_motion()

      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      self.target_pose.position.z += 0.05
      waypoints.append(deepcopy(self.target_pose))
      self.target_pose.position.z -= 0.05
      waypoints.append(deepcopy(self.target_pose))
      self.group.set_start_state_to_current_state()
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('close')
      self.wait_for_complete_motion()

      self.moveit_joint_cmd(Q5)
      self.wait_for_complete_motion()

    ### Placing the object to the target in the right side ###
    if(self.start_flag == TASK_ARM_PLACE):
      self.moveit_joint_cmd(Q_SEARCH_RIGHT)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('close')
      self.wait_for_complete_motion()

      
      self.pnp_pub.publish('search')
      self.wait_for_complete_motion()

      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      self.target_pose.position.z += 0.10
      waypoints.append(deepcopy(self.target_pose))
      self.target_pose.position.z -= 0.07
      waypoints.append(deepcopy(self.target_pose))    
      self.group.set_start_state_to_current_state()
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()

      self.pnp_pub.publish('open')
      self.wait_for_complete_motion()

      waypoints = []
      self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
      waypoints.append(deepcopy(self.start_pose))
      self.start_pose.position.z += 0.05
      waypoints.append(deepcopy(self.start_pose))
      plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
      if 1-fraction < 0.2:
         self.group.execute(plan)
      self.wait_for_complete_motion()
      
      self.moveit_joint_cmd(Q5)
      self.wait_for_complete_motion()
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    
  

##############################################################################################################

if __name__=='__main__':
  mp=MoveGroupPythonInteface()
  while not rospy.is_shutdown():
    if(mp.dsr_flag == 2):
      mp.robotus = "running"
    elif(mp.robotus == "running" and mp.dsr_flag == 1):
      mp.robotus = "done"
    else:
			mp.robotus = "waiting"
    mp.dsr_status.publish(URStatus(status=mp.robotus, arm_status = mp.joints_state))
    rospy.sleep(0.1)