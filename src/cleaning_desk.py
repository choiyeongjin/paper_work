#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/cleaning_pkg/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *
from pytesseract import *

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


class DRLInterface():
    def __init__(self, ros_node_name="snu_drl_commander"):
        self.dsr_flag = None
        self.joints_state = None
        self.robot_status = "waiting"
        self.target_pose = Pose()
        self.drl_pose = Q_TOP_PLATE
        self.eulerZYZ = np.zeros(3)
        self.cmd_protocol = ACTION_HOME

        self.toolforce          = []
        self.toolforce_max      = 0.0
        self.toolforce_max_flag = False
        
        self.offset_x           = 0.0
        self.offset_y           = -0.12
        self.offset_z           = 0.2
        self.robvelj            = 30
        self.robaccj            = 30
        self.robvelx            = 50
        self.robaccx            = 50
        
        ##opencv drawing related
        self.imagewindowflag =0
        self.bridge = CvBridge()
        
        rospy.init_node(ros_node_name, anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("ur_pnp", String, self.pnp_cb, queue_size=1)
        rospy.Subscriber("dsr/state", RobotState, self.dsr_state_cb, queue_size=1)
        rospy.Subscriber("dsr/joint_states", JointState, self.current_status_cb, queue_size=1)
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)
        self.pnp_pub    = rospy.Publisher("ur_pnp", String, queue_size=1)
        self.status_pub = rospy.Publisher("ur_status", URStatus, queue_size=1)
        self.vision_pub = rospy.Publisher("vision_2d_flag",Int32, queue_size=1)

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        rospy.sleep(1)
        self.robot_status = "working"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
        # movej(Q_TOP_PLATE, 50, 50)
        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))

        
    '''
        dsr_state_cb: "~/dsr/state" topic callback function (update dsr_flag)
    '''
    def dsr_state_cb(self, msg):
        self.dsr_flag = msg.robot_state
        self.current_posx = msg.current_posx
        self.toolforce = msg.actual_ett
        # print(self.toolforce[0], self.toolforce[1], self.toolforce[2])
        # print(self.current_posx)

        ## Initialize Maximum tool force
        if not self.toolforce_max_flag:
            self.toolforce_max = self.toolforce[2]
            self.toolforce_max_flag = True
        
        ## Capture Maximum tool force
        if self.toolforce_max < self.toolforce[2]:
            self.toolforce_max = self.toolforce[2]
            # print(self.toolforce_max)
        

    '''
        current_status_cb: update "~/ur_status" from "~/dsr/joint_state"
    '''
    def current_status_cb(self, data):
        self.joints_state = data


    '''
        vision_cb: OpenCV image visualization
    '''
    def vision_cb(self, data):
        # pass
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.draw_image = copy.deepcopy(self.cv_image)
            self.specimen_image = copy.deepcopy(self.draw_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)
        # if self.imagewindowflag ==0:
        #     cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
        #     cv2.imshow('robot endeffector image', self.cv_image)
        #     cv2.waitKey(1)
        # elif self.imagewindowflag ==1:
        #     cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
        #     cv2.imshow('robot endeffector image', self.draw_image)
        #     cv2.waitKey(1)


    '''
        update_cmd_pose: update 'target_pose' to feed 'movel' function for Doosan-robot
            @ input 1: geometry_msgs/Vector3 trans
            @ input 2: geometry_msgs/Quaternion rot
    '''
    def update_cmd_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]
        print(self.target_pose)


    '''
        updateEulZYZ: Calculate ZYZ rotation to feed 'movel' function for Doosan-robot
    '''
    def updateEulZYZ(self):
        q_w = self.target_pose.orientation.w
        q_x = self.target_pose.orientation.x
        q_y = self.target_pose.orientation.y
        q_z = self.target_pose.orientation.z
        t1 = math.atan2(q_x, q_y)
        t2 = math.atan2(q_z, q_w)
        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(q_w*q_w + q_z*q_z))
        z2 = t2 + t1  
        self.eulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        print('The Euler angles are calculated:', self.eulerZYZ)


    '''
        search_ar_target: lookupTransform to get AR_Target (calculated from AR_Marker)
            @ input 1: int ar_tag_number (ex - 0, 1, 2, 3, ...)
    '''
    def search_ar_target(self, ar_tag_number):
        target_frame_name = 'ar_target_' + str(ar_tag_number)
        reference_frame_name = 'base_0'
        print "Searching AR tag ..."
        print("Target frame: "    + target_frame_name)
        print("Reference frame: " + reference_frame_name)
        listener = tf.TransformListener()
        try:
            print "Trying to search the target: %s ..."%target_frame_name
            self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            self.update_cmd_pose(trans, rot)
            self.updateEulZYZ()
            self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z ,self.eulerZYZ[0], self.eulerZYZ[1], self.eulerZYZ[2]))
            print('Target DRL Pose: ' , self.drl_pose)
        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            pass


    '''
        UpdateParam: Updating parameters for target pose w.r.t. AR_Marker
            @ input 1: double dx [m]
            @ input 2: double dy [m]
            @ input 3: double dz [m]
    '''
    def UpdateParam(self, dx, dy, dz):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.sleep(2)
    

    '''
        Doosan-robot Relative Move (translation in x, y, z [mm])
            @ input 1: double distance [mm]
            @ input 2: intArray velx = [50, 10] [mm/s, mm/s]
            @ input 3: intArray accx = [50, 10] [mm/s^2, mm/s^2]
    '''
    def movel_x(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_y(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_z(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def move_xyz(self, x, y, z, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(x, y, z, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_xyzjoint(self,x,y,z,jz1,jy,jz2,velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm], angle [degree]
        movel(posx(x, y, z, jz1, jy, jz2), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)


    '''
        setVelAcc: Set Doosan-robot Velocity(joint, task), Acceleration(joint, task)
            @ input 1: int velj

    '''
    def setVelAcc(self, velj=DSR_DEFAULT_VELJ, accj=DSR_DEFAULT_ACCJ, velx=DSR_DEFAULT_VELX, accx=DSR_DEFAULT_ACCX):
        set_velj(velj)
        set_accj(accj)
        set_velx(velx[0], velx[1])
        set_accx(accx[0], accx[1])


    def traj_gen(self, position):
        radius = 150
        gripper_size = 50
        r_step = np.round(radius / gripper_size)
        path = []
        path.append([position[0], position[1], position[2], 0, 180, 0])
        if (r_step <= 1):
            path.append([position[0], position[1] + 50, position[2], 0, 180, 0])
            path.append([position[0] + 50, position[1], position[2], 0, 180, 0])
            path.append([position[0], position[1] - 50, position[2], 0, 180, 0])
            path.append([position[0] - 50, position[1], position[2], 0, 180, 0])
        else:
            dev = 16
            for i in range(r_step):
                for j in range(dev):
                    path.append(
                        [position[0] + np.sin(2 * np.pi * j / dev) * gripper_size * (2 * (i + (j + 0.0) / dev) + 1) / 2,
                         position[1] + np.cos(2 * np.pi * j / dev) * gripper_size * (2 * (i + (j + 0.0) / dev) + 1) / 2,
                         position[2], 0, 180, 0])
        return path

    def IO_init(self):
        for i in range(1, 16+1):
            set_digital_output(i, 0)
        rospy.sleep(0.5)

    def rotate_on(self):
        pin = ACTION_IO_ROTATE
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def rotate_off(self):
        pin = ACTION_IO_ROTATE
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)

    
    '''
        "~/ur_pnp" Topic Protocol (for Doosan-robot control)
    '''
    def pnp_cb(self, msg):
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        release_compliance_ctrl()
        self.robot_status = "running"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))


        #######################################################################################################################################################


        movej(Q_TOP_PLATE, 50, 50)  # Search pose
        # TO ATTACH FABRIC TO EE
        self.UpdateParam(0.0, -0.12, 0.20)
        self.search_ar_target(5)  # Set AR TAG Number
        if not self.drl_pose[0] ==0:
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 1st approach
            self.search_ar_target(5)
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 2nd approach
            self.search_ar_target(5)
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 3rd approach
            self.search_ar_target(5)
        else:
            print("AR Tag not founded")
        fab_pose = deepcopy(self.drl_pose)
        fab_pose[2] = 75 # Height of table (need to be calculated)

        self.setVelAcc(50, 50, [50, 100], [50, 100])
        task_compliance_ctrl([500, 4500, 4000, 1000, 1000, 1000])
        movel(fab_pose)
        release_compliance_ctrl()

        self.setVelAcc(100, 100, [400, 100], [400, 100])

        # TO GET AR TAG POSE(TARGET)
        movej(Q_TOP_PLATE, 50, 50)  # Search pose
        self.UpdateParam(0.0, -0.12, 0.20)
        self.search_ar_target(6)  # Set AR TAG Number
        if not self.drl_pose[0] == 0:
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 1st approach
            self.search_ar_target(6)
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 2nd approach
            self.search_ar_target(6)
            movel(self.drl_pose, vel=[100, 50], acc=[100, 50])  # 3rd approach
            self.search_ar_target(6)
        else:
            print("AR Tag not founded")

        traject = self.traj_gen(self.drl_pose)
        p_1 = [1,1,1,1,1,1] # STARTING POINT / If starting point is center of AR Tag, delete it.
        movel(p_1)
        task_compliance_ctrl([500, 4500, 4000, 1000, 1000, 1000])
        self.rotate_on()
        for i in range(np.size(traject,0)):  #Moving path
            movel(posx(traject[i][0], traject[i][1], 75, 0, 180, 0), vel=[100,50], acc=[100,50])  # z-coordinates needs to be calculated
        self.rotate_off()
        release_compliance_ctrl()
        movel(end_point)







        ########################################################################################################################################################
        set_robot_mode(ROBOT_MODE_MANUAL)
        release_compliance_ctrl()  
        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
    
        

if __name__=='__main__':
    idim = DRLInterface("snu_drl_commander")
    
    while not rospy.is_shutdown():
        pass
        if(idim.dsr_flag == 2):
            idim.robot_status = "running"
        #elif(idim.dsr_flag == 1 and idim.robot_status == "running"):
        #    idim.robot_status = "done"
        #else:
        #    idim.robot_status = "waiting"
        idim.status_pub.publish(URStatus(status=idim.robot_status, arm_status = idim.joints_state))
        rospy.sleep(0.1)
    # set_robot_mode(ROBOT_MODE_MANUAL)
    
