#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *
from pytesseract import *

################  TO DO LIST  ################
# 1. organize parameter

################  TF NAME ################
CAMERA_FRAME_PREFIX_    = 'camera_link'
OBJECT_TARGET_PREFIX_   = 'specimen_table_'
TEMP_PREFIX_            = 'temp_'


class snu_vision_2d():
    def __init__(self, ros_node_name="snu_vision_2d"):
   
        rospy.init_node('ros_node_name', anonymous=True)
        self.bridge = CvBridge()      
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()
        self.cmd_pose = Pose()
        self.vision_status = "waiting"

        ### Topics to Subscribe ###
        self.flag_sub  = rospy.Subscriber("/R_001/vision_2d_flag", Int32, self.vision_flag, queue_size=1) # Trigger Topic
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw", Image, self.vision)
        self.imagewindowflag = False #False : image pass, True : image show for debugging

        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked

        print(self.vision_status)

    def vision(self, data):
        # pass
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.specimen_image = copy.deepcopy(self.cv_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)
        if self.imagewindowflag == False :
            pass
        elif self.imagewindowflag == True:
            cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
            cv2.imshow('robot endeffector image', self.draw_image)
            cv2.waitKey(1)


    '''
        Translation: (trans, rot) -> geometry_msgs/Pose
    '''
    def update_cmd_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1] # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]


    def updateEulZYZ(self):
        q_w = self.cmd_pose.orientation.w
        q_x = self.cmd_pose.orientation.x
        q_y = self.cmd_pose.orientation.y
        q_z = self.cmd_pose.orientation.z
        t1 = math.atan2(q_x, q_y)
        t2 = math.atan2(q_z, q_w)
        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(q_w*q_w + q_z*q_z))
        z2 = t2 + t1  
        self.eulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        print('The Euler angles are calculated:', self.eulerZYZ)

    '''
        Update ROS Parameters
    '''    
    def update_ros_param(self):
        pass

    '''
        TF : Static transform 
        TRANSLATION : X, Y, Z [mm] 
        ROTATION : Z1, Y, Z2 [radian]
    ''' 
    def tf_broadcaster(self, PARENT_ID, CHILD_ID, TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z, ROTATION_Z1, ROTATION_Y, ROTATION_Z2) :
        self.static_transformStamped.header.frame_id = PARENT_ID
        self.static_transformStamped.child_frame_id  = CHILD_ID
        self.static_transformStamped.transform.translation.x = MM2M(TRANSLATION_X)
        self.static_transformStamped.transform.translation.y = MM2M(TRANSLATION_Y)
        self.static_transformStamped.transform.translation.z = MM2M(TRANSLATION_Z)
        quat = tf.transformations.quaternion_from_euler(ROTATION_Z1, ROTATION_Y, ROTATION_Z2)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(self.static_transformStamped)

    def vision_flag(self, msg):
        self.vision_status = "running"
        self.vision_protocol = msg.data
        # self.update_ros_param()
        print(self.vision_protocol)

        ########################################################################
        # VISION [30001] : SEARCH SPECIMEN (CANNY -> CONTOUR)
        if self.vision_protocol == TWOD_VISION_SEARCH_SPECIMEN :
            
            #Set Region of Interest
            rowEnd=616
            colEnd=402
            rowStart=241 #224
            colStart=24

            #Offset from camera to endeffector
            cam_offsetx = 112
            cam_offsety = 36

            obj_count=1

            bgr_temp = self.specimen_image
            gray_temp=cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)

            gray=gray_temp[colStart:colEnd, rowStart:rowEnd]
            bgr=bgr_temp[colStart:colEnd, rowStart:rowEnd]

            #Canny edge detection & Hough lines transform
            edges=cv2.Canny(gray,50,200)
            # cv2.imshow('Canny', edges)
            # cv2.waitKey(1000)
            _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            
            for ii in range(len(contours)):
                ptAccum=np.squeeze(contours[ii])

                # FILTER1 : the specimen edge should contain more than 300 points
                if len(ptAccum) <500: 
                    print('bad search : point shortage')
                    continue

                x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
                x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
                y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
                y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

                #find four rectnagular Vertices using maximum coordinate
                x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
                x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
                y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
                y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]

                # FILTER2
                print(ptAccum[x_Max,0], ptAccum[x_Min,0], ptAccum[y_Max,1], ptAccum[y_Min,1])            

                orientation1= float(x_Max_Vertice[1]-x_Min_Vertice[1])/float(x_Max_Vertice[0]-x_Min_Vertice[0])
                orientation2= float(y_Max_Vertice[1]-y_Min_Vertice[1])/float(y_Max_Vertice[0]-y_Min_Vertice[0])
                orientation=(orientation1+orientation2)/2.0
                theta=math.atan(orientation)

                #centroid : average of all coordinates
                centroid=[float(sum(ptAccum[:,0])/float(len(ptAccum[:,0]))), float(sum(ptAccum[:,1]))/float(len(ptAccum[:,1]))]

                #plotting for debugging 
                cv2.circle(bgr, (int(centroid[0]), int(centroid[1])),2,(0,0,255),4)
                cv2.circle(bgr, (int(y_Max_Vertice[0]), int(y_Max_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(x_Min_Vertice[0]), int(x_Min_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(y_Min_Vertice[0]), int(y_Min_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(x_Max_Vertice[0]), int(x_Max_Vertice[1])),1,(0,255,255),2)
                # cv2.imshow('image', bgr)
                # cv2.waitKey(30)

                #Calibration 150mm / 277.24pixels
                px2mm_Row=(centroid[0]+rowStart-320)*150/277.24
                px2mm_Col=(centroid[1]+colStart-240)*150/277.24
                
                if centroid is not None:
                    self.tf_broadcaster(CAMERA_FRAME_PREFIX_, TEMP_PREFIX_ + str(obj_count), 320.0, -px2mm_Row+18.0, -px2mm_Col-8.0, 0, np.pi/2, np.pi)
                    self.tf_broadcaster(TEMP_PREFIX_, (OBJECT_TARGET_PREFIX_ + str(obj_count)), 0.0, 0.0, 0.0, 0.0, 0.0, -theta)
                    obj_count = obj_count+1

        self.vision_status = "vision processing complete"
        print(self.vision_status)

        ###############################################################################################################3
    
if __name__ == "__main__":
    twod_vision = snu_vision_2d()

    while not rospy.is_shutdown():
        pass

    print 'good bye!'
