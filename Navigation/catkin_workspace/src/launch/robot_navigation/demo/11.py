#! /usr/bin/env python
# -*- coding: utf-8 -*-

# import sys
# sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy
from sensor_msgs.msg import Image, CameraInfo

import math
import numpy as np
# aruco
import cv2
from cv2 import aruco
import tf
from tf.transformations import quaternion_matrix
from cv_bridge import CvBridge
bridge = CvBridge()


def Intrinsics_camera():
    
    color_info = rospy.wait_for_message('/camera/camera_info', CameraInfo)
    print('Common information has get finished!!!')
    K = color_info.K
    D = color_info.D
    fx = K[0]
    cx = K[2]
    fy = K[4]
    cy = K[5]

    distCoe = np.array( [D[0], D[1], D[2], D[3], D[4]] )
    cameraMat = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])

    return cameraMat, distCoe



def aruco_detecter(size):
    ''' detect aruco marker use robot cam'''
    cameraMatrix, distCoeffs = Intrinsics_camera()
    #print(cameraMatrix,distCoeffs)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)  字体

    # 选择aruco模块中预定义的字典来创建一个字典对象 
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  #cv2.aruco.DICT_ARUCO_ORIGINAL
    parameters =  aruco.DetectorParameters_create()  

    while True:
        data = rospy.wait_for_message('/camera/image_raw', Image)

        # from message to img
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        # BGR -> RAY
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        # lists of ids and the corners beloning to each id // numpy.ndarray, list
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, 
                                                            aruco_dict, 
                                                            parameters=parameters)
        if ids is not None:
            # 标记相对于相机框架的旋转, 平移  
            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, size, cameraMatrix, distCoeffs)
            #print("_:",_)
            #(rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            (rvec-tvec).any() # get rid of that nasty numpy value array error  
            #print("rvec:",rvec)
            #print("tvec:",tvec)
            
            R, _ = cv2.Rodrigues(rvec)
            t = tvec[0].T
            TT = np.hstack((R, t)) 
            TT = np.vstack((TT, np.array([0, 0, 0, 1]))) 
            #print(TT)
            # transition_mat = np.array([[1, 0, 0, 0],          # ???????????????
            #                           [0, 1, 0, -0.12],
            #                           [0, 0, 0, 0],
            #                           [0, 0, 0, 1]])
            # final_mat = np.dot(TT, transition_mat)
            # final = final_mat[:, 3]
            # aruco_cam_fra = matrix([[final[0]], [final[1]], [final[2]],[1]])
            TT = np.linalg.inv(TT) 
            print('aruco_to_camera')
            print(TT)
            
            
            
            ###### 距离估计 #####
            distance = ((tvec + 0.025) * 0.0105) * 100  # 单位是米

            # 
            for i in range(rvec.shape[0]):
                aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec[i, :, :], tvec[i, :, :], 0.15)
                aruco.drawDetectedMarkers(frame, corners)

            ###### DRAW ID #####  
            cv2.putText(frame, "Id: " + str(ids), (10,50), font, 1, (0,255,0),2, cv2.LINE_AA)  
            cv2.putText(frame, "horizontal direction: %.3f m" % distance[0][0][2], (0, 110), font, 1, (255,0,0),2, cv2.LINE_AA)
            cv2.putText(frame, "vertical direction:%.3f m" % (TT[0,3]+0.0), (0, 140), font, 1, (255,0,0),2, cv2.LINE_AA)

        else:
            # draw "NO IDS" 
            cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)  
    
        # Display the resulting frame  
        cv2.imshow("zed",frame)
        #cv2.imshow("zed1", gray)
        # 键盘检测，检测到esc键退出
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break


        
if __name__ == "__main__":
    rospy.init_node('aruco_detection')
    aruco_detecter(size = 0.1)