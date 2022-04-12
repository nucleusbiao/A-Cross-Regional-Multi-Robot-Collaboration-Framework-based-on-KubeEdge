#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import sys
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import message_filters

from sensor_msgs.msg import Image, CameraInfo

import math
import numpy as np
# aruco
import cv2
from cv2 import aruco

import time

import tf
from tf.transformations import quaternion_matrix
from cv_bridge import CvBridge

bridge = CvBridge()

def Intrinsics_camera():
    color_info = rospy.wait_for_message('/usb_cam/camera_info', CameraInfo)
    print('Common information has get finished!!!')
    K = color_info.K
    D = color_info.D
    fx = K[0]
    cx = K[2]
    fy = K[4]
    cy = K[5]

    distCoe = np.array([D[0], D[1], D[2], D[3], D[4]])
    cameraMat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    return cameraMat, distCoe


def aruco_detecter(size):
    ''' detect aruco marker use robot cam'''

    cameraMatrix, distCoeffs = Intrinsics_camera()

    font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)  字体

    # 选择aruco模块中预定义的字典来创建一个字典对象
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # cv2.aruco.DICT_ARUCO_ORIGINAL
    parameters = aruco.DetectorParameters_create()

    data = rospy.wait_for_message('/usb_cam/image_raw', Image)

    # from message to img
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    # BGR -> RAY
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # lists of ids and the corners beloning to each id // numpy.ndarray, list
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              aruco_dict,
                                                              parameters=parameters)
    # Display the resulting frame
    cv2.imshow("zed", frame)
    d = 0
    h = 1
    if ids is not None:
            # 标记相对于相机框架的旋转, 平移
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, size, cameraMatrix, distCoeffs)

        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        print(tvec)

        deg = rvec[0][0][2] / math.pi * 180
        # 旋转矩阵到欧拉角
        R1 = np.zeros((3, 3), dtype=np.float64)
        cv2.Rodrigues(rvec, R1)
        sy = math.sqrt(R1[0, 0] * R1[0, 0] + R1[1, 0] * R1[1, 0])
        singular = sy < 1e-6
        if not singular:  # 偏航，俯仰，滚动
            x = math.atan2(R1[2, 1], R1[2, 2])
            y = math.atan2(-R1[2, 0], sy)
            z = math.atan2(R1[1, 0], R1[0, 0])
        else:
            x = math.atan2(-R1[1, 2], R1[1, 1])
            y = math.atan2(-R1[2, 0], sy)
            z = 0
        # 偏航，俯仰，滚动换成角度
        rx = x * 180.0 / 3.141592653589793
        ry = y * 180.0 / 3.141592653589793
        rz = z * 180.0 / 3.141592653589793
        print('deg_z:' + str(ry) + str('deg'))
        R, _ = cv2.Rodrigues(rvec)
        t = tvec[0].T
        TT = np.hstack((R, t))
        TT = np.vstack((TT, np.array([0, 0, 0, 1])))
        # print(TT)
        # transition_mat = np.array([[1, 0, 0, 0],          # ???????????????
        #                           [0, 1, 0, -0.12],
        #                           [0, 0, 0, 0],
        #                           [0, 0, 0, 1]])
        # final_mat = np.dot(TT, transition_mat)
        # final = final_mat[:, 3]
        # aruco_cam_fra = matrix([[final[0]], [final[1]], [final[2]],[1]])
        TT = np.linalg.inv(TT)
        print('aruco_to_camera')
        print(TT[0, 3] + 0.0)

        ###### 距离估计 #####
        distance = ((tvec + 0.025) * 0.0105) * 100  # 单位是米

        #
        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec[i, :, :], tvec[i, :, :], 0.15)
            aruco.drawDetectedMarkers(frame, corners)
        print("horizontal direction: %.3f m" % distance[0][0][2])
        print("vertical direction:%.3f m" % (TT[0, 3] + 0.0))
        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (10, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, 'deg_z:' + str(ry) + str('deg'), (0, 170), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "horizontal direction: %.3f m" % distance[0][0][2], (0, 110), font, 1, (255, 0, 0), 2,
                        cv2.LINE_AA)
        cv2.putText(frame, "vertical direction:%.3f m" % (TT[0, 3] + 0.0), (0, 140), font, 1, (255, 0, 0), 2,
                        cv2.LINE_AA)

        d = TT[0, 3] + 0.0
        #d = math.fabs(d)
        h = distance[0][0][2]
        return d,h
    else:
        # draw "NO IDS"
        cv2.putText(frame, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

def callback(data1, data3):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    if data1.data == 0:
        twist = Twist()
        twist.linear.x = data3.linear.x
        twist.angular.z = data3.angular.z
        # 打印字符串
        #rospy.loginfo(twist)
        # 利用发布器发布话题
        pub.publish(twist)


    if data1.data == 1:
        d,h = aruco_detecter(size=0.1)
        twist = Twist()
        # a = d / h
        b = h - 0.7

        if 0 < d:
            num1 = int(d / 0.05)
            if num1 != 0:
                for i in range(1, 6):
                    twist.linear.x = 0
                    twist.angular.z = 0.4
                    pub.publish(twist)
                    time.sleep(2)
                    print "turn left"

                for n in range(0, num1):
                    twist.linear.x = 0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"

                for i in range(1, 6):
                    twist.linear.x = 0
                    twist.angular.z = -0.4
                    pub.publish(twist)
                    time.sleep(2)
                    print "turn right"

            if 0 < b:
                num2 = int(b / 0.05)
                print ("num2:" + str(num2))
                for n in range(0, num2):
                    twist.linear.x = 0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"
            else:
                b = math.fabs(b)
                num2 = int(b / 0.05)
                print ("num2:" + str(num2))
                for n in range(0, num2):
                    twist.linear.x = -0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"

        else:
            d = math.fabs(d)
            num1 = int(d / 0.05)
            if num1 != 0:
                for i in range(1, 6):
                    twist.linear.x = 0
                    twist.angular.z = -0.4
                    pub.publish(twist)
                    time.sleep(2)
                    print "turn right"

                for n in range(0, num1):
                    twist.linear.x = 0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"

                for i in range(1, 6):
                    twist.linear.x = 0
                    twist.angular.z = 0.4
                    pub.publish(twist)
                    time.sleep(2)
                    print "turn lift"

            if 0 < b:
                num2 = int(b / 0.05)
                print ("num2:" + str(num2))
                for n in range(0, num2):
                    twist.linear.x = 0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"
            else:
                b = math.fabs(b)
                num2 = int(b / 0.05)
                print ("num2:" + str(num2))
                for n in range(0, num2):
                    twist.linear.x = -0.05
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(2)
                    print "Go straight"
        rospy.signal_shutdown("closed!")


def listener():
    rospy.init_node('listener', anonymous=True)
    t1 = message_filters.Subscriber("lidar_judge", Int8)
    #t2 = message_filters.Subscriber("cam_judge", Int8)
    t3 = message_filters.Subscriber("nav_vel", Twist)
    #t4 = message_filters.Subscriber("cam_vel", Twist)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t3], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()