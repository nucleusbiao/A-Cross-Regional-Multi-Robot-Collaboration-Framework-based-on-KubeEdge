#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import roslib
import rospy
import random
import actionlib
import tf.transformations

import os
import time
import sys
import paho.mqtt.client as mqtt
import json
import threading
import multiprocessing
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class PatrolNav():
    def __init__(self):

        rospy.init_node('patrol_nav_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.rest_time = rospy.get_param("~rest_time", 5)
        self.keep_patrol = rospy.get_param("~keep_patrol", False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type = rospy.get_param("~patrol_type", 0)
        self.patrol_loop = rospy.get_param("~patrol_loop", 2)
        self.patrol_time = rospy.get_param("~patrol_time", 5)
        self.x = 0
        self.y = 0

        while not rospy.is_shutdown():
            self.server_main()
            if self.x != 0 or self.y != 0:            #x and y are the x-axis coordinates and y-axis coordinates of the robot respectively。They are designated by the cloud
                a = self.x
                b = self.y
            else:
                a = 0
                b = 0
            self.locations = dict()
            
            self.locations['one'] = Pose(Point(a, b, 0), Quaternion(0, 0, 0, 1))
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.move_base.wait_for_server(rospy.Duration(6000))
            
    
            n_successes = 0
            target_num = 0
            location = ""
            start_time = rospy.Time.now()
            locations_cnt = len(self.locations)
            sequeue = ['one']
            #sequeue = ['one', 'two', 'three', 'four', 'five', 'six']
    
            #rospy.loginfo("Starting position navigation ")

        #while not rospy.is_shutdown():

            location = sequeue[target_num]

            #rospy.loginfo("target_num_value:" + str(target_num))

            #rospy.loginfo("Going to: " + str(location))
            self.send_goal(location)

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60000))
            if not finished_within_time:
               self.move_base.cancel_goal()
               rospy.logerr("ERROR:Timed out achieving goal")
            else:
               state = self.move_base.get_state()

            target_num += 1
            if target_num > 0:
                target_num = 0

    def send_goal(self, locate):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal)  # send goal to move_base

    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

    def on_connect(self,client, userdata, flags, rc):
        #print('connected to mqtt with resurt code ', rc)
        client.subscribe("$hw/events/device/" + "ros1" + "/twin/update/delta")  #mqtt topic   Create in the master node of kubeedge

    def on_message(self,client, userdata, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.x = float(payload['delta']['xposition'])
        self.y = float(payload['delta']['yposition'])

    def server_conenet(self,client):
        client_id = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        client = mqtt.Client(client_id)
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect("192.168.8.114", 1883, 60)                      #mqtt broker ip
        client.loop_start()
        # client.loop_forever()

    def server_main(self):
        client_id = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        client = mqtt.Client(client_id, transport='tcp')
        self.server_conenet(client)


if __name__ == '__main__':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")



       

