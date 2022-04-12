#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import sys
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import message_filters

def callback(data1, data2, data3, data4):
    pub = rospy.Publisher("/cmd_vel", String, queue_size=1)
    rospy.loginfo( "I heard %s %s %s %s", data1.data,data2.data,data3.data,data4.data)
    if data1.data ==0:
        hello_str = "%s"%data3.data
        # 打印字符串
        rospy.loginfo(hello_str)
        # 利用发布器发布话题
        pub.publish(hello_str)




def listener():
    rospy.init_node('listener', anonymous=True)

    t1= message_filters.Subscriber("lidar_judge", Int8)
    t2 =message_filters.Subscriber("cam_judge", Int8)
    t3 = message_filters.Subscriber("chatter1", String)
    t4 = message_filters.Subscriber("chatter2", String)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2, t3, t4], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()