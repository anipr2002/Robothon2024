#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os

def callback(msg):
    command = "rosrun " + msg.data
    rospy.loginfo(f"Executing: {command}")
    os.system(command)

def listener():
    rospy.init_node('rosrun_launcher', anonymous=True)
    rospy.Subscriber("/rosrun", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
