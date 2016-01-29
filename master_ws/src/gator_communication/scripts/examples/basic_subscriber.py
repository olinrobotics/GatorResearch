#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " | I heard %s", data)

def listener():

    rospy.init_node('listener', anonymous=True)
    doug = rospy.Subscriber("/debugtest", DiagnosticArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()