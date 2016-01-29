#!/usr/bin/env python

import rospy
from gator_basic_communications.msg import TestArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "| I heard:\n{data}".format(data=data))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/debugtest", TestArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()