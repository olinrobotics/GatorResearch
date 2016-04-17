#!/usr/bin/env python
# read_gps.py

from simple_udp import TopicPublisher
from sensor_msgs.msg import NavSatFix
import rospy


def main():
    port = 5021
    name = 'fix'
    rate = 1  # Hz
    rospy.init_node('read_gps', anonymous=False)
    publisher = rospy.Publisher(name, NavSatFix, queue_size=3)
    gps = TopicPublisher(port, name, publisher, rate)
    gps.go()

if __name__ == '__main__':
    main()
