#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import TopicPublisher
from std_msgs.msg import Header
import rospy


def main():
    port = 5018
    name = 'heartbeat'
    rate = 2  # Hz
    rospy.init_node('read_heartbeat', anonymous=False)
    publisher = rospy.Publisher(name, Header, queue_size=3)
    heartbeat = TopicPublisher(port, name, publisher, rate)
    heartbeat.go()

if __name__ == '__main__':
    main()
