#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import SimplePublisher
from std_msgs.msg import Bool
import rospy


def main():
    port = 5018
    name = 'heartbeat'
    rate = 2  # Hz
    rospy.init_node('read_heartbeat', anonymous=False)
    publisher = rospy.Publisher(name, Bool, queue_size=3)
    heartbeat = SimplePublisher(port, name, publisher, rate)
    heartbeat.go()

if __name__ == '__main__':
    main()
