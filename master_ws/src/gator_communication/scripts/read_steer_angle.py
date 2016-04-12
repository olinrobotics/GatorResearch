#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import SimplePublisher
from std_msgs.msg import Float64
import rospy


def main():
    port = 5016
    name = 'steer_angle'
    rate = 30 # Hz
    rospy.init_node('read_steer_angle', anonymous=False)
    publisher = rospy.Publisher(name, Float64, queue_size=5)
    steer_angles = SimplePublisher(port, name, publisher, rate, ">d")
    steer_angles.go()

if __name__ == '__main__':
    main()
