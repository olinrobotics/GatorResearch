#!/usr/bin/env python
# read_odometer.py

from simple_udp import SimplePublisher
from std_msgs.msg import Float64
import rospy


def main():
    port = 5022
    name = 'odometer'
    rate = 30  # Hz
    rospy.init_node('read_odometer', anonymous=False)
    publisher = rospy.Publisher(name, Float64, queue_size=5)
    odometer = SimplePublisher(port, name, publisher, rate, ">d")
    odometer.go()

if __name__ == '__main__':
    main()
