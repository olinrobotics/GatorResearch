#!/usr/bin/env python
# read_wheel_vel.py

from simple_udp import SimplePublisher
from std_msgs.msg import Float32
import rospy


def main():
    port = 5015
    name = 'wheel_vel'
    rate = 30  # Hz
    rospy.init_node('read_wheel_vel', anonymous=False)
    publisher = rospy.Publisher(name, Float32, queue_size=5)
    wheel_velocity = SimplePublisher(port, name, publisher, rate, ">f")
    wheel_velocity.go()

if __name__ == '__main__':
    main()
