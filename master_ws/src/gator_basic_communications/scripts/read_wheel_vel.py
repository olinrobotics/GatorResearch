#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import SimplePublisher
from std_msgs.msg import Float64
import traceback
import rospy


def main():
    port = 5016
    name = 'wheel_vel'
    rate = 30  # Hz
    rospy.init_node('read_wheel_vel', anonymous=False)
    publisher = rospy.Publisher(name, Float64, queue_size=5)
    wheel_velocity = SimplePublisher(port, name, publisher, rate)
    wheel_velocity.go()

if __name__ == '__main__':
    main()
