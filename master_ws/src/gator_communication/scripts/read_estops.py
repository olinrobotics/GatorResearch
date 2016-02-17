#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import SimplePublisher
from std_msgs.msg import Bool
import rospy


def main():
    port = 5017
    name = 'estop'
    rate = 30  # Hz
    rospy.init_node('read_estops', anonymous=False)
    publisher = rospy.Publisher(name, Bool, queue_size=5)
    e_stops = SimplePublisher(port, name, publisher, rate , ">?")
    e_stops.go()

if __name__ == '__main__':
    main()
