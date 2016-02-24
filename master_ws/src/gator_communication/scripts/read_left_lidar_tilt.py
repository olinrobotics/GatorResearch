#!/usr/bin/env python
# read_left_lidar_tilt.py
from simple_udp import SimplePublisher
from std_msgs.msg import Float32
import rospy


def main():
    port = 5030
    name = 'tilt_angle'
    rate = 50  # Hz
    rospy.init_node('read_left_lidar_tilt', anonymous=False)
    publisher = rospy.Publisher(name, Float32, queue_size=5)
    steer_angles = SimplePublisher(port, name, publisher, rate, ">f")
    steer_angles.go()

if __name__ == '__main__':
    main()
