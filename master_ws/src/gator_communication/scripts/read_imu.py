#!/usr/bin/env python
# read_steer_angle.py

from simple_udp import TopicPublisher
from sensor_msgs.msg import Imu
import rospy


def main():
    port = 5020
    name = 'IMU_ned'
    rate = 30  # Hz
    rospy.init_node('read_imu', anonymous=False)
    publisher = rospy.Publisher(name, Imu, queue_size=5)
    heartbeat = TopicPublisher(port, name, publisher, rate)
    heartbeat.go()

if __name__ == '__main__':
    main()
