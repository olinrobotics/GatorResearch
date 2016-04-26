#!/usr/bin/env python
# read_gps.py

from simple_udp import TopicPublisher
from sensor_msgs.msg import PointCloud
import rospy


def main():
    port = 5033
    name = 'lidar_pointcloud'
    rate = 2  # Hz
    rospy.init_node('raw_pc', anonymous=False)
    publisher = rospy.Publisher(name, PointCloud, queue_size=3)
    lidar_pc = TopicPublisher(port, name, publisher, rate)
    lidar_pc.go()

if __name__ == '__main__':
    main()
