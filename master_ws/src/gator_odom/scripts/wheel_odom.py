#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64


def publish_on_new(func):
    def func_wrapper(self, data):
        func(self, data)  # Call func() to update instance attributes
        if self.NEW_DATA_PAIR:
            self.publish_odom_msg()
            self.NEW_DATA_PAIR = False
        else:
            self.NEW_DATA_PAIR = True
    return func_wrapper


class WheelOdometryNode(object):
    """docstring for WheelOdometryNode"""

    def __init__(self, pub_odom, wheel_speed_topic, wheel_speed_type,
                 steer_angle_topic, steer_angle_type):
        super(WheelOdometryNode, self).__init__()
        self.pub = pub_odom
        self.wheel_sub = rospy.Subscriber(
            wheel_speed_topic,
            wheel_speed_type,
            self.cb_wheel
        )
        self.angle_sub = rospy.Subscriber(
            steer_angle_topic,
            steer_angle_type,
            self.cb_steer
        )
        self.wheel_data = None
        self.steer_data = None

        self.NEW_DATA_PAIR = False

    @publish_on_new
    def cb_wheel(self, data):
        self.wheel_data = data

    @publish_on_new
    def cb_steer(self, data):
        self.steer_data = data


def main():
    "/wheel_vel", Float32

if __name__ == '__main__':
    main()
