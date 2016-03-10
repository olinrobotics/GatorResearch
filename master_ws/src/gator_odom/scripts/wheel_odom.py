#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32, Float64

from math import radians, sin, cos, tan


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

    def __init__(self, pub_odom, wheel_dist_topic, wheel_dist_type,
                 steer_angle_topic, steer_angle_type, frame_id, child_frame_id):
        super(WheelOdometryNode, self).__init__()
        self.pub = pub_odom
        self.wheel_sub = rospy.Subscriber(
            wheel_dist_topic,
            wheel_dist_type,
            self.cb_wheel
        )
        self.angle_sub = rospy.Subscriber(
            steer_angle_topic,
            steer_angle_type,
            self.cb_steer
        )

        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        self.wheel_data = None
        self.steer_data = None

        # Assume starting at the origin & vehicle facing +x direction
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0  # Theta measured from +x axis towards +y axis

        self.NEW_DATA_PAIR = False

    @publish_on_new
    def cb_wheel(self, data):
        self.wheel_data = data

    @publish_on_new
    def cb_steer(self, data):
        self.steer_data = data

    def compute_odom(self):
        L = rospy.get_param('vehicle_length')  # Len from rear diff to fr. axle
        steer_rad = radians(self.steer_data)
        d_dr = self.wheel_data
        d_theta = d_dr/float(L) * tan(steer_rad)
        dx = d_dr * cos(self.theta_est + d_theta/2)
        dy = d_dr * sin(self.theta_est + d_theta/2)
        self.x_est += dx
        self.y_est += dy
        self.theta_est += d_theta

    def publish_odom_msg(self):
        self.compute_odom()
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position = Point(self.x_est, self.y_est, 0)  # z = 0
        msg.pose.pose.orientation = Quaternion()

        self.pub.publish()


def main():
    "/wheel_vel", Float32

if __name__ == '__main__':
    main()
