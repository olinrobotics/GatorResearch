#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from std_msgs.msg import Float32, Float64

import tf
from tf import transformations

from math import radians, sin, cos, tan
from time import clock


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
    """
    Publishes wheel odometry based on rear wheel distance data and steering wheel angle data.
    Publishes after new rear wheel data AND new steer angle data received.
    """

    def __init__(self, pub_odom, wheel_vel_topic, wheel_vel_type,
                 steer_angle_topic, steer_angle_type, frame_id, child_frame_id, broadcast_tf):
        """
            Args:
                pub_odom (rospy.Publisher()): Configured ROS odometry publisher
                wheel_vel_topic (str): Name of a topic publishing wheel
                    velocity information in meters per second.
                wheel_vel_type (msg type): Data type of a topic publishing
                    wheel velocity information in meters per second.
                steer_angle_topic (str): Name of a topic publishing steering
                     angle information
                steer_angle_type (msg type): Data of a topic publishing
                    steering angle information
                frame_id (str): Name of the wheel odometry frame.
                child_frame_id (str): Name of the wheel odometry's child frame.
                broadcast_tf (bool): If True, this node will also publish a transform
                    from frame_id to child_frame_id.
        """
        super(WheelOdometryNode, self).__init__()
        self.broadcast_tf = broadcast_tf
        self.pub = pub_odom
        self.tf_br = None
        if broadcast_tf:
            self.tf_br = tf.TransformBroadcaster()
        self.wheel_sub = rospy.Subscriber(
            wheel_vel_topic,
            wheel_vel_type,
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

        self.vehicle_length = float(rospy.get_param('vehicle_length'))  # Len. from rear diff to fr. axle

        # Assume starting at the origin & vehicle facing +x direction
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0  # Theta measured from +x axis towards +y axis

        self.vx_calc = 0  # Linear velocity in x direction
        self.vy_calc = 0  # Linear velocity in y direction

        self.az_calc = 0  # Angular velocity around Z axis

        self._prev_time = None

        self.NEW_DATA_PAIR = False

    @publish_on_new
    def cb_wheel(self, data):
        self.wheel_data = data

    @publish_on_new
    def cb_steer(self, data):
        self.steer_data = data

    def compute_odom(self):
        current_time = clock()
        steer_rad = radians(self.steer_data)
        d_dr = self.wheel_data
        d_theta = d_dr/self.vehicle_length * tan(steer_rad)
        dx = d_dr * cos(self.theta_est + d_theta/2)
        dy = d_dr * sin(self.theta_est + d_theta/2)

        self.x_est += dx
        self.y_est += dy
        self.theta_est += d_theta
        if self._prev_time is None:
            self._prev_time = current_time
            return -1

        dt = float(current_time - self._prev_time)
        self.vx_calc = dx/dt
        self.vy_calc = dy/dt

        self.az_calc = d_theta/dt
        self._prev_time = current_time
        return 1

    def publish_odom_msg(self):
        publish = self.compute_odom()
        if publish is -1:  # Does not publish first reading, which may have inaccurate computed velocities.
            return
        quat_rot = transformations.quaternion_from_euler(0, 0, self.theta_est, 'sxyz')
        cur_time = rospy.Time.now()
        if self.broadcast_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = cur_time
            tf_msg.header.frame_id = "wheel_odom"
            tf_msg.child_frame_id = "wheel_odom_bl"
            tf_msg.transform.translation.x = self.x_est
            tf_msg.transform.translation.y = self.y_est
            tf_msg.transform.translation.z = self.z_est
            tf_msg.transform.rotation = self.quat_rot
            self.tf_br.sendTransform(tf_msg)
        msg = Odometry()
        msg.header.stamp = cur_time
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position = Point(self.x_est, self.y_est, 0)  # z = 0
        msg.pose.pose.orientation = quat_rot
        msg.pose.covariance = [1e-3, 0, 0, 0, 0, 0,  # 1e-3 is TurtleBot
                               0, 1e-3, 0, 0, 0, 0,  # default wheel odom
                               0, 0, 1e-3, 0, 0, 0,  # covariances.
                               0, 0, 0, 1e-3, 0, 0,  # Tune if necessary.
                               0, 0, 0, 0, 1e-3, 0,  # UMBmark?
                               0, 0, 0, 0, 0, 1e-3]

        msg.twist.twist.linear = Vector3(self.vx_calc, self.vy_calc, 0)  # z-velocity = 0
        msg.twist.twist.angular = Vector3(0, 0, self.az_calc)  # No x- or y- angular velocities
        msg.twist.covariance = [1e-3, 0, 0, 0, 0, 0,  # 1e-3 is TurtleBot
                                0, 1e-3, 0, 0, 0, 0,  # default wheel odom
                                0, 0, 1e-3, 0, 0, 0,  # covariances.
                                0, 0, 0, 1e-3, 0, 0,  # Tune if necessary.
                                0, 0, 0, 0, 1e-3, 0,  # UMBmark?
                                0, 0, 0, 0, 0, 1e-3] 

        self.pub.publish()

    def go(self):
        rospy.spin()

def main():
    pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
    wheel_topic = "/sensors/vehicle_state/wheel_vel"
    wheel_type = Float64

    steer_topic = "/sensors/vehicle_state/steer_angle"
    steer_type = Float64
    rospy.init_node('wheel_odom', anonymous=True)
    odom = WheelOdometryNode(pub, wheel_topic, wheel_type, steer_topic, steer_type, "wheel_odom", "wheel_odom_bl", False)
    odom.go()

if __name__ == '__main__':
    main()
