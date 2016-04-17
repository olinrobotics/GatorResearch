#!/usr/bin/env python

from sensor_msgs.msg import Imu
from std_msgs.msg import String
import rospy

class PublishAndSubscribe:
    def __init__(self):
        rospy.init_node('convert_imu', anonymous=True)
        self.pub=rospy.Publisher('imu/data', Imu, queue_size=10)
        self.sub=rospy.Subscriber('IMU_ned', Imu, self.callback)

    def callback(self, data):
        imu_message=Imu()

        imu_message.header.stamp=rospy.Time.now()
        imu_message.header.frame_id="IMU"

        imu_message.orientation.x=data.orientation.y
        imu_message.orientation.y=data.orientation.x
        imu_message.orientation.z=-data.orientation.z
        imu_message.orientation.w=data.orientation.w
        imu_message.orientation_covariance=data.orientation_covariance

        imu_message.linear_acceleration.x=data.linear_acceleration.y
        imu_message.linear_acceleration.y=data.linear_acceleration.x
        imu_message.linear_acceleration.z=-data.linear_acceleration.z
        imu_message.linear_acceleration_covariance=data.linear_acceleration_covariance

        imu_message.angular_velocity.x=data.angular_velocity.y
        imu_message.angular_velocity.y=data.angular_velocity.x
        imu_message.angular_velocity.z=-data.angular_velocity.z
        imu_message.angular_velocity_covariance=data.linear_acceleration_covariance

        self.pub.publish(imu_message)


if __name__ == '__main__':
    PublishAndSubscribe()

    rospy.spin()