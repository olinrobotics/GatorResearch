#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def fake_imu():
    pub = rospy.Publisher('IMU_ned', Imu, queue_size=10)
    rospy.init_node('test_convertimu', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    
    while not rospy.is_shutdown():
        imu_message=Imu()

        imu_message.header.stamp=rospy.Time.now()
        imu_message.header.frame_id="IMU"

        imu_message.orientation.x=1
        imu_message.orientation.y=2
        imu_message.orientation.z=3
        imu_message.orientation.w=4
        imu_message.orientation_covariance=[-1,0,0,0,-1,0,0,0,-1]

        imu_message.linear_acceleration.x=6
        imu_message.linear_acceleration.y=7
        imu_message.linear_acceleration.z=8
        imu_message.linear_acceleration_covariance=[1e6, 0,0,0,1e6,0,0,0,1e6]

        imu_message.angular_velocity.x=10
        imu_message.angular_velocity.y=11
        imu_message.angular_velocity.z=12
        imu_message.angular_velocity_covariance=[1e6, 0,0,0,1e6,0,0,0,1e6]

        pub.publish(imu_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_imu()
    except rospy.ROSInterruptException:
        pass