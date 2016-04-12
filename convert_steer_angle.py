#!/usr/bin/env python

from std_msgs.msg import Float64
import rospy


class SteerConversion(object):

    def __init__(self):
        super(SteerConversion, self).__init__()
        rospy.init_node('convert_steer_angle', anonymous=True)
        self.pub = rospy.Publisher('imu/data', Float64, queue_size=10)
        self.sub = rospy.Subscriber('IMU_ned', Float64, self.convert_angle)

    def convert_angle(self, in_angle):
        """
        Converts incoming steering wheel angle (in degrees) to steer angle
        (of the front wheels) in radians. Publishes this new value to
        self.pub.
        """

        # According to Midbrain.VI, steering wheel angle is converted to
        # an orientation angle by dividing by 10 degrees.
        steer_angle_deg = in_angle / 10.
        out_angle = Float64()

        self.pub.publish(out_angle)

    def go(self):
        rospy.spin()

if __name__ == "__main__":
    converter = SteerConversion()
    converter.go()
