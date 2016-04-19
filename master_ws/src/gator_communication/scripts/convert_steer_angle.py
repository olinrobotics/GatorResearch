#!/usr/bin/env python

from std_msgs.msg import Float64
from math import radians
import rospy


class SteerConversion(object):

    def __init__(self):
        super(SteerConversion, self).__init__()
        rospy.init_node('convert_steer_angle', anonymous=True)
        self.pub = rospy.Publisher('steer_angle_rad', Float64, queue_size=10)
        self.sub = rospy.Subscriber('steer_angle', Float64, self.convert_angle)

    def convert_angle(self, in_angle):
        """
        Converts incoming steering wheel angle (in degrees) to steer angle
        (of the front wheels) in radians. Publishes this new value to
        self.pub.
        """
        # According to Midbrain.VI, steering wheel angle is converted to
        # an orientation angle by dividing by 10 degrees.
        steer_angle_deg = -(in_angle.data / 16.99)

        # Convert from degrees to radians
        out_angle = Float64(radians(steer_angle_deg))

        self.pub.publish(out_angle)

    def go(self):
        rospy.spin()

if __name__ == "__main__":
    converter = SteerConversion()
    converter.go()
