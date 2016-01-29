#!/usr/bin/env python
# read_steer_angle.py

from simple_udps import SimplePublisher
from std_msgs.msg import Float64
import traceback
import rospy


def main():
    port = 5017
    name = 'steer_angle2'
    rate = 2  # Hz
    rospy.init_node('second_node', anonymous=False)
    publisher = rospy.Publisher(name, Float64, queue_size=5)
    steer_angles = SimplePublisher(port, name, publisher, rate)
    try:
        steer_angles.run()
    except Exception:
        traceback.print_exc()
        steer_angles.reciever.stop()


if __name__ == '__main__':
    main()
