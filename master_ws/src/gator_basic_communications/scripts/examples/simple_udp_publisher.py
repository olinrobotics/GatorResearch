#!/usr/bin/env python
# simple_udp_publisher.py
import rospy
from UDP import UDPtoROS

from std_msgs.msg import String


class OnePublisher(UDPtoROS):
    """
    This is a simple example of how to use UDPtoROS.
    """

    def __init__(self):
        rospy.init_node('test_node', anonymous=True)
        socket_config = {
            "source_ip": "192.168.1.10",
            "local_port": 5015,
            "name": "test_receiver",
            "blocking": False,
        }
        super(OnePublisher, self).__init__(socket_config)
        self.pub = rospy.Publisher('test', String, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz

    def broadcast(self, data):
        rospy.logdebug("Data Recieved: {d}".format(d=data))
        self.pub.publish(data["data"])
        self.rate.sleep()


def main():
    pubobj = OnePublisher()
    pubobj.run()


if __name__ == '__main__':
    main()
