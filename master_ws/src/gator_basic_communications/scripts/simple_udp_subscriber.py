#!/usr/bin/env python
# simple_udp_subscriber.py
import rospy
from UDP import ROStoUDP

from std_msgs.msg import String


class OneSubscriber(ROStoUDP):
    """
    This is a simple example of how to use ROStoUDP.
    """

    def __init__(self, destination):
        rospy.init_node('test_node', anonymous=True)
        socket_config = {
            "source_ip": "192.168.1.10",
            "local_port": 5016,
            "name": "test_sender",
            "blocking": False,
        }
        super(OneSubscriber, self).__init__(socket_config, destination)
        self.pub = rospy.Subscriber("/debugtest", String, self.callback)

    def callback(self, data):
        """
        This is called upon receipt of a new message on the .sub object's
        subscribed topic. If there were any parsing and/or flattening of the
        received data necessary before transmission, it would also happen here.
        Args:
            data: A String message received from ROS.

        Returns:
            None
        """
        self.send_data = data

def main():
    destination = {
        "address": "192.168.1.11",
        "port": 5051
    }
    subobj = OneSubscriber(destination)
    subobj.run()


if __name__ == '__main__':
    main()
