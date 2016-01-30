#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray


class Monitor(object):
    """Monitors the heartbeat connection to the Gator and handles reported errors."""

    def __init__(self, node, rate):
        super(Monitor, self).__init__()
        self.node = node
        rospy.init_node(self.node)
        self.heart_sub = rospy.Subscriber(
            "/heartbeat", DiagnosticArray, self.callback)
        self.rate = rate

    def run(self):
        r = rospy.Rate(self.rate)
        rospy.spin()

    def callback(self, data):
        """
        This currently prints the data received.
        # TODO Create error codes and a dispatcher for handling each as they come in.
        # TODO Handle loss of heartbeat.
        """
        rospy.loginfo(
            rospy.get_caller_id() + "| I heard:\n{data}".format(data=data))


if __name__ == '__main__':
    hbmon = Monitor("monitor", 2)
    hbmon.run()
