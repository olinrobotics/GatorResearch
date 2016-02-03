#!/usr/bin/env python
# send_cmd_vel.py

from simple_udp import OnDemandSubscriber
from geometry_msgs.msg import Twist
import rospy


def main():
    local_port = 6015
    destination = {
        "address": "192.168.1.11",
        "port": 6015
    }
    name = 'cmd_vel'
    rospy.init_node('send_cmd_vel', anonymous=False)
    cmd_vel = OnDemandSubscriber(local_port, name, destination)
    cmd_vel.sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel.callback)
    cmd_vel.go()

if __name__ == '__main__':
    main()
