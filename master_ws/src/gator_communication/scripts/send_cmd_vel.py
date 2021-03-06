#!/usr/bin/env python
# send_cmd_vel.py

from simple_udp import CmdVelSubscriber
from geometry_msgs.msg import Twist
import rospy


def main():
    local_port = 60015
    destination = {
        "address": "192.168.2.11",
        "port": 60015
    }
    name = 'cmd_vel'
    rospy.init_node('send_cmd_vel', anonymous=False)
    cmd_vel = CmdVelSubscriber(local_port, name, destination)
    cmd_vel.sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel.callback)
    cmd_vel.go()

if __name__ == '__main__':
    main()
