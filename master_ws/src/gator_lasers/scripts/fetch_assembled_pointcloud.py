#!/usr/bin/env python
import roslib
roslib.load_manifest('laser_assembler')
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud, PointCloud2


def main():
    rospy.wait_for_service("assemble_scans")
    if rospy.get_param("pc2", False):
        assemble_scans = rospy.ServiceProxy(
            'assemble_scans2', AssembleScans2)
        pub_dtype = PointCloud2
    else:
        assemble_scans = rospy.ServiceProxy(
            'assemble_scans', AssembleScans)
        pub_dtype = PointCloud

    pub = rospy.Publisher('assembled', pub_dtype, queue_size=10)
    rospy.init_node("assembled_pointcloud_fetcher")
    rate = rospy.Rate(1.0)  # 1 Hz.

    prev_time = rospy.get_rostime()

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        try:
            # Change to AssembleScans1 for sensor_msgs/PointCloud
            resp = assemble_scans(prev_time, current_time)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        else:
            rospy.logdebug("Received assembled pointcloud with {} points.".format(resp.cloud.points))
            pub.publish(resp.cloud)
        prev_time = current_time
        rate.sleep()

if __name__ == "__main__":
    main()
