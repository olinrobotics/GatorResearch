import rospy
from geographic_msgs.srv import GetGeographicMap

rospy.init()
get_map = rospy.ServiceProxy('Get_Geographic_Map', GetGeographicMap)
