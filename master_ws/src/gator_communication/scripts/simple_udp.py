import rospy
import traceback
from StringIO import StringIO
import struct

from UDP import UDPtoROS
from UDP import ROStoUDPonDemand
import constants as cs


class SimplePublisher(UDPtoROS):
    """
    This is a simple example of how to use UDPtoROS. This works to unpack a
    single value from a transmitted UDP stream and publish it to a ROS topic.
    """

    def __init__(self, port, name, publisher, rate_hz, decode_str):
        """
        Args:
            port (int): Local port on which to receive incoming UDP data. This
                should be unique among all local UDP receivers and transmitters.
            name (str): A human-readable name for this publisher to aid in 
                monitoring and debugging.
            publisher (rospy.Publisher): Configured ROS publisher on which the 
                received data will be broadcast.
            rate_hz (num): The rate, in Hertz, at which this topic will be
                published on the specified publisher.
            decode_str (str): String used by struct.unpack to unpack single
                value from received UDP string.
        """
        socket_config = {
            # TODO: Tie ip to ros parameter or environment variable.
            "source_ip": "192.168.2.10",
            "local_port": port,
            "name": name,
            "blocking": True,
            "timeout": 0.5
        }
        super(SimplePublisher, self).__init__(socket_config, 4096)
        self.pub = publisher
        self.rate = rospy.Rate(rate_hz)
        self.decode_str = decode_str

    def broadcast(self, data):
        rospy.logdebug("Data Recieved: {d}".format(d=data))
        if data["data"]:
            res = struct.unpack(self.decode_str, data["data"])
            self.pub.publish(res[0])
        else:
            res = None
        self.rate.sleep()

    def go(self):
        """
        Runs the receiving and publishing loops, handling exceptions by
        shutting down gracefully.
        """
        try:
            self.run()
        except Exception:
            traceback.print_exc()
            self.receiver.stop()


class TopicPublisher(UDPtoROS):
    """
    Uses UDPtoROS to interpret an incoming UDP string which has already been
    formatted as a ROS message. The type of message interpreted from UDP is
    determined by the message type specified by the ROS publisher.
    """

    def __init__(self, port, name, publisher, rate_hz):
        """
        Args:
            port (int): Local port on which to receive incoming UDP data. This
                should be unique among all local UDP receivers and transmitters.
            name (str): A human-readable name for this publisher to aid in 
                monitoring and debugging.
            publisher (rospy.Publisher): Configured ROS publisher on which the 
                received data will be broadcast.
            rate_hz (num): The rate, in Hertz, at which this topic will be
                published on the specified publisher.
        """
        socket_config = {
            # TODO: Tie ip to ros parameter or environment variable.
            "source_ip": "192.168.2.10",
            "local_port": port,
            "name": name,
            "blocking": True,
            "timeout": 0.5
        }
        super(TopicPublisher, self).__init__(socket_config, 4096)
        self.pub = publisher
        self.rate = rospy.Rate(rate_hz)
        self._data_class = self.pub.data_class()
        self.decoder = self._data_class.deserialize

    def broadcast(self, data):
        rospy.logdebug("Data Recieved: {d}".format(d=data))
        if data["data"]:
            res = self.decoder(data["data"])
            self.pub.publish(res)
        else:
            res = None
        self.rate.sleep()

    def go(self):
        """
        Runs the receiving and publishing loops, handling exceptions by
        shutting down gracefully.
        """
        try:
            self.run()
        except Exception:
            traceback.print_exc()
            self.receiver.stop()


class OnDemandSubscriber(ROStoUDPonDemand):
    """docstring for OnDemandSubscriber"""

    def __init__(self, port, name, destination):
        socket_config = {
            # TODO: Tie ip to ros parameter or environment variable.
            "source_ip": "192.168.2.10",
            "local_port": port,
            "name": name,
        }
        super(OnDemandSubscriber, self).__init__(socket_config, destination)
        self._sub = None
        self._data_class = None
        self.serializer = None

    @property
    def sub(self):
        return self._sub

    @sub.setter
    def sub(self, new_sub):
        # TODO: Check type of new_sub
        if isinstance(new_sub, rospy.Subscriber):
            self._sub = new_sub
            self._data_class = new_sub.data_class()
            self.serializer = self._data_class.serialize
        else:
            rospy.logerr(
                "OnDemandSubscriber {n} has been given an invalid subscriber."
                " It may not broadcast.").format(n=self.name)

    def callback(self, data):
        ser_buff = StringIO()
        data.serialize(ser_buff)
        self.send(ser_buff.getvalue())
        ser_buff.close()

    def go(self):
        rospy.spin()


class CmdVelSubscriber(ROStoUDPonDemand):
    """docstring for CmdVelSubscriber"""

    def __init__(self, port, name, destination):
        socket_config = {
            # TODO: Tie ip to ros parameter or environment variable.
            "source_ip": "192.168.2.10",
            "local_port": port,
            "name": name,
        }
        super(CmdVelSubscriber, self).__init__(socket_config, destination)
        self._sub = None

    @property
    def sub(self):
        return self._sub

    @sub.setter
    def sub(self, new_sub):
        # TODO: Check type of new_sub
        if isinstance(new_sub, rospy.Subscriber):
            self._sub = new_sub
        else:
            rospy.logerr(
                "CmdVelSubscriber {n} has been given an invalid subscriber."
                " It may not broadcast.").format(n=self.name)

    def callback(self, data):
        angle_data = data.angular.z
        vel_data = data.linear.x
        send_str = struct.pack(
            '>{}dd'.format(cs.trans_type),
            cs.trans_prefix,
            vel_data,
            angle_data)
        self.send(send_str)

    def go(self):
        rospy.spin()
