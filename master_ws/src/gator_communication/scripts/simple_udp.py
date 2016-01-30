import rospy
from UDP import UDPtoROS
import traceback


class SimplePublisher(UDPtoROS):
    """This is a simple example of how to use UDPtoROS """

    def __init__(self, port, name, publisher, rate_hz):
        """
        Args:
            port (TYPE): Description
            name (TYPE): Description
            publisher (TYPE): Description
            rate_hz (TYPE): Description
        """
        socket_config = {
            # TODO: Tie ip to ros parameter or environment variable.
            "source_ip": "192.168.1.10",
            "local_port": port,
            "name": name,
            "blocking": True,
            "timeout": 0.5
        }
        super(SimplePublisher, self).__init__(socket_config)
        self.pub = publisher
        self._data_class = self.pub.data_class()
        self.decoder = self._data_class.deserialize
        self.rate = rospy.Rate(rate_hz)

    def broadcast(self, data):
        rospy.logdebug("Data Recieved: {d}".format(d=data))
        if data["data"]:
            try:
                res = self.decoder(data["data"])
            except TypeError:
                # Occurs when a ROS topic such as Header expects fields
                # but receives None when no messages have been received.
                res = None
        else:
            res = None
        self.pub.publish(res)
        self.rate.sleep()

    def go(self):
        try:
            self.run()
        except Exception:
            traceback.print_exc()
            self.receiver.stop()


class SimpleHeaderPublisher(SimplePublisher):
    """
    Receives and publishes a header message from LabVIEW.
    """

    def __init__(self, port, name, publisher, rate_hz):
        super(SimpleHeaderPublisher, self).__init__(
            port, name, publisher, rate_hz)

    def broadcast(self, data):
        rospy.logdebug("Data Recieved: {d}".format(d=data))
        if data["data"]:
            res = self.decoder(data["data"])
        else:
            res = self._data_class
        self.pub.publish(res)
        self.rate.sleep()
