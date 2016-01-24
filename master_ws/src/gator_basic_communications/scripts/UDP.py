import abc
import rospy
import socket
from threading import Thread


class UDPThread(Thread):
    """
    Subclass of threading.Thread. This monitors incoming packets in a separate
    thread and asychronously updates self.data as packets are recieved.
    """

    def __init__(self, conf_socket, buf_size=1024):
        """
        Args:
            conf_socket (socket.socket()): A configured python (UDP) socket.
            buf_size (int): Integer specifying the maximum amount of data to be
                            received at once (in bytes).
        """
        super(UDPThread, self).__init__()
        # Coninue flag. If true, the main thread loop continues to execute
        self.cont = True
        # The most recent data recieved
        self.data = {"data": None, "addr": None, "error_count": 0}
        self.sock = conf_socket  # A configured python socket
        # The maximum amount of data to be received at once (in bytes).
        self.buf_size = buf_size
        self.error_threshold = 5  # After five consecutive socket errors, we
        # declare this socket 'dead' and reset its data to None.

    def run(self):
        rec_buf_size = self.buf_size
        while (self.cont):
            try:
                # buffer size is 1024 bytes
                data, addr = self.sock.recvfrom(rec_buf_size)
            except socket.error:
                # Catches errors which occur when no new data is recieved
                # before timeout or when non-blocking call made, etc.
                self.data["error_count"] += 1
                if self.data["error_count"] >= self.error_threshold:
                    self.data["data"] = None
                    self.data["addr"] = None
                pass
            else:
                self.data = {'data': data, 'addr': addr, 'error_count': 0}

    def stop(self):
        self.cont = False


class UDPReciever(object):
    """
    Object to manage the multithreaded reception of UDP packets.
    """

    def __init__(self, source_ip, local_port, name=None, blocking=True, timeout=5):
        """
        Args:
            source_ip (str): IP address of the UDP packets to listen for.
            local_port (int): Local port number to listen for UDP packets at.
            name (Optional[str]): Human-readable name for identification or
                debugging purposes. Defaults to None.
            blocking (Optional[bool]): Optional boolean specifying whether 
                socket recieve call should be blocking [True] or 
                non-blocking [False].Defaults to True.
            timeout (Optional[number]): Optional numerical value which specifies
                the timeout (in seconds) of a blocking socket recieve call.
                Set to 0 (or 0.0 or None) for an infinitely blocking call.
                Defaults to 5.
        """
        super(UDPReciever, self).__init__()
        self.ip = source_ip
        self.port = local_port
        self.sock = self._create_socket(blocking, timeout)
        self.live = UDPThread(self.sock)

    def _create_socket(self, blocking, timeout):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.ip, self.port))
        if not blocking:
            # Make socket non-blocking
            sock.setblocking(False)
        else:
            # If blocking, set timeout
            # This line shouldn't be necessary. Default behavior is blocking
            sock.setblocking(True)
            sock.settimeout(timeout)
        return sock

    def run(self):
        self.live.start()

    def stop(self):
        self.live.stop()
        self.live.join()


class UDPtoROS(object):
    """
    Abstract base class for asynchronously publishing ROS topics from incoming
    streams of UDP data. Subclasses must define a broadcast() method which takes
    the UDP data and publishes it to a ROS topic(s). Subclasses must create this
    publisher(s), initialize a node, and call the UDPtoROS __init__ function
    (via super()) in it's __init__() function. The broadcast() method is run
    inside a while loop--the subclass's implementation of broadcast() should
    include some means of limiting the rate of this loop.

    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, socket_config):
        """
        Args:
            socket_config (dict): Dictionary of socket configuration values.
                The following keys must be included as specified--

                "source_ip" (str): IP address of the UDP packets to listen for.
                "local_port" (int): Local port number to listen for UDP packets at.
                "name" (Optional[str]): Human-readable name for identification
                    or debugging purposes. Defaults to None.
                "blocking" (Optional[bool]): Optional boolean specifying whether 
                    socket recieve call should be blocking [True] or 
                    non-blocking [False]. Defaults to True.
                "timeout" (Optional[number]): Optional numerical value which 
                    specifies the timeout (in seconds) of a blocking socket 
                    recieve call. Set to 0 (or 0.0 or None) for an infinitely
                    blocking call. Defaults to 0.5.
        """
        super(UDPtoROS, self).__init__()
        # Set default socket configuration values if needed.
        if "blocking" not in socket_config.keys():
            socket_config["blocking"] = True
        if "timeout" not in socket_config.keys():
            socket_config["timeout"] = 0.5
        if "name" not in socket_config.keys():
            socket_config["name"] = None

        self.reciever = UDPReciever(
            socket_config["source_ip"],
            socket_config["local_port"],
            blocking=socket_config["blocking"],
            timeout=socket_config["timeout"]
        )
        self.socket_conf = socket_config

        rospy.loginfo("Creating UDP Reciever \"{n}\" on port {p} listening to {a}.".format(
            p=socket_config["local_port"],
            a=socket_config["source_ip"],
            n=socket_config["name"] if socket_config["name"] else "<anonymous>"
        )
        )

    @abc.abstractmethod
    def broadcast(self, data):
        """
        Abstract method which should recieve a dictionary of the recieved data
        and should publish to a ROS topic(s). This method must also include a
        rate.sleep() or similar rate-limiting call. This should be non-blocking.
        """
        pass

    def run(self):
        self.reciever.run()
        while not rospy.is_shutdown():
            current_data = self.reciever.live.data
            err = self.broadcast(current_data)
            if err == -1:
                break
        self.reciever.stop()
        rospy.loginfo("Stopping UDP Reciever \"{n}\" on port {p} listening to {a}.".format(
            p=self.socket_conf["local_port"],
            a=self.socket_conf["source_ip"],
            n=self.socket_conf["name"] if self.socket_conf[
                "name"] else "<anonymous>"
        )
        )


def main():
    pass

if __name__ == '__main__':
    main()
