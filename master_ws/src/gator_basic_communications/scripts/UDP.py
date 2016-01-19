import abc
import rospy
import socket
from threading import Thread


class UDPThread(Thread):
    """
    Abstract base subclass of threading.Thread.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, conf_socket):
        """
        Args:
            conf_socket (socket.socket()): A configured python (UDP) socket.
            buf_size (int): Integer specifying the maximum amount of data to be
                            received at once (in bytes).
        """
        super(UDPThread, self).__init__()
        self.cont = True  # Continue flag. If true, the main thread loop
        # continues to execute
        self.sock = conf_socket  # A configured python socket

    @abc.abstractmethod
    def run(self):
        pass

    def stop(self):
        self.cont = False


class UDPSendThread(UDPThread):
    """
    Subclass of UDPThread used for periodically sending UDP packets to a set
    address and port pair.
    """

    def __init__(self, conf_socket, destination, period):
        super(UDPSendThread, self).__init__(conf_socket)
        self.dest_addr = destination["address"]
        self.dest_port = destination["port"]
        self.data = ""  # Contains the string of data to send.
        self.period = period  # In seconds, how often to transmit data.

    def run(self):
        while self.cont:
            self.sock.sendto(self.data, (self.dest_addr, self.dest_port))
            time.sleep(self.period)


class UDPReceiveThread(UDPThread):
    """
    Subclass of UDPThread used for receiving UDP packets. This
    implements a run() method which monitors incoming packets in a separate
    thread and asynchronously updates a .data attribute as packets are received.
    """

    def __init__(self, conf_socket, buf_size=1024):
        super(UDPReceiveThread, self).__init__(conf_socket)
        self.buf_size = buf_size  # The maximum amount of data to be received
        #  at once (in bytes).
        self.data = {"data": None, "addr": None}  # The most recent data
        # received

    def run(self):
        rec_buf_size = self.buf_size
        while self.cont:
            try:
                data, addr = self.sock.recvfrom(rec_buf_size)
            except socket.error, e:
                # Catches errors which occur when no new data is recieved
                # before timeout or when non-blocking call made, etc.
                pass
            else:
                self.data = {'data': data, 'addr': addr}


class UDPRouter(object):
    """
    Object to manage the multithreaded reception of UDP packets.
    """

    def __init__(self, local_ip, local_port, name=None, blocking=True,
                 timeout=5, rec_buf_size=1024, destination=None, period=0.5):
        """
        Args:
            local_ip (str): Host (local) IP address to listen for UDP
                packets at.
            local_port (int): Local port number to listen for UDP packets at.
            name (Optional[str]): Human-readable name for identification or
                debugging purposes. Defaults to None.
            blocking (Optional[bool]): Optional boolean specifying whether
                socket receive call should be blocking [True] or
                non-blocking [False].Defaults to True.
            timeout (Optional[number]): Optional numerical value which specifies
                the timeout (in seconds) of a blocking socket receive call.
                Set to 0 (or 0.0 or None) for an infinitely blocking call.
                Defaults to 5.
            rec_buf_size (Optional[int]): Integer specifying the maximum
                amount of data to be received at once (in bytes). This
                defaults to 1024. This value will be disregarded if
                destination is specified [see below].
            destination (Optional[dict]): Optional dictionary specifying an
                "address" (string) and a "port" (int). These correspond to the
                destination of packets to be sent. Inclusion of a destination
                keyword argument is used to determine whether the UDPRouter will
                act as a receiver or a sender; if a destination is given,
                this UDPRouter instance will act as a sender.
            period (Optional[num]): Optional number which specifies how often to
                broadcast data (in seconds). This is disregarded if
                destination is not specified [see above]. Defaults to 0.5.
        """
        super(UDPRouter, self).__init__()
        self.ip = local_ip
        self.port = local_port
        self.sock = self._create_socket(blocking, timeout)
        if destination:
            self.destination = destination
            self.period = period
            self.live = UDPSendThread(self.sock, destination, period)
        else:
            self.buf_size = rec_buf_size
            self.live = UDPReceiveThread(self.sock, buf_size=rec_buf_size)

    def _create_socket(self, blocking, timeout):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip, self.port))
        if not blocking:
            # Make socket non-blocking
            sock.setblocking(False)
        else:
            # If blocking, set timeout
            sock.setblocking(True)  # This line shouldn't be necessary.
            # Default behavior is blocking
            sock.settimeout(timeout)
        return sock

    def run(self):
        self.live.start()

    def stop(self):
        self.live.stop()
        self.live.join()


class UDPROS(object):
    """
    Abstract base class for working cooperatively with UDP and ROS.
    Subclasses should initialize a UDPRouter in it's __init__() function and
    must implement a run() method.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, socket_config):
        """
        Args:
            socket_config (dict): Dictionary of socket configuration values.
                The following keys must be included as specified--

                "source_ip" (str): IP address of the UDP packets to listen for.
                "local_port" (int): Local port number to listen for UDP
                    packets at.
                "name" (Optional[str]): Human-readable name for identification
                    or debugging purposes. Defaults to None.
                "blocking" (Optional[bool]): Optional boolean specifying whether
                    socket receive call should be blocking [True] or
                    non-blocking [False].Defaults to True.
                "timeout" (Optional[number]): Optional numerical value which
                    specifies the timeout (in seconds) of a blocking socket
                    receive call. Set to 0 (or 0.0 or None) for an infinitely
                    blocking call. Defaults to 5.
        """
        super(UDPROS, self).__init__()
        # Set default socket configuration values if needed.
        if "blocking" not in socket_config.keys():
            socket_config["blocking"] = True
        if "timeout" not in socket_config.keys():
            socket_config["timeout"] = 5
        if "name" not in socket_config.keys():
            socket_config["name"] = None

        self.socket_conf = socket_config

    @abc.abstractmethod
    def run(self):
        pass


class UDPtoROS(UDPROS):
    """
    Abstract base class for asynchronously publishing ROS topics from incoming
    streams of UDP data. Subclasses must define a broadcast() method which takes
    the UDP data and publishes it to a ROS topic(s). Subclasses must create this
    publisher(s), initialize a node, and call the UDPtoROS __init__() function
    (via super()) in it's __init__() function. The broadcast() method is run
    inside a while loop--the subclass's implementation of broadcast() should
    include some means of limiting the rate of this loop.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, socket_config, receive_buffer_size=1024):
        """
        Args:
            socket_config (dict): Dictionary of socket configuration values.
                The following keys must be included as specified--

                "source_ip" (str): IP address of the UDP packets to listen for.
                "local_port" (int): Local port number to listen for UDP
                    packets at.
                "name" (Optional[str]): Human-readable name for identification
                    or debugging purposes. Defaults to None.
                "blocking" (Optional[bool]): Optional boolean specifying whether
                    socket receive call should be blocking [True] or
                    non-blocking [False].Defaults to True.
                "timeout" (Optional[number]): Optional numerical value which
                    specifies the timeout (in seconds) of a blocking socket
                    receive call. Set to 0 (or 0.0 or None) for an infinitely
                    blocking call. Defaults to 5.
            receive_buffer_size (Optional[int]): Integer specifying the
            maximum amount of data to be received at once (in bytes).
            Defaults to 1024.
        """
        super(UDPtoROS, self).__init__(socket_config)
        self.receiver = UDPRouter(
                self.socket_conf["source_ip"],
                self.socket_conf["local_port"],
                blocking=self.socket_conf["blocking"],
                timeout=self.socket_conf["timeout"],
                rec_buf_size=receive_buffer_size
        )

        rospy.loginfo(
                "Creating UDP Receiver \"{n}\" on port {p} listening to {"
                "a}.".format(
                        p=self.socket_conf["local_port"],
                        a=self.socket_conf["source_ip"],
                        n=self.socket_conf["name"] if self.socket_conf[
                            "name"] else "<anonymous>"
                )
        )

    @abc.abstractmethod
    def broadcast(self, data):
        """
        Abstract method which should receive a dictionary of the received data
        and should publish to a ROS topic(s). This method must also include a
        rate.sleep() or similar rate-limiting call. This should be non-blocking.
        """
        pass

    def run(self):
        self.receiver.run()
        while not rospy.is_shutdown():
            current_data = self.receiver.live.data
            err = self.broadcast(current_data)
            if err == -1:
                break
        self.receiver.stop()
        rospy.loginfo(
                "Stopping UDP Receiver \"{n}\" on port {p} listening to {"
                "a}.".format(
                        p=self.socket_conf["local_port"],
                        a=self.socket_conf["source_ip"],
                        n=self.socket_conf["name"] if self.socket_conf[
                            "name"] else "<anonymous>"
                )
        )


class ROStoUDP(UDPROS):
    """
    Abstract base class for asynchronously publishing UDP data from incoming
    ROS topics. Subclasses must define a broadcast() method which takes
    the UDP data and publishes it to a ROS topic(s). Subclasses must create this
    publisher(s), initialize a node, and call the UDPtoROS __init__() function
    (via super()) in it's __init__() function. The broadcast() method is run
    inside a while loop--the subclass's implementation of broadcast() should
    include some means of limiting the rate of this loop.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, socket_config, destination):
        """
        Args:
            socket_config (dict): Dictionary of socket configuration values.
                The following keys must be included as specified--

                "source_ip" (str): IP address of the UDP packets to listen for.
                "local_port" (int): Local port number to listen for UDP
                    packets at.
                "name" (Optional[str]): Human-readable name for identification
                    or debugging purposes. Defaults to None.
                "blocking" (Optional[bool]): Optional boolean specifying whether
                    socket receive call should be blocking [True] or
                    non-blocking [False].Defaults to True.
                "timeout" (Optional[number]): Optional numerical value which
                    specifies the timeout (in seconds) of a blocking socket
                    receive call. Set to 0 (or 0.0 or None) for an infinitely
                    blocking call. Defaults to 5.
            destination (dict): Optional dictionary specifying an
                "address" (string) and a "port" (int). These correspond to the
                destination of packets to be sent.
        """
        super(ROStoUDP, self).__init__(socket_config)
        self.send_data = ""
        self.destination = destination
        self.sender = UDPRouter(
                self.socket_conf["source_ip"],
                self.socket_conf["local_port"],
                blocking=self.socket_conf["blocking"],
                timeout=self.socket_conf["timeout"],
                destination=destination
        )

        rospy.loginfo(
                "Creating UDP Sender \"{n}\" on local address {la}:{lp} "
                "broadcasting to {ra}:{rp}.".format(
                        la=self.socket_conf["source_ip"],
                        lp=self.socket_conf["local_port"],
                        ra=self.destination["address"],
                        rp=self.destination["port"],
                        n=self.socket_conf["name"] if self.socket_conf[
                            "name"] else "<anonymous>"
                )
        )

    @abc.abstractmethod
    def callback(self, data):
        """
        Abstract method which should be called on receipt of a message from a
        subscribed ROS topic. This must update self.send_data, which the .run()
        method will call periodically to update the data being sent. As
        with most ROS callbacks, this should be non-blocking.
        """
        pass

    def run(self):
        self.sender.run()
        while not rospy.is_shutdown():
            self.sender.live.data = self.send_data  # Get latest data to be sent
        self.sender.stop()
        rospy.loginfo(
                "Stopping UDP Sender \"{n}\" on local address {la}:{lp} "
                "broadcasting to {ra}:{rp}.".format(
                        la=self.socket_conf["source_ip"],
                        lp=self.socket_conf["local_port"],
                        ra=self.destination["address"],
                        rp=self.destination["port"],
                        n=self.socket_conf["name"] if self.socket_conf[
                            "name"] else "<anonymous>"
                )
        )


def main():
    pass


if __name__ == '__main__':
    main()
