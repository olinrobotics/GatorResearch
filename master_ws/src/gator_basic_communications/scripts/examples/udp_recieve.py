# udp_recieve.py
# A simple UDP listener.
# Adapted from https://wiki.python.org/moin/UdpCommunication
import socket

UDP_IP = "192.168.1.10"
UDP_PORT = 5012

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
# sock.setblocking(False)
sock.settimeout(5)
print "STARTING:"
while True:
    try:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    except socket.error, e:
        pass
    else:
        print "received message:", data
        print addr