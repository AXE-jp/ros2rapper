import socket
import struct


def receive_udp(multicast_addr, port):
    """
    Receive UDP packets on the designated port.
    This program receives  UDP packets which are sent
    to the host IP address (unicast), the designated multicast address,
    the direct broadcast address or the limited broadcast address.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as s:
        # Allow the port reuse
        s.setsockpot(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Receive broadcast packets
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(("0.0.0.0", port))
        # Join the multicast group
        mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        # Receive UDP packets
        while True:
            data, addr = sock.recvfrom(2048)
            print(data.decode("utf-8"))


if __name__ == "__main__":
    receive_udp("239.1.1.1", 1234)
