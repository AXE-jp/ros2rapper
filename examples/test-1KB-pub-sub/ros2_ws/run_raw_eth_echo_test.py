import binascii
import netifaces


def mac_to_bin(mac):
    return binascii.unhexlify(mac.replace(":", ""))


def get_mac(ifname):
    return netifaces.ifaddresses(ifname)[netifaces.AF_LINK][0]["addr"]


if __name__ == "__main__":
    import random
    import socket
    import sys

    if len(sys.argv) < 2:
        ifname = "eth0"
    else:
        ifname = sys.argv[1]

    eth_type_u16 = 0xffff

    src_mac = mac_to_bin(get_mac(ifname))
    dst_mac = mac_to_bin("02:00:00:00:00:00")
    eth_type = eth_type_u16.to_bytes(2, byteorder="big")

    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(eth_type_u16)) as s:
        s.bind((ifname, eth_type_u16))
        for length in (1500, 1499, 1498, 1497):
            payload = random.randbytes(length)
            s.send(dst_mac + src_mac + eth_type + payload)
            received = s.recv(2048)
            dst_is_ok = (received[0:6] == src_mac)
            src_is_ok = (received[6:12] == dst_mac)
            type_is_ok = (received[12:14] == eth_type)
            payload_is_ok = (received[14:] == payload)
            if dst_is_ok and src_is_ok and type_is_ok and payload_is_ok:
                print("OK")
            else:
                print("Bad response")
