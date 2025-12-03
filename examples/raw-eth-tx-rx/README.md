# Raw Ether TX/RX example of ROS2rapper

* Publish ROS2 topic and send string messages.
* Send raw ether frames (UDP packets)

## Requirements
* Arty A7-100T FPGA board
* Linux machine
  * Ubuntu 22.04 LTS (recommended)
  * Vivado 2023.2
  * Vitis HLS 2023.2
  * Docker

## Configure
Set `dest_mac_addr` and `dest_ip_addr` in `top.v` properly.

## Build
To run high-level synthesis, logic synthesis and PnR, run following commands.
```
$ make create-proj
$ make synth
```

Then write generated bitstream (ros2rapper-pubsub/ros2rapper-pubsub.runs/impl\_1/top.bit) to FPGA.

## Run
### Prepare
* Connect Linux machine and FPGA board through Ethernet.
* Configure Linux machine's IP address to be the same network address of FPGA's. IP address of FPGA is `192.168.1.100`.

### Test raw ether frame send feature

This example sends raw ether (UDP) datagrams to port 1234 of 192.168.1.2 (default).
To show payload of UDP datagrams arrived at port 1234 of Linux machine, run following command.

```
nc -ul 1234
```

Text "raw ether test\n" will be shown periodically.


### Test raw ether fream receive feature

This example receives raw ether frames, excluding ARP packets and most of RTPS packets.
To send UDP datagrams to port 1234 of FPGA, run following command.

```
nc -u 192.168.1.100 1234
```

Input any text, then press enter key to send UDP datagram.
LED 5-7 on FPGA board will be changed when UDP datagram has been arrived.

LED 5 is on when datagram length % 2 == 1.
LED 6 is on when datagram length % 4 == 2 or 3.
LED 7 is on when datagram length % 8 == 4 to 7.
Note that a datagram contains newline code.

### Test ROS2rapper Publisher feature
* This example publishes the "/bbb" topic.
* To subscribe this topic, run following command.
  * `./run-subscriber.sh`
  * This script runs subscriber on Docker container. This docker container uses a host network.
* Message "Message from FPGA - n" will be shown periodically (Last number 'n' changes from 0 to 9 at intervals of about 1 sec).
