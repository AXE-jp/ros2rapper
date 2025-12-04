# Raw Ether TX/RX example of ROS2rapper

* Send raw ether frames (UDP packets).
* Receive raw ether frames (use only UDP packets).
* Publish ROS2 topic and send string messages.

## Requirements
* Arty A7-100T FPGA board
* Linux machine
  * Ubuntu 22.04 LTS (recommended)
  * Vivado 2023.2
  * Vitis HLS 2023.2
  * Docker

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

* This example sends raw ether (UDP) datagrams to port 1234 of 192.168.1.2 (default).
* To show payload of UDP datagrams arrived at port 1234 of Linux machine, run following command.

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
If the text is not begins with "RTPS", LED 4-7 on FPGA board will be changed when UDP datagram has been arrived.

* LED 4 is on when datagram length >= 1.
* LED 5 is on when datagram length >= 5.
* LED 6 is on when datagram length >= 10.
* LED 7 is on when datagram length >= 15.

Note that a datagram contains newline code.

### Test ROS2rapper Publisher feature
* This example publishes the "/bbb" topic.
* To subscribe this topic, run following command.
  * `./run-subscriber.sh`
  * This script runs subscriber on Docker container. This docker container uses a host network.
* Message "Message from FPGA - n" will be shown periodically (Last number 'n' changes from 0 to 9 at intervals of about 1 sec).
