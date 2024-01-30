# UDP Send/Receive Example of ROS2rapper with Ethernet

* Send UDP datagrams.
* Receive UDP datagrams arrived at the specified port number.

## Requirements
* Arty A7-100T FPGA board
* Linux machine
  * Ubuntu 22.04 LTS (recommended)
  * Vivado 2023.2
  * Vitis HLS 2023.2

## Build
To run high-level synthesis, logic synthesis and PnR, run following commands.
```
$ make create-proj
$ make synth
```

Then write generated bitstream (ros2rapper-udp/ros2rapper-udp.runs/impl\_1/top.bit) to FPGA.

## Run
### Prepare
* Connect Linux machine and FPGA board through Ethernet.
* Configure Linux machine's IP address to `192.168.1.10`. IP address of ROS2rapper is `192.168.1.100`.

### Test UDP send feature
* This example sends UDP datagrams to port 1234 of `192.168.1.10`.
* To show payload of UDP datagrams arrived at port 1234 of Linux machine, run following command.
  * `nc -u 1234`
* Text "UDP Send Test\n" will be shown periodically.

### Test UDP receive feature
* This example receives only UDP datagrams arrived at port 1234.
* To send UDP datagrams to port 1234 of FPGA, run following command.
  * `nc -ul 192.168.1.100 1234`
* Input any text, then press enter key to send UDP datagram.
* LED 4-8 on FPGA board will be changed when UDP datagram has been arrived.
  * LED 4 is on when datagram length >= 1.
  * LED 5 is on when datagram length >= 5.
  * LED 6 is on when datagram length >= 10.
  * LED 7 is on when datagram length >= 15.
  * Note that a datagram contains newline code.