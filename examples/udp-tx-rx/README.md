# UDP Send/Receive Example of ROS2rapper with Ethernet

* Send UDP datagrams.
* Receive UDP datagrams to the specified port number.

## Requirements
* Arty A7-100T
* Vivado 2023.2
* Vitis HLS 2023.2

## Build
To synthesize ROS2rapper and create Vivado project, run following commands.
```
$ make create-proj
$ make synth
```

Then write generated bitstream (ros2rapper-udp/ros2rapper-udp.runs/impl\_1/top.bit) to FPGA.

## Run
* Configure IP address to be the same network address. IP address of this example is `192.168.1.100`.
