# Example Project of ROS2rapper with Ethernet

* Publish ROS2 topic and send string messages.
* Subscribe ROS2 topic and reveice string messages.

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

Then write generated bitstream (ros2rapper-pubsub/ros2rapper-pubsub.runs/impl\_1/top.bit) to FPGA.

## Run
* Configure IP address to be the same network address. IP address of this example is `192.168.1.100`.
* Subscribe "talker" topic by another node.
* Publish "talker" topic by another node.
