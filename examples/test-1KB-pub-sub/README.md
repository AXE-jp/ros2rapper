# Publish/Subscribe 1KB messages by ROS2rapper with Ethernet

* Publish ROS2 topic and send 1KB messages.
* Subscribe ROS2 topic and receive 1KB messages.
* Echo back raw ether packets

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

Then write generated bitstream (ros2rapper-1KB/ros2rapper-1KB.runs/impl\_1/top.bit) to FPGA.

## Run
### Prepare
* Connect Linux machine and FPGA board through Ethernet.
* Configure Linux machine's IP address to be the same network address of FPGA's. IP address of FPGA is `192.168.1.100`.

### Test ROS2rapper Subscriber feature
* This example subscribes the "/sample_topic" topic.
* To subscribe this topic, run following command.
  * `./run-pub.sh`
  * This script runs publisher on Docker container. This docker container uses a host network.
* Publisher on the Linux machine sends a message of type `Uint16x512` (see below) with a period of 1 sec.
* When ROS2rapper receives a message with correct payload (see below), LED 4 lights up.
  When ROS2rapper receives a message with correct payload (see below), LED 5 lights up.

### Test ROS2rapper Publisher feature
* This example publishes the "/sample_topic" topic.
* To subscribe this topic, run following command.
  * `./run-sub.sh`
  * This script runs subscriber on Docker container. This docker container uses a host network.
* When the subscriber on the Linux machine receives a message with correct payload, a message "Recieved n: Valid" is shown.
  When the subscriber receives a message with wrong payload, a message "Recieved n: Invalid" is shown.

### Payload of messages
This example uses the custom interface "Uint16x512". It is defined in `ros2_ws/src/sample_msgs/msg/Uint16x512.msg`.
The publishers in this example sends uint16 array of length 512 (say `data[512]`),
and it is expected that `data[i] == data[0] + i` for i = 1, ..., 511.

### Echo raw ether packet
* This example send a raw ether frame of type 0xffff from PC, and the ROS2rapper echoes back it.
* To send raw ether frame from PC, run the following command.
  * `./run-raw-eth.sh NETWORK_INTERFACE_NAME`
    * If NETWORK_INTERFACE_NAME is not supplied, "eth0" is used.
  * This script runs a python program on a Docker container. This docker container uses a host network.
* When the program on the Linux machine receives the echo with correct payload, a message "OK" is shown.
  When the program receives a message with wrong payload, a message "Bad response" is shown.
