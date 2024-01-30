# Topic Publish/Subscribe Example of ROS2rapper with Ethernet

* Publish ROS2 topic and send string messages.
* Subscribe ROS2 topic and receive string messages.

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

### Test ROS2rapper Publisher feature
* This example publishes the "fpga_msg" topic.
* To subscribe this topic, run following command.
  * `./run-subscriber.sh`
  * This script runs subscriber on Docker container.
* Message "Message from FPGA - n" will be shown periodically (Last number 'n' changes from 0 to 9 periodically).

### Test ROS2rapper Subscriber feature
* This example subscribes the "numstr" topic.
* To subscribe this topic, run following command.
  * `./run-publisher.sh`
  * This script runs publisher on Docker container.
* Publisher on Linux machine sends following string messages with a period of 1 sec.
  *  "0", "1", "2", ..., "9", "0", "1", ...
* When ROS2rapper receives message, LED 4-7 is changed according to lower 4 bits of first character of message.