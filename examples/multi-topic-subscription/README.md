# Multi-Topic Subscribe Example of ROS2rapper with Ethernet

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

Then write generated bitstream (ros2rapper-multi-topic-sub/ros2rapper-multi-topic-sub.runs/impl\_1/top.bit) to FPGA.

## Run
### Prepare
* Connect Linux machine and FPGA board through Ethernet.
* Configure Linux machine's IP address to be the same network address of FPGA's. IP address of FPGA is `192.168.1.100`.

### Test ROS2rapper Publisher feature
* This example publishes the "/bbb" topic.
* To subscribe this topic, run following command.
  * `./run-subscriber.sh`
  * This script runs subscriber on Docker container. This docker container uses a host network.
* Message "Message from FPGA - n" will be shown periodically (Last number 'n' changes from 0 to 9 at intervals of about 1 sec).

### Test ROS2rapper Subscriber feature
* This example subscribes the "/aaa", "/ccc", "/ddd" or "/eee" topic.
  * You can enable these topics by switches on Arty A7.
    * If SW0 is set 1, the topic "/aaa" is enabled.
    * If SW1 is set 1, the topic "/ccc" is enabled.
    * If SW2 is set 1, the topic "/ddd" is enabled.
    * If SW3 is set 1, the topic "/eee" is enabled.
  * You may have to reset ROS2rapper after you enable or disable these topics.
* To subscribe this topic, run following command.
  * `./run-publisher.sh <command>`
  * This script runs a publisher on Docker container. This docker container uses a host network.
  * If <command> is "0" or not given, the publisher uses the topic "/aaa".
    If <command> is "1", the publisher uses "/ccc".
    If <command> is "2", the publisher uses "/ddd".
    If <command> is "3", the publisher uses "/eee".
    If <command> is "all", four publishers are created, and they uses "/aaa", "/ccc", "/ddd" and "/eee".
* Publisher on Linux machine sends following string messages with a period of 1 sec.
  * topic "/aaa": "A", "AA", "AAA", ...
  * topic "/ccc": "C", "CC", "CCC", ...
  * topic "/ddd": "D", "DD", "DDD", ...
  * topic "/eee": "E", "EE", "EEE", ...
* When ROS2rapper receives a message of topic "/aaa", LED 4-7 is changed according to lower 4 bits of first character of message.
  When ROS2rapper receives a message of topic "/ccc", blue ones of LED 0-3 is changed.
  When ROS2rapper receives a message of topic "/ddd", green ones of LED 0-3 is changed.
  When ROS2rapper receives a message of topic "/eee", red ones of LED 0-3 is changed.
