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

### Test ROS2rapper Raw Ether TX feature
* Run the next command in the Linux machine.
```
nc -ul 1234
````

* You may see the message `raw ether test` periodically.

### Test ROS2rapper Publisher feature
* This example publishes the "/bbb" topic.
* To subscribe this topic, run following command.
  * `./run-subscriber.sh`
  * This script runs subscriber on Docker container. This docker container uses a host network.
* Message "Message from FPGA - n" will be shown periodically (Last number 'n' changes from 0 to 9 at intervals of about 1 sec).
