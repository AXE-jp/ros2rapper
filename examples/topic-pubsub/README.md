# Example Project of ROS2rapper with Ethernet

* Publish ROS2 topic and send string messages.
* Subscribe ROS2 topic and reveice string messages.

## Requirements
* Arty A7-100T
* Vivado 2023.2
* Requirements for ROS2rapper (See README in ros2rapper repository)

## Build
To synthesize ROS2rapper and create Vivado project, run following commands.
```
$ make -C .. vitis
$ make create-proj
```

Then open ros2rapper-project/ros2rapper-project.xpr in Vivado and run "Generate bitstream".

## Run
* Configure IP address to be the same network address. IP address of this example is `192.168.1.100`.
* Subscribe "talker" topic by another node.
* Publish "talker" topic by another node.
