# ros2rapper-ether
This repository contains Ethernet MAC component (from [alexforencich/verilog-ethernet](https://github.com/alexforencich/verilog-ethernet)) and adapter for ROS2rapper.

## Requirements
Following softwares are required to build example project in addition to ROS2rapper's requirements.
* Vivado 2023.2

## Try
There is an example project under `example` directory. You can try ROS2rapper on Arty A7-100 FPGA board.

1. Run following commands to synthesize ROS2rapper and create Vivado project.
```
$ make vitis
$ cd example
$ make create-project
```



## License
See file header (LGPL or MIT).