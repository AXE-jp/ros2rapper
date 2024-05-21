# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2

add_files hls/app.cpp
add_files hls/checksum.cpp
add_files hls/ip.cpp
add_files hls/ros2.cpp
add_files hls/rtps.cpp
add_files hls/sedp.cpp
add_files hls/slip.cpp
add_files hls/spdp.cpp
add_files hls/udp.cpp

set_top ros2

open_solution -reset solution1

set_part xc7a100tcsg324-1
config_rtl -reset all -reset_level low -reset_async
create_clock -period 50MHz

csynth_design
#export_design -format ip_catalog

exit
